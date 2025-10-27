%% OFDM Receiver Main Script - OPTIMIZED FOR LIVE MODE
% Continuously receives and decodes OFDM signals from ADALM PLUTO SDR

clear; close all; clc;

%% ============ USER CONFIGURATION ============
RX_GAIN = 60;                    % MAX RX gain for weak signals
FRAME_SIZE = 16384;              % LARGER buffer to reduce overflows
DETECTION_THRESHOLD = 0.75;       % LOWER threshold for weak signals
DEBUG_PLOTS = false;             % DISABLE - causes major slowdown
CONTINUOUS_MODE = true;          
MAX_ITERATIONS = 1000;           % Limit for testing

% Display options
SHOW_CONSTELLATION = false;      % DISABLE live plots - too slow
SHOW_SPECTRUM = false;           
UPDATE_RATE = 50;                % Update less frequently

% Processing options
MIN_SNR_THRESHOLD = 5.0;         % Only decode if SNR > 5 dB
SKIP_FRAMES = 5;                 % Process every Nth frame only

%% ============ LOAD TX CONFIGURATION ============
fprintf('\n========================================\n');
fprintf('OFDM RECEIVER - OPTIMIZED LIVE MODE\n');
fprintf('========================================\n');

if ~exist('ofdm_tx_config.mat', 'file')
    error('Configuration file not found. Run ofdm_tx_main.m first!');
end

fprintf('Loading TX configuration...\n');
configData = load('ofdm_tx_config.mat');
config = configData.config;

if exist('tx_metadata.mat', 'file')
    metaData = load('tx_metadata.mat');
    txMetadata = metaData.txMetadata;
    fprintf('Expected message: "%s"\n', txMetadata.message);
    CENTER_FREQ = txMetadata.centerFreq;
    SAMPLE_RATE = txMetadata.sampleRate;
else
    warning('Metadata file not found.');
    CENTER_FREQ = 915e6;
    SAMPLE_RATE = 2e6;
    txMetadata = [];
end

%% ============ CREATE OFDM HELPER ============
ofdm = OFDMHelper(config.Nc, config.Ndata, config.Npilot, ...
                  config.cpLength, config.modulationOrder, ...
                  config.nPayloadSymbols, config.bbSampleRate, ...
                  config.ovsamplingFactor);

requiredLength = ofdm.frameLength * ofdm.ovsamplingFactor * 2;

%% ============ SDR CONFIGURATION ============
fprintf('\n--- SDR Configuration ---\n');

sdr = SDRHelper('sampleRate', SAMPLE_RATE, ...
                'centerFrequency', CENTER_FREQ, ...
                'rxGain', RX_GAIN, ...
                'rxFrameLength', FRAME_SIZE);

sdr.connect();
sdr.configureRx();
fprintf('✓ Receiver configured\n');

%% ============ LIVE RECEPTION LOOP ============
fprintf('\n========================================\n');
fprintf('LIVE RECEPTION - OPTIMIZED\n');
fprintf('========================================\n');
fprintf('RX Gain: %.1f dB (MAX)\n', RX_GAIN);
fprintf('Frame Size: %d samples\n', FRAME_SIZE);
fprintf('Processing: Every %d frames\n', SKIP_FRAMES);
fprintf('\n📡 RECEIVING...\n');
fprintf('Press Ctrl+C to stop\n\n');

% Statistics
stats = struct();
stats.totalFrames = 0;
stats.processedFrames = 0;
stats.successfulDecodes = 0;
stats.failedDecodes = 0;
stats.totalOverflows = 0;
stats.messages = {};
stats.SNRs = [];

% Buffer
rxBuffer = [];
bufferMaxSize = requiredLength * 3;

iteration = 0;
frameCounter = 0;
startTime = tic;

% Suppress warnings during loop
warning('off', 'SDRHelper:RxOverflow');

fprintf('%-6s | %-10s | %-8s | %-8s | %-40s\n', 'Frame', 'Time', 'SNR(dB)', 'BER', 'Message');
fprintf('%s\n', repmat('-', 1, 80));

while iteration < MAX_ITERATIONS
    iteration = iteration + 1;
    
    % Receive frame (no overflow check to speed up)
    rxFrame = sdr.rxSDR();
    
    % Append to buffer
    rxBuffer = [rxBuffer; rxFrame];
    
    % Trim buffer
    if length(rxBuffer) > bufferMaxSize
        rxBuffer = rxBuffer(end-bufferMaxSize+1:end);
    end
    
    frameCounter = frameCounter + 1;
    
    % Only process every SKIP_FRAMES frames to reduce CPU load
    if frameCounter >= SKIP_FRAMES && length(rxBuffer) >= requiredLength
        frameCounter = 0;
        stats.processedFrames = stats.processedFrames + 1;
        
        % Quick SNR check before full decode
        signalPower = mean(abs(rxBuffer).^2);
        if 10*log10(signalPower) < -60
            % Signal too weak, skip processing
            continue;
        end
        
        % Attempt decode
        [decoded, success, rxMetrics] = attemptDecode(ofdm, rxBuffer, ...
                                                      DETECTION_THRESHOLD, ...
                                                      txMetadata, config, ...
                                                      MIN_SNR_THRESHOLD);
        
        if success
            stats.successfulDecodes = stats.successfulDecodes + 1;
            stats.messages{end+1} = decoded.message;
            stats.SNRs(end+1) = decoded.SNR;
            
            % Only print if SNR is reasonable
            if decoded.SNR > MIN_SNR_THRESHOLD
                fprintf('%6d | %10s | %8.2f | %8.2e | %s\n', ...
                        iteration, ...
                        datestr(now, 'HH:MM:SS'), ...
                        decoded.SNR, ...
                        decoded.BER, ...
                        decoded.message);
            end
            
            rxBuffer = []; % Clear after successful decode
        else
            stats.failedDecodes = stats.failedDecodes + 1;
        end
    end
    
    stats.totalFrames = iteration;
    
    % Status update
    if mod(iteration, 500) == 0
        elapsedTime = toc(startTime);
        fprintf('\n[Status] Frames: %d | Processed: %d | Success: %d | Rate: %.1f fps | Overflows: %d\n\n', ...
                stats.totalFrames, stats.processedFrames, stats.successfulDecodes, ...
                stats.totalFrames/elapsedTime, sdr.rxOverflowCount);
    end
end

warning('on', 'SDRHelper:RxOverflow');

%% ============ CLEANUP ============
fprintf('\n\n--- Stopping Reception ---\n');
sdr.cleanup();

fprintf('\n========================================\n');
fprintf('RECEPTION SUMMARY\n');
fprintf('========================================\n');
fprintf('Total Frames: %d\n', stats.totalFrames);
fprintf('Processed Frames: %d\n', stats.processedFrames);
fprintf('Successful Decodes: %d\n', stats.successfulDecodes);
fprintf('Total Overflows: %d\n', sdr.rxOverflowCount);
fprintf('Runtime: %.2f seconds\n', toc(startTime));

if ~isempty(stats.SNRs)
    fprintf('\nAverage SNR: %.2f dB\n', mean(stats.SNRs));
    fprintf('Valid Messages: %d\n', sum(stats.SNRs > MIN_SNR_THRESHOLD));
end

%% ============ HELPER FUNCTIONS ============

function [decoded, success, rxMetrics] = attemptDecode(ofdm, rxSignal, threshold, txMetadata, config, minSNR)
    
    decoded = struct();
    decoded.message = '';
    decoded.SNR = NaN;
    decoded.BER = NaN;
    success = false;
    rxMetrics = [];
    
    referenceData = {};
    if ~isempty(txMetadata) && isfield(txMetadata, 'payloadData')
        referenceData = txMetadata.payloadData;
    end
    
    % Decode with NO debug plots (faster)
    [decodedBits, rxMetrics] = ofdm.receiveFrame(rxSignal, ...
                                                  'debug', false, ...
                                                  'threshold', threshold, ...
                                                  'referenceData', referenceData);
    
    if isempty(decodedBits) || rxMetrics.SNR_est < minSNR
        return;
    end
    
    % Bit to symbol conversion
    allDecodedBits = [];
    bitsPerModSymbol = log2(config.modulationOrder);
    
    for i = 1:length(decodedBits)
        symbols = decodedBits{i};
        bitString = [];
        for j = 1:length(symbols)
            bits = de2bi(symbols(j), bitsPerModSymbol, 'left-msb');
            bitString = [bitString, bits];
        end
        allDecodedBits = [allDecodedBits, bitString];
    end
    
    if ~isempty(txMetadata) && isfield(txMetadata, 'paddingBits')
        validBits = allDecodedBits(1:end-txMetadata.paddingBits);
    else
        validBits = allDecodedBits;
    end
    
    decodedMessage = binary2string(validBits);
    
    if length(decodedMessage) > 0 && sum(isstrprop(decodedMessage, 'print')) > length(decodedMessage)*0.5
        decoded.message = decodedMessage;
        decoded.SNR = rxMetrics.SNR_est;
        decoded.BER = rxMetrics.BER;
        success = true;
    end
end

function str = binary2string(binaryArray)
    remainder = mod(length(binaryArray), 8);
    if remainder ~= 0
        binaryArray = [binaryArray, zeros(1, 8-remainder)];
    end
    
    numBytes = length(binaryArray) / 8;
    str = '';
    
    for i = 1:numBytes
        startIdx = (i-1)*8 + 1;
        endIdx = i*8;
        byte = binaryArray(startIdx:endIdx);
        asciiValue = bi2de(byte, 'left-msb');
        
        if asciiValue >= 32 && asciiValue <= 126
            str = [str, char(asciiValue)];
        elseif asciiValue == 0
            break;
        end
    end
end
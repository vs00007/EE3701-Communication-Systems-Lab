%% OFDM Transmitter Main Script
% Transmits a text message using ADALM PLUTO SDR and OFDM modulation
% Author: [Your Name]
% Date: [Date]

clear; close all; clc;

%% ============ USER CONFIGURATION ============
MESSAGE = 'Hello OFDM World! Testing 123.';  % Your message here
CENTER_FREQ = 915e6;                          % 915 MHz (ISM band)
SAMPLE_RATE = 2e6;                            % 2 MHz baseband sample rate
TX_GAIN = -20;                                % TX gain in dB
TRANSMIT_MODE = 'repeat';                     % 'single' or 'repeat'
REPEAT_DURATION = 30;                         % seconds (if repeat mode)

% OFDM Parameters
Nc = 64;                    % FFT size
Ndata = 48;                 % Data subcarriers
Npilot = 4;                 % Pilot subcarriers
cpLength = 16;              % Cyclic prefix length
modulationOrder = 4;        % 4-PSK (2 bits per symbol)
ovsamplingFactor = 2;       % Oversampling factor

%% ============ MESSAGE ENCODING ============
fprintf('\n========================================\n');
fprintf('OFDM TRANSMITTER INITIALIZATION\n');
fprintf('========================================\n');

% Convert string to binary
messageBinary = string2binary(MESSAGE);
fprintf('Message: "%s"\n', MESSAGE);
fprintf('Message length: %d characters\n', length(MESSAGE));
fprintf('Binary length: %d bits\n', length(messageBinary));

% Calculate number of OFDM symbols needed
bitsPerSymbol = Ndata * log2(modulationOrder);
nPayloadSymbols = ceil(length(messageBinary) / bitsPerSymbol);
fprintf('Bits per OFDM symbol: %d\n', bitsPerSymbol);
fprintf('OFDM symbols required: %d\n', nPayloadSymbols);

% Pad message to fill all symbols
totalBitsNeeded = nPayloadSymbols * bitsPerSymbol;
paddingBits = totalBitsNeeded - length(messageBinary);
messageBinaryPadded = [messageBinary, zeros(1, paddingBits)];
fprintf('Padding bits added: %d\n', paddingBits);

%% ============ OFDM MODULATION ============
fprintf('\n--- OFDM Modulation ---\n');

% Create OFDM helper
ofdm = OFDMHelper(Nc, Ndata, Npilot, cpLength, modulationOrder, ...
                  nPayloadSymbols, SAMPLE_RATE, ovsamplingFactor);

% Print OFDM configuration
ofdm.printSummary();

% Convert bits to symbols (Gray coded)
payloadData = cell(1, nPayloadSymbols);
for i = 1:nPayloadSymbols
    startIdx = (i-1) * bitsPerSymbol + 1;
    endIdx = i * bitsPerSymbol;
    symbolBits = messageBinaryPadded(startIdx:endIdx);
    
    % Group bits into log2(M) bit groups and convert to symbols
    bitsPerModSymbol = log2(modulationOrder);
    numModSymbols = bitsPerSymbol / bitsPerModSymbol;
    symbols = zeros(1, numModSymbols);
    
    for j = 1:numModSymbols
        bitStartIdx = (j-1) * bitsPerModSymbol + 1;
        bitEndIdx = j * bitsPerModSymbol;
        bitGroup = symbolBits(bitStartIdx:bitEndIdx);
        symbols(j) = bi2de(bitGroup, 'left-msb');
    end
    
    payloadData{i} = symbols;
end

% Generate OFDM frame
fprintf('Generating OFDM frame...\n');
txFrame = ofdm.generateFrame(payloadData{:});
fprintf('Frame generated: %d samples\n', length(txFrame));

% Apply pulse shaping
fprintf('Applying pulse shaping...\n');
txSignal = ofdm.applyPulseShaping(txFrame);
fprintf('TX signal length: %d samples (%.2f ms)\n', ...
    length(txSignal), length(txSignal) * ofdm.Ts * 1000);

% Calculate PAPR
instantPower = abs(txSignal).^2;
avgPower = mean(instantPower);
peakPower = max(instantPower);
PAPR = 10*log10(peakPower/avgPower);
fprintf('PAPR: %.2f dB\n', PAPR);

%% ============ SAVE CONFIGURATION ============
fprintf('\n--- Saving Configuration ---\n');

% Save OFDM configuration for receiver
ofdm.saveConfig('ofdm_tx_config.mat');

% Save transmission metadata
txMetadata = struct();
txMetadata.message = MESSAGE;
txMetadata.messageBinary = messageBinary;
txMetadata.paddingBits = paddingBits;
txMetadata.payloadData = payloadData;
txMetadata.centerFreq = CENTER_FREQ;
txMetadata.sampleRate = SAMPLE_RATE;
txMetadata.txGain = TX_GAIN;
txMetadata.timestamp = datetime('now');
save('tx_metadata.mat', 'txMetadata');
fprintf('Metadata saved to tx_metadata.mat\n');

%% ============ VISUALIZE TX SIGNAL ============
fprintf('\n--- Visualizing TX Signal ---\n');

% Plot transmitter analysis
ofdm.plotTxSignal(txSignal, payloadData);

% Plot frequency domain
ofdm.plotFrequencyDomain(txFrame);

% Visualize frame structure
ofdm.visualizeFrame(txFrame);

%% ============ SDR CONFIGURATION ============
fprintf('\n--- SDR Configuration ---\n');

% Create SDR helper
sdr = SDRHelper('sampleRate', SAMPLE_RATE, ...
                'centerFrequency', CENTER_FREQ, ...
                'txGain', TX_GAIN, ...
                'txFrameLength', length(txSignal));

% Connect to hardware
fprintf('Connecting to ADALM PLUTO...\n');
try
    sdr.connect();
    fprintf('✓ Connection successful\n');
catch ME
    fprintf('✗ Connection failed: %s\n', ME.message);
    fprintf('Please check:\n');
    fprintf('  1. PLUTO is connected via USB\n');
    fprintf('  2. Drivers are installed\n');
    fprintf('  3. Device is recognized by MATLAB\n');
    return;
end

% Configure transmitter
fprintf('Configuring transmitter...\n');
sdr.configureTx();
fprintf('✓ Transmitter configured\n');

%% ============ TRANSMIT ============
fprintf('\n========================================\n');
fprintf('TRANSMISSION\n');
fprintf('========================================\n');
fprintf('Center Frequency: %.2f MHz\n', CENTER_FREQ/1e6);
fprintf('Sample Rate: %.2f MHz\n', SAMPLE_RATE/1e6);
fprintf('TX Gain: %.1f dB\n', TX_GAIN);
fprintf('Signal Length: %d samples\n', length(txSignal));
fprintf('Mode: %s\n', TRANSMIT_MODE);

if strcmp(TRANSMIT_MODE, 'repeat')
    fprintf('\n⚠ WARNING: Ensure you have proper license for transmission!\n');
    fprintf('⚠ Use appropriate antennas and filtering.\n');
    fprintf('⚠ Transmission will repeat for %d seconds.\n\n', REPEAT_DURATION);
    
    response = input('Start transmission? (y/n): ', 's');
    if ~strcmpi(response, 'y')
        fprintf('Transmission cancelled.\n');
        sdr.cleanup();
        return;
    end
    
    % Transmit in repeat mode
    fprintf('\n🔊 TRANSMITTING...\n');
    fprintf('Press Ctrl+C to stop early\n\n');
    
    [success, underflow] = sdr.transmit(txSignal, 'repeat', true, ...
                                        'duration', REPEAT_DURATION);
    
    if success
        fprintf('\n✓ Transmission completed successfully\n');
    else
        fprintf('\n✗ Transmission failed\n');
    end
    
else
    % Single transmission
    fprintf('\n🔊 Transmitting once...\n');
    [success, underflow] = sdr.transmit(txSignal, 'repeat', false);
    
    if success
        fprintf('✓ Transmission completed\n');
        if underflow
            fprintf('⚠ Warning: TX underflow occurred\n');
        end
    else
        fprintf('✗ Transmission failed\n');
    end
end

%% ============ CLEANUP ============
fprintf('\n--- Cleanup ---\n');
sdr.cleanup();
fprintf('SDR released\n');

fprintf('\n========================================\n');
fprintf('TRANSMISSION COMPLETE\n');
fprintf('========================================\n');
fprintf('Configuration saved for receiver:\n');
fprintf('  - ofdm_tx_config.mat\n');
fprintf('  - tx_metadata.mat\n');
fprintf('\nRun ofdm_rx_main.m to receive and decode.\n\n');

%% ============ HELPER FUNCTIONS ============
function binaryArray = string2binary(str)
    % Convert string to binary array
    asciiValues = double(str);
    binaryStrings = dec2bin(asciiValues, 8);
    binaryArray = [];
    for i = 1:size(binaryStrings, 1)
        binaryArray = [binaryArray, str2num(binaryStrings(i,:).').'];
    end
end
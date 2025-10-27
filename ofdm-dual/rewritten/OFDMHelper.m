% classdef OFDMHelper
%     properties
%         % Minimum Independent Parameters
%         Nc;
%         Ndata;
%         Npilot;
%         cpLength;
%         modulationOrder;
%         nPayloadSymbols;
        
%         % Derived Parameters
%         Nvirtual;
%         pilotIndices;
%         dataIndices;
%         nullIndices;
        
%         % Fixed sequences 
%         shortPreambleSeq;
%         longPreambleSeq;
%         pilotSeq;
        
%         ovsamplingFactor;
%         bbSampleRate;
        
%         % Timing parameters (derived)
%         shortPreambleLength;
%         longPreambleLength;
%         payloadSymbolLength;
%         frameLength;
%         Ts; % Sample time
%     end
    
%     methods
%         function obj = OFDMHelper(Nc, Ndata, Npilot, cpLength, modulationOrder, nPayloadSymbols, bbSampleRate, ovsamplingFactor)
%             if nargin < 8
%                 obj.Nc = 64;
%                 obj.Ndata = 48;
%                 obj.Npilot = 4;
%                 obj.cpLength = 16;
%                 obj.modulationOrder = 4;
%                 obj.nPayloadSymbols = 2;
%                 obj.ovsamplingFactor = 2;
%                 obj.bbSampleRate = 1e6;
%             else
%                 obj.Nc = Nc;
%                 obj.Ndata = Ndata;
%                 obj.Npilot = Npilot;
%                 obj.cpLength = cpLength;
%                 obj.modulationOrder = modulationOrder;
%                 obj.nPayloadSymbols = nPayloadSymbols;
%                 obj.ovsamplingFactor = ovsamplingFactor;
%                 obj.bbSampleRate = bbSampleRate;
%             end
            
%             obj = obj.deriveParameters();
%             obj = obj.calculateTimingParameters();
            
%             % Fixed sequences [from the 802.11a standard]
%             obj.shortPreambleSeq = sqrt(13/6)*[0,0,1+1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,-1-1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,0,0,0,0,-1-1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,1+1j,0,0,0,1+1j,0,0,0,1+1j,0,0];
%             obj.longPreambleSeq = [1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1];
%             obj.pilotSeq = repmat([1,1,1,-1], 1, ceil(obj.Npilot/4)); 
%             obj.pilotSeq = obj.pilotSeq(1:obj.Npilot);
%         end
        
%         function obj = deriveParameters(obj)
%             % Calculate number of virtual subcarriers
%             obj.Nvirtual = obj.Nc - obj.Ndata - obj.Npilot - 1; % -1 for DC, as it is handled later.
            
%             % Standard subcarrier allocation
%             virtualLeft = floor(obj.Nvirtual/2);
%             virtualRight = obj.Nvirtual - virtualLeft;
            
%             % DC is at center (index Nc/2 + 1 in 1-indexed)
%             dcIndex = obj.Nc/2 + 1;
            
%             obj.nullIndices = [1:virtualLeft, dcIndex, (obj.Nc-virtualRight+1):obj.Nc]; % [1 x (Nvirtual + 1)]
            
%             activeIndices = setdiff(1:obj.Nc, obj.nullIndices);
            
%             % Distribute pilots evenly among active subcarriers
%             pilotSpacing = floor(length(activeIndices) / obj.Npilot);
%             obj.pilotIndices = activeIndices(1:pilotSpacing:obj.Npilot*pilotSpacing);
%             obj.pilotIndices = obj.pilotIndices(1:obj.Npilot); 
            
%             % Remaining are data subcarriers
%             obj.dataIndices = setdiff(activeIndices, obj.pilotIndices);
%             obj.dataIndices = obj.dataIndices(1:obj.Ndata);
%         end
        
%         function obj = calculateTimingParameters(obj)
%             % Calculate frame timing parameters
%             obj.Ts = 1 / (obj.bbSampleRate * obj.ovsamplingFactor);
            
%             % Short preamble: 10 repetitions of 16 samples each
%             obj.shortPreambleLength = 160; % Fixed for 802.11a-style
            
%             % Long preamble: 32 CP + 64 + 64 samples
%             obj.longPreambleLength = 160; % Fixed for 802.11a-style
            
%             % Payload symbol: CP + FFT size
%             obj.payloadSymbolLength = obj.cpLength + obj.Nc;
            
%             % Total frame length
%             obj.frameLength = obj.shortPreambleLength + obj.longPreambleLength + ...
%                              obj.nPayloadSymbols * obj.payloadSymbolLength;
%         end
        
%         function saveConfig(obj, filename)
%             % Save all parameters needed for RX to a .mat file
%             if nargin < 2
%                 filename = 'ofdm_config.mat';
%             end
            
%             % Create configuration structure
%             config = struct();
            
%             % Basic parameters
%             config.Nc = obj.Nc;
%             config.Ndata = obj.Ndata;
%             config.Npilot = obj.Npilot;
%             config.cpLength = obj.cpLength;
%             config.modulationOrder = obj.modulationOrder;
%             config.nPayloadSymbols = obj.nPayloadSymbols;
%             config.ovsamplingFactor = obj.ovsamplingFactor;
%             config.bbSampleRate = obj.bbSampleRate;
            
%             % Derived parameters
%             config.Nvirtual = obj.Nvirtual;
%             config.pilotIndices = obj.pilotIndices;
%             config.dataIndices = obj.dataIndices;
%             config.nullIndices = obj.nullIndices;
            
%             % Sequences
%             config.shortPreambleSeq = obj.shortPreambleSeq;
%             config.longPreambleSeq = obj.longPreambleSeq;
%             config.pilotSeq = obj.pilotSeq;
            
%             % Timing parameters
%             config.Ts = obj.Ts;
%             config.shortPreambleLength = obj.shortPreambleLength;
%             config.longPreambleLength = obj.longPreambleLength;
%             config.payloadSymbolLength = obj.payloadSymbolLength;
%             config.frameLength = obj.frameLength;
            
%             % Generate reference sequences for RX
%             config.longPreambleSlotFrequency = obj.generateLongPreambleFreqDomain();
            
%             % Save to file
%             save(filename, 'config');
%             fprintf('Configuration saved to %s\n', filename);
%         end
        
%         function obj = loadConfig(obj, filename)
%             % Load configuration from .mat file
%             if nargin < 2
%                 filename = 'ofdm_config.mat';
%             end
            
%             if ~exist(filename, 'file')
%                 error('Configuration file %s not found', filename);
%             end
            
%             data = load(filename);
%             config = data.config;
            
%             % Load all parameters
%             obj.Nc = config.Nc;
%             obj.Ndata = config.Ndata;
%             obj.Npilot = config.Npilot;
%             obj.cpLength = config.cpLength;
%             obj.modulationOrder = config.modulationOrder;
%             obj.nPayloadSymbols = config.nPayloadSymbols;
%             obj.ovsamplingFactor = config.ovsamplingFactor;
%             obj.bbSampleRate = config.bbSampleRate;
            
%             obj.Nvirtual = config.Nvirtual;
%             obj.pilotIndices = config.pilotIndices;
%             obj.dataIndices = config.dataIndices;
%             obj.nullIndices = config.nullIndices;
            
%             obj.shortPreambleSeq = config.shortPreambleSeq;
%             obj.longPreambleSeq = config.longPreambleSeq;
%             obj.pilotSeq = config.pilotSeq;
            
%             obj.Ts = config.Ts;
%             obj.shortPreambleLength = config.shortPreambleLength;
%             obj.longPreambleLength = config.longPreambleLength;
%             obj.payloadSymbolLength = config.payloadSymbolLength;
%             obj.frameLength = config.frameLength;
            
%             fprintf('Configuration loaded from %s\n', filename);
%         end
        
%         function longPreambleFreq = generateLongPreambleFreqDomain(obj)
%             % Generate the frequency domain long preamble for RX reference
%             virtualSubcarrier = zeros(1, obj.Nc - length(obj.longPreambleSeq));
%             longPreambleFreq = [virtualSubcarrier(1:6), obj.longPreambleSeq, virtualSubcarrier(7:11)];
%         end
        
%         function shortPreamble = generateShortPreamble(obj)
%             virtualSubcarrier = zeros(1, obj.Nc - length(obj.shortPreambleSeq));
%             shortPreambleSlotFreq = [virtualSubcarrier(1:6), obj.shortPreambleSeq, virtualSubcarrier(7:11)];
%             shortPreambleSlotTime = ifft(ifftshift(shortPreambleSlotFreq));
%             shortPreamble = repmat(shortPreambleSlotTime(1:16), 1, 10);
%         end
        
%         function longPreamble = generateLongPreamble(obj)
%             virtualSubcarrier = zeros(1, obj.Nc - length(obj.longPreambleSeq));
%             longPreambleSlotFreq = [virtualSubcarrier(1:6), obj.longPreambleSeq, virtualSubcarrier(7:11)];
%             longPreambleSlotTime = ifft(ifftshift(longPreambleSlotFreq));
%             longPreamble = [longPreambleSlotTime(obj.Nc/2+1:obj.Nc), longPreambleSlotTime, longPreambleSlotTime];
%         end
        
%         function payloadSymbol = generatePayloadSymbol(obj, dataBits)
%             if obj.modulationOrder == 2
%                 dataModulated = pskmod(dataBits, obj.modulationOrder, 0, "gray");
%             else
%                 dataModulated = pskmod(dataBits, obj.modulationOrder, pi/4, "gray");
%             end
            
%             freqSymbol = zeros(1, obj.Nc);
%             freqSymbol(obj.dataIndices(1:length(dataModulated))) = dataModulated;
%             freqSymbol(obj.pilotIndices) = obj.pilotSeq;
%             freqSymbol(obj.nullIndices) = 0;
            
%             % Convert to time domain and add CP
%             timeSymbol = ifft(ifftshift(freqSymbol));
%             payloadSymbol = [timeSymbol(obj.Nc - obj.cpLength + 1:obj.Nc), timeSymbol];
%         end
        
%         function frame = generateFrame(obj, varargin)
%             % Generate frame with variable number of payload symbols
%             shortPreamble = obj.generateShortPreamble();
%             longPreamble = obj.generateLongPreamble();
            
%             frame = [shortPreamble, longPreamble];
            
%             % Add payload symbols
%             for i = 1:obj.nPayloadSymbols
%                 if i <= length(varargin)
%                     payloadData = varargin{i};
%                 else
%                     % Generate randomly if not provided
%                     payloadData = randi([0 obj.modulationOrder-1], 1, obj.Ndata);
%                 end
%                 payload = obj.generatePayloadSymbol(payloadData);
%                 frame = [frame, payload];
%             end
%         end
        
%         function txSignal = applyPulseShaping(obj, frame)
%             frameOversampled = obj.oversamp(frame, obj.ovsamplingFactor);
%             rolloff = 0.5;
%             L = 6;
%             RRC = rcosdesign(rolloff, L, obj.ovsamplingFactor, 'sqrt');
%             txSignal = conv(frameOversampled, RRC);
%         end
        
%         function txSignal = getTransmitFrame(obj, varargin)
%             frame = obj.generateFrame(varargin{:});
%             txSignal = obj.applyPulseShaping(frame);
%         end

%         function oversampledSignal = oversamp(~, signal, factor)
%             oversampledSignal = zeros(1, length(signal) * factor);
%             oversampledSignal(1:factor:end) = signal;
%         end
        
%         % ===== RECEIVER METHODS =====
%         function [decodedBits, rxMetrics] = receiveFrame(obj, rxSignal, varargin)
%             % Main receiver function
%             % Optional arguments: 'debug', true/false for plotting
            
%             p = inputParser;
%             addParameter(p, 'debug', false, @islogical);
%             addParameter(p, 'threshold', 0.75, @isnumeric);
%             parse(p, varargin{:});
            
%             debugMode = p.Results.debug;
%             threshold = p.Results.threshold;
            

%             rxFiltered = obj.applyMatchedFilter(rxSignal);
%             [packetStart, M_n, thresholdGraph] = obj.detectPacket(rxFiltered, threshold);
%             frameDownsampled = obj.downsampleFrame(rxFiltered, packetStart);
%             [frameAfterCFO, coarseCFO, fineCFO] = obj.correctCFO(frameDownsampled);
%             [H_est, H_est_time] = obj.estimateChannel(frameAfterCFO);
%             [decodedBits, rxPayloads] = obj.demodulatePayloads(frameAfterCFO, H_est);
            
%             rxMetrics = struct();
%             rxMetrics.packetStart = packetStart;
%             rxMetrics.M_n = M_n;
%             rxMetrics.thresholdGraph = thresholdGraph;
%             rxMetrics.coarseCFO = coarseCFO;
%             rxMetrics.fineCFO = fineCFO;
%             rxMetrics.H_est = H_est;
%             rxMetrics.H_est_time = H_est_time;
%             rxMetrics.rxPayloads = rxPayloads;
            
%             % Debug plotting
%             if debugMode
%                 obj.plotRxDebug(rxSignal, rxMetrics, decodedBits);
%             end
%         end
        
%         function rxFiltered = applyMatchedFilter(obj, rxSignal)
%             rolloff = 0.5;
%             L = 6;
%             RRC = rcosdesign(rolloff, L, obj.ovsamplingFactor, 'sqrt');
%             rxFiltered = conv(rxSignal, RRC);
%         end
        
%         function [packetStart, M_n, thresholdGraph] = detectPacket(~, rxSignal, threshold)
%             % Packet detection using autocorrelation of short preamble
%             D = 16; 
%             L = 32; 
            
%             C_n = zeros(1, length(rxSignal) - D + 1 - L);
%             P_n = zeros(1, length(rxSignal) - D + 1 - L);
%             C_k = zeros(1, L);
%             P_k = zeros(1, L);

%             for n = 1:length(C_n)
%                 for k = 1:L
%                     C_k(k) = rxSignal(n + k - 1) * conj(rxSignal(n + k - 1 + D));
%                     P_k(k) = abs(rxSignal(n + k - 1 + D))^2;
%                 end
%                 C_n(n) = sum(C_k);
%                 P_n(n) = sum(P_k);
%             end
            
%             % Calculate detection metric
%             M_n = (abs(C_n).^2) ./ (P_n.^2);
            
%             % Find packet start
%             loc = find(M_n > threshold);
%             if isempty(loc)
%                 error('No packet detected above threshold');
%             end
            
%             % Find packet front (gaps > 300 samples indicate new packet)
%             temp_1 = [loc, 0];
%             temp_2 = [0, loc];
%             temp_3 = temp_1 - temp_2;
%             packetFrontIdx = loc(temp_3 > 300);
            
%             lengthOverThreshold = 230;
%             packetStart = [];
            
%             for x = 1:length(packetFrontIdx)-1
%                 if M_n(packetFrontIdx(x) + lengthOverThreshold) > threshold
%                     packetStart = packetFrontIdx(x) + 6 + 1; % L_RRC = 6
%                     break;
%                 end
%             end
            
%             if isempty(packetStart)
%                 packetStart = packetFrontIdx(1) + 6 + 1;
%             end

%             % Create threshold graph for debugging
%             thresholdGraph = threshold * ones(1, length(M_n));
%             thresholdGraph(packetStart - 6 - 1) = 1.15;
%         end
        
%         function frameDownsampled = downsampleFrame(obj, rxSignal, packetStart)
%             endIdx = packetStart + obj.ovsamplingFactor * obj.frameLength - 1;
%             if endIdx > length(rxSignal)
%                 error('Frame extends beyond received signal length');
%             end
%             frameDownsampled = rxSignal(packetStart:obj.ovsamplingFactor:endIdx);
%         end
        
%         function [frameAfterCFO, coarseCFO, fineCFO] = correctCFO(obj, frameDownsampled)
%             shortPreambleSlotLength = 16;
%             % Coarse CFO estimation using short preamble
%             z1 = frameDownsampled(shortPreambleSlotLength*5+1:shortPreambleSlotLength*6) * conj(frameDownsampled(shortPreambleSlotLength*6+1:shortPreambleSlotLength*7)).';
%             coarseCFO = (-1/(2*pi*shortPreambleSlotLength*obj.Ts)) * angle(z1);
%             % Apply coarse correction
%             timeVector = obj.Ts * (0:obj.frameLength-1);
%             frameAfterCoarse = frameDownsampled .* exp(-1j*2*pi*coarseCFO*timeVector); 
%             % Fine CFO estimation using long preamble
%             z2 = frameAfterCoarse(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16) * frameAfterCoarse(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20)';
%             fineCFO = (-1/(2*pi*64*obj.Ts)) * angle(z2);
            
%             frameAfterCFO = frameAfterCoarse .* exp(-1j*2*pi*fineCFO*timeVector);
%         end
        
%         function [H_est, H_est_time] = estimateChannel(obj, frameAfterCFO)
%             % Channel estimation using long preamble symbols
%             shortPreambleSlotLength = 16;
            
%             % Extract long preamble symbols
%             longPreamble1 = frameAfterCFO(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16);
%             longPreamble2 = frameAfterCFO(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20);
            
%             % Convert to frequency domain
%             longPreamble1FFT = fftshift(fft(longPreamble1));
%             longPreamble2FFT = fftshift(fft(longPreamble2));
            
%             % Average the two symbols and compare with known sequence
%             longPreambleRef = obj.generateLongPreambleFreqDomain();
%             H_est = 0.5 * (longPreamble1FFT + longPreamble2FFT) .* conj(longPreambleRef);
            
%             H_est_time = ifft(ifftshift(H_est));
%         end
        
%         function [decodedBits, rxPayloads] = demodulatePayloads(obj, frameAfterCFO, H_est)
%             % Demodulate all payload symbols
%             payloadStartIdx = obj.shortPreambleLength + obj.longPreambleLength + 1;
            
%             decodedBits = cell(1, obj.nPayloadSymbols);
%             rxPayloads = struct();
%             rxPayloads.noEqualizer = cell(1, obj.nPayloadSymbols);
%             rxPayloads.withEqualizer = cell(1, obj.nPayloadSymbols);
            
%             for i = 1:obj.nPayloadSymbols
%                 % Extract payload symbol
%                 symbolStart = payloadStartIdx + (i-1) * obj.payloadSymbolLength;
%                 symbolEnd = symbolStart + obj.payloadSymbolLength - 1;
%                 payloadTime = frameAfterCFO(symbolStart:symbolEnd);
                
%                 % Remove cyclic prefix
%                 payloadNoCP = payloadTime(obj.cpLength+1:end);
                
%                 % Convert to frequency domain
%                 payloadFreq = fftshift(fft(payloadNoCP));
                
%                 % Apply equalizer
%                 payloadFreqEq = payloadFreq ./ H_est;
                
%                 % Extract data subcarriers (remove pilots and nulls)
%                 dataSymbolsNoEq = payloadFreq(obj.dataIndices);
%                 dataSymbolsEq = payloadFreqEq(obj.dataIndices);
                
%                 % Store for debugging
%                 rxPayloads.noEqualizer{i} = dataSymbolsNoEq;
%                 rxPayloads.withEqualizer{i} = dataSymbolsEq;
                
%                 % Demodulate
%                 if obj.modulationOrder == 2
%                     decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, 0);
%                 else
%                     decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, pi/4);
%                 end
%             end
%         end
        
%         function BER = calculateBER(~, decodedBits, referenceData)
%             % Calculate bit error rate
%             if length(decodedBits) ~= length(referenceData)
%                 error('Number of payload symbols mismatch');
%             end
            
%             totalErrors = 0;
%             totalBits = 0;
            
%             for i = 1:length(decodedBits)
%                 errors = sum(abs(sign(referenceData{i} - decodedBits{i})));
%                 totalErrors = totalErrors + errors;
%                 totalBits = totalBits + length(referenceData{i});
%             end
            
%             BER = totalErrors / totalBits;
%         end
        
%         function plotRxDebug(obj, rxSignal, rxMetrics)
%             % Debug plotting similar to original OFDM_RX
%             figure('Units', 'centimeters', 'Position', [1 2 49 24]);
            
%             % Raw received signal constellation
%             subplot(2,4,1);
%             plot(rxSignal, '.');
%             title('RX-Raw');
%             axis([-1.5 1.5 -1.5 1.5]);
%             axis square;
            
%             % I component
%             subplot(2,4,2);
%             plot(real(rxSignal));
%             title('I');
%             axis([1 3000 -1.5 1.5]);
%             axis square;
            
%             % Q component
%             subplot(2,4,3);
%             plot(imag(rxSignal));
%             title('Q');
%             axis([1 3000 -1.5 1.5]);
%             axis square;
            
%             % Power spectral density
%             subplot(2,4,4);
%             [spectrumWaveform, welchFreq] = pwelch(rxSignal, [], [], [], 1/obj.Ts, 'centered', 'power');
%             plot(welchFreq, pow2db(spectrumWaveform));
%             title('Welch Power Spectral Density');
%             axis square;
            
%             % Packet detection
%             subplot(2,4,5);
%             plot(1:length(rxMetrics.M_n), rxMetrics.M_n, 1:length(rxMetrics.M_n), rxMetrics.thresholdGraph);
%             title('Packet Detection');
%             axis([1, length(rxMetrics.M_n), 0, 1.2]);
%             axis square;
            
%             % Channel estimation
%             subplot(2,4,6);
%             plot(abs(rxMetrics.H_est_time));
%             title('Channel Estimation');
%             axis([1 64 0 7]);
%             axis square;
%             xlabel('Time');
            
%             % Before equalizer
%             subplot(2,4,7);
%             hold on;
%             for i = 1:obj.nPayloadSymbols
%                 plot(rxMetrics.rxPayloads.noEqualizer{i}, '*');
%             end
%             title('Before Equalizer');
%             axis([-8 8 -8 8]);
%             axis square;
%             hold off;
            
%             % After equalizer (demodulation)
%             subplot(2,4,8);
%             hold on;
%             for i = 1:obj.nPayloadSymbols
%                 plot(rxMetrics.rxPayloads.withEqualizer{i}, '*');
%             end
%             title('Demodulation');
%             axis([-1.5 1.5 -1.5 1.5]);
%             axis square;
%             hold off;
%         end
%     end
% end

classdef OFDMHelper
    properties
        % Minimum Independent Parameters
        Nc;
        Ndata;
        Npilot;
        cpLength;
        modulationOrder;
        nPayloadSymbols;
        
        % Derived Parameters
        Nvirtual;
        pilotIndices;
        dataIndices;
        nullIndices;
        
        % Fixed sequences 
        shortPreambleSeq;
        longPreambleSeq;
        pilotSeq;
        
        ovsamplingFactor;
        bbSampleRate;
        
        % Timing parameters (derived)
        shortPreambleLength;
        longPreambleLength;
        payloadSymbolLength;
        frameLength;
        Ts; % Sample time
    end
    
    methods
        function obj = OFDMHelper(Nc, Ndata, Npilot, cpLength, modulationOrder, nPayloadSymbols, bbSampleRate, ovsamplingFactor)
            if nargin < 8
                obj.Nc = 64;
                obj.Ndata = 48;
                obj.Npilot = 4;
                obj.cpLength = 16;
                obj.modulationOrder = 4;
                obj.nPayloadSymbols = 2;
                obj.ovsamplingFactor = 2;
                obj.bbSampleRate = 1e6;
            else
                obj.Nc = Nc;
                obj.Ndata = Ndata;
                obj.Npilot = Npilot;
                obj.cpLength = cpLength;
                obj.modulationOrder = modulationOrder;
                obj.nPayloadSymbols = nPayloadSymbols;
                obj.ovsamplingFactor = ovsamplingFactor;
                obj.bbSampleRate = bbSampleRate;
            end
            
            obj = obj.deriveParameters();
            obj = obj.calculateTimingParameters();
            
            % Fixed sequences [from the 802.11a standard]
            obj.shortPreambleSeq = sqrt(13/6)*[0,0,1+1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,-1-1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,0,0,0,0,-1-1j,0,0,0,-1-1j,0,0,0,1+1j,0,0,0,1+1j,0,0,0,1+1j,0,0,0,1+1j,0,0];
            obj.longPreambleSeq = [1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1];
            obj.pilotSeq = repmat([1,1,1,-1], 1, ceil(obj.Npilot/4)); 
            obj.pilotSeq = obj.pilotSeq(1:obj.Npilot);
        end
        
        function obj = deriveParameters(obj)
            % Calculate number of virtual subcarriers
            obj.Nvirtual = obj.Nc - obj.Ndata - obj.Npilot - 1; % -1 for DC
            
            % Standard subcarrier allocation
            virtualLeft = floor(obj.Nvirtual/2);
            virtualRight = obj.Nvirtual - virtualLeft;
            
            % DC is at center (index Nc/2 + 1 in 1-indexed)
            dcIndex = obj.Nc/2 + 1;
            
            obj.nullIndices = [1:virtualLeft, dcIndex, (obj.Nc-virtualRight+1):obj.Nc];
            
            activeIndices = setdiff(1:obj.Nc, obj.nullIndices);
            
            % Distribute pilots evenly among active subcarriers
            pilotSpacing = floor(length(activeIndices) / obj.Npilot);
            obj.pilotIndices = activeIndices(1:pilotSpacing:obj.Npilot*pilotSpacing);
            obj.pilotIndices = obj.pilotIndices(1:obj.Npilot); 
            
            % Remaining are data subcarriers
            obj.dataIndices = setdiff(activeIndices, obj.pilotIndices);
            obj.dataIndices = obj.dataIndices(1:obj.Ndata);
        end
        
        function obj = calculateTimingParameters(obj)
            % Calculate frame timing parameters
            obj.Ts = 1 / (obj.bbSampleRate * obj.ovsamplingFactor);
            
            % Short preamble: 10 repetitions of 16 samples each
            obj.shortPreambleLength = 160; % Fixed for 802.11a-style
            
            % Long preamble: 32 CP + 64 + 64 samples
            obj.longPreambleLength = 160; % Fixed for 802.11a-style
            
            % Payload symbol: CP + FFT size
            obj.payloadSymbolLength = obj.cpLength + obj.Nc;
            
            % Total frame length
            obj.frameLength = obj.shortPreambleLength + obj.longPreambleLength + ...
                             obj.nPayloadSymbols * obj.payloadSymbolLength;
        end
        
        function saveConfig(obj, filename)
            % Save all parameters needed for RX to a .mat file
            if nargin < 2
                filename = 'ofdm_config.mat';
            end
            
            % Create configuration structure
            config = struct();
            
            % Basic parameters
            config.Nc = obj.Nc;
            config.Ndata = obj.Ndata;
            config.Npilot = obj.Npilot;
            config.cpLength = obj.cpLength;
            config.modulationOrder = obj.modulationOrder;
            config.nPayloadSymbols = obj.nPayloadSymbols;
            config.ovsamplingFactor = obj.ovsamplingFactor;
            config.bbSampleRate = obj.bbSampleRate;
            
            % Derived parameters
            config.Nvirtual = obj.Nvirtual;
            config.pilotIndices = obj.pilotIndices;
            config.dataIndices = obj.dataIndices;
            config.nullIndices = obj.nullIndices;
            
            % Sequences
            config.shortPreambleSeq = obj.shortPreambleSeq;
            config.longPreambleSeq = obj.longPreambleSeq;
            config.pilotSeq = obj.pilotSeq;
            
            % Timing parameters
            config.Ts = obj.Ts;
            config.shortPreambleLength = obj.shortPreambleLength;
            config.longPreambleLength = obj.longPreambleLength;
            config.payloadSymbolLength = obj.payloadSymbolLength;
            config.frameLength = obj.frameLength;
            
            % Generate reference sequences for RX
            config.longPreambleSlotFrequency = obj.generateLongPreambleFreqDomain();
            
            % Save to file
            save(filename, 'config');
            fprintf('Configuration saved to %s\n', filename);
        end
        
        function obj = loadConfig(obj, filename)
            % Load configuration from .mat file
            if nargin < 2
                filename = 'ofdm_config.mat';
            end
            
            if ~exist(filename, 'file')
                error('Configuration file %s not found', filename);
            end
            
            data = load(filename);
            config = data.config;
            
            % Load all parameters
            obj.Nc = config.Nc;
            obj.Ndata = config.Ndata;
            obj.Npilot = config.Npilot;
            obj.cpLength = config.cpLength;
            obj.modulationOrder = config.modulationOrder;
            obj.nPayloadSymbols = config.nPayloadSymbols;
            obj.ovsamplingFactor = config.ovsamplingFactor;
            obj.bbSampleRate = config.bbSampleRate;
            
            obj.Nvirtual = config.Nvirtual;
            obj.pilotIndices = config.pilotIndices;
            obj.dataIndices = config.dataIndices;
            obj.nullIndices = config.nullIndices;
            
            obj.shortPreambleSeq = config.shortPreambleSeq;
            obj.longPreambleSeq = config.longPreambleSeq;
            obj.pilotSeq = config.pilotSeq;
            
            obj.Ts = config.Ts;
            obj.shortPreambleLength = config.shortPreambleLength;
            obj.longPreambleLength = config.longPreambleLength;
            obj.payloadSymbolLength = config.payloadSymbolLength;
            obj.frameLength = config.frameLength;
            
            fprintf('Configuration loaded from %s\n', filename);
        end
        
        function longPreambleFreq = generateLongPreambleFreqDomain(obj)
            % Generate the frequency domain long preamble for RX reference
            virtualSubcarrier = zeros(1, obj.Nc - length(obj.longPreambleSeq));
            longPreambleFreq = [virtualSubcarrier(1:6), obj.longPreambleSeq, virtualSubcarrier(7:11)];
        end
        
        function shortPreamble = generateShortPreamble(obj)
            virtualSubcarrier = zeros(1, obj.Nc - length(obj.shortPreambleSeq));
            shortPreambleSlotFreq = [virtualSubcarrier(1:6), obj.shortPreambleSeq, virtualSubcarrier(7:11)];
            shortPreambleSlotTime = ifft(ifftshift(shortPreambleSlotFreq)) * obj.Nc; % Normalize
            shortPreamble = repmat(shortPreambleSlotTime(1:16), 1, 10);
        end
        
        function longPreamble = generateLongPreamble(obj)
            virtualSubcarrier = zeros(1, obj.Nc - length(obj.longPreambleSeq));
            longPreambleSlotFreq = [virtualSubcarrier(1:6), obj.longPreambleSeq, virtualSubcarrier(7:11)];
            longPreambleSlotTime = ifft(ifftshift(longPreambleSlotFreq)) * obj.Nc; % Normalize
            longPreamble = [longPreambleSlotTime(obj.Nc/2+1:obj.Nc), longPreambleSlotTime, longPreambleSlotTime];
        end
        
        function payloadSymbol = generatePayloadSymbol(obj, dataBits)
            if obj.modulationOrder == 2
                dataModulated = pskmod(dataBits, obj.modulationOrder, 0, "gray");
            else
                dataModulated = pskmod(dataBits, obj.modulationOrder, pi/4, "gray");
            end
            
            freqSymbol = zeros(1, obj.Nc);
            freqSymbol(obj.dataIndices(1:length(dataModulated))) = dataModulated;
            freqSymbol(obj.pilotIndices) = obj.pilotSeq;
            freqSymbol(obj.nullIndices) = 0;
            
            % Convert to time domain and add CP
            timeSymbol = ifft(ifftshift(freqSymbol)) * obj.Nc; % Normalize
            payloadSymbol = [timeSymbol(obj.Nc - obj.cpLength + 1:obj.Nc), timeSymbol];
        end
        
        function frame = generateFrame(obj, varargin)
            % Generate frame with variable number of payload symbols
            shortPreamble = obj.generateShortPreamble();
            longPreamble = obj.generateLongPreamble();
            
            frame = [shortPreamble, longPreamble];
            
            % Add payload symbols
            for i = 1:obj.nPayloadSymbols
                if i <= length(varargin)
                    payloadData = varargin{i};
                else
                    % Generate randomly if not provided
                    payloadData = randi([0 obj.modulationOrder-1], 1, obj.Ndata);
                end
                payload = obj.generatePayloadSymbol(payloadData);
                frame = [frame, payload];
            end
        end
        
        function txSignal = applyPulseShaping(obj, frame)
            frameOversampled = obj.oversamp(frame, obj.ovsamplingFactor);
            rolloff = 0.5;
            L = 6;
            RRC = rcosdesign(rolloff, L, obj.ovsamplingFactor, 'sqrt');
            txSignal = conv(frameOversampled, RRC, 'same'); % Use 'same' to maintain length
        end
        
        function txSignal = getTransmitFrame(obj, varargin)
            frame = obj.generateFrame(varargin{:});
            txSignal = obj.applyPulseShaping(frame);
        end

        function oversampledSignal = oversamp(~, signal, factor)
            oversampledSignal = zeros(1, length(signal) * factor);
            oversampledSignal(1:factor:end) = signal;
        end
        
        % ===== RECEIVER METHODS =====
        function [decodedBits, rxMetrics] = receiveFrame(obj, rxSignal, varargin)
            % Main receiver function
            % Optional arguments: 'debug', true/false for plotting
            
            p = inputParser;
            addParameter(p, 'debug', false, @islogical);
            addParameter(p, 'threshold', 0.75, @isnumeric);
            addParameter(p, 'referenceData', {}, @iscell); % For BER calculation
            parse(p, varargin{:});
            
            debugMode = p.Results.debug;
            threshold = p.Results.threshold;
            referenceData = p.Results.referenceData;
            
            % RX chain
            rxFiltered = obj.applyMatchedFilter(rxSignal);
            [packetStart, M_n, thresholdGraph] = obj.detectPacket(rxFiltered, threshold);
            frameDownsampled = obj.downsampleFrame(rxFiltered, packetStart);
            [frameAfterCFO, coarseCFO, fineCFO] = obj.correctCFO(frameDownsampled);
            [H_est, H_est_time] = obj.estimateChannel(frameAfterCFO);
            [decodedBits, rxPayloads] = obj.demodulatePayloads(frameAfterCFO, H_est);
            
            % Calculate SNR and EVM
            [SNR_est, EVM] = obj.calculateQualityMetrics(rxPayloads, H_est);
            
            % Calculate BER if reference provided
            BER = NaN;
            if ~isempty(referenceData)
                BER = obj.calculateBER(decodedBits, referenceData);
            end
            
            % Build metrics structure
            rxMetrics = struct();
            rxMetrics.packetStart = packetStart;
            rxMetrics.M_n = M_n;
            rxMetrics.thresholdGraph = thresholdGraph;
            rxMetrics.coarseCFO = coarseCFO;
            rxMetrics.fineCFO = fineCFO;
            rxMetrics.totalCFO = coarseCFO + fineCFO;
            rxMetrics.H_est = H_est;
            rxMetrics.H_est_time = H_est_time;
            rxMetrics.rxPayloads = rxPayloads;
            rxMetrics.SNR_est = SNR_est;
            rxMetrics.EVM = EVM;
            rxMetrics.BER = BER;
            rxMetrics.frameDownsampled = frameDownsampled;
            rxMetrics.rxFiltered = rxFiltered;
            
            % Debug plotting
            if debugMode
                obj.plotRxComprehensive(rxSignal, rxMetrics, decodedBits);
            end
        end
        
        function rxFiltered = applyMatchedFilter(obj, rxSignal)
            rolloff = 0.5;
            L = 6;
            RRC = rcosdesign(rolloff, L, obj.ovsamplingFactor, 'sqrt');
            rxFiltered = conv(rxSignal, RRC, 'same'); % Use 'same'
        end
        
        function [packetStart, M_n, thresholdGraph] = detectPacket(~, rxSignal, threshold)
            % Packet detection using autocorrelation of short preamble
            D = 16; 
            L = 32; 
            
            C_n = zeros(1, length(rxSignal) - D + 1 - L);
            P_n = zeros(1, length(rxSignal) - D + 1 - L);
            C_k = zeros(1, L);
            P_k = zeros(1, L);

            for n = 1:length(C_n)
                for k = 1:L
                    C_k(k) = rxSignal(n + k - 1) * (rxSignal(n + k - 1 + D));
                    P_k(k) = abs(rxSignal(n + k - 1 + D))^2;
                end
                C_n(n) = sum(C_k);
                P_n(n) = sum(P_k);
            end
            
            % Calculate detection metric1
            M_n = (abs(C_n).^2) ./ (P_n.^2 + eps); % Add eps to avoid division by zero
            
            % Find packet start
            loc = find(M_n > threshold);
            if isempty(loc)
                error('No packet detected above threshold %.2f', threshold);
            end
            
            % Find packet front (gaps > 300 samples indicate new packet)
            temp_1 = [loc, 0];
            temp_2 = [0, loc];
            temp_3 = temp_1 - temp_2;
            packetFrontIdx = loc(temp_3 > 300);
            
            lengthOverThreshold = 230;
            packetStart = [];
            
            for x = 1:length(packetFrontIdx)-1
                if M_n(packetFrontIdx(x) + lengthOverThreshold) > threshold
                    packetStart = packetFrontIdx(x) + 6 + 1; % L_RRC = 6
                    break;
                end
            end
            
            if isempty(packetStart)
                packetStart = packetFrontIdx(1) + 6 + 1;
            end

            % Create threshold graph for debugging
            thresholdGraph = threshold * ones(1, length(M_n));
            thresholdGraph(packetStart - 6 - 1) = 1.15;
        end
        
        function frameDownsampled = downsampleFrame(obj, rxSignal, packetStart)
            endIdx = packetStart + obj.ovsamplingFactor * obj.frameLength - 1;
            if endIdx > length(rxSignal)
                warning('Frame extends beyond received signal, truncating');
                endIdx = length(rxSignal);
            end
            frameDownsampled = rxSignal(packetStart:obj.ovsamplingFactor:endIdx);
        end
        
        function [frameAfterCFO, coarseCFO, fineCFO] = correctCFO(obj, frameDownsampled)
            shortPreambleSlotLength = 16;
            
            % Coarse CFO estimation using short preamble
            z1 = frameDownsampled(shortPreambleSlotLength*5+1:shortPreambleSlotLength*6).' * ...
                 conj(frameDownsampled(shortPreambleSlotLength*6+1:shortPreambleSlotLength*7));
            
            coarseCFO = (-1/(2*pi*shortPreambleSlotLength*obj.Ts)) * angle(z1);
            % disp(size(z1));
            % disp(size(coarseCFO));
            % Apply coarse correction
            timeVector = obj.Ts * (0:length(frameDownsampled)-1);
            % disp(size(timeVector));
            frameAfterCoarse = frameDownsampled .* exp(-1j*2*pi*coarseCFO*timeVector); 
            
            % disp(size(frameDownsampled));
            % Fine CFO estimation using long preamble
            z2 = frameAfterCoarse(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16) * ...
                 frameAfterCoarse(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20)';
            fineCFO = (-1/(2*pi*64*obj.Ts)) * angle(z2);
            
            frameAfterCFO = frameAfterCoarse .* exp(-1j*2*pi*fineCFO*timeVector);
        end
        
        function [H_est, H_est_time] = estimateChannel(obj, frameAfterCFO)
            % Channel estimation using long preamble symbols
            shortPreambleSlotLength = 16;
            
            % Extract long preamble symbols
            longPreamble1 = frameAfterCFO(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16);
            longPreamble2 = frameAfterCFO(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20);
            
            % Convert to frequency domain (normalized)
            longPreamble1FFT = fftshift(fft(longPreamble1)) / obj.Nc;
            longPreamble2FFT = fftshift(fft(longPreamble2)) / obj.Nc;
            
            % Average the two symbols and compare with known sequence
            longPreambleRef = obj.generateLongPreambleFreqDomain();
            H_est = 0.5 * (longPreamble1FFT + longPreamble2FFT) ./ (longPreambleRef + eps);
            
            H_est_time = ifft(ifftshift(H_est)) * obj.Nc;
        end
        
        function [decodedBits, rxPayloads] = demodulatePayloads(obj, frameAfterCFO, H_est)
            % Demodulate all payload symbols
            payloadStartIdx = obj.shortPreambleLength + obj.longPreambleLength + 1;
            
            decodedBits = cell(1, obj.nPayloadSymbols);
            rxPayloads = struct();
            rxPayloads.noEqualizer = cell(1, obj.nPayloadSymbols);
            rxPayloads.withEqualizer = cell(1, obj.nPayloadSymbols);
            rxPayloads.pilots = cell(1, obj.nPayloadSymbols);
            
            for i = 1:obj.nPayloadSymbols
                % Extract payload symbol
                symbolStart = payloadStartIdx + (i-1) * obj.payloadSymbolLength;
                symbolEnd = symbolStart + obj.payloadSymbolLength - 1;
                
                if symbolEnd > length(frameAfterCFO)
                    warning('Payload symbol %d extends beyond frame', i);
                    break;
                end
                
                payloadTime = frameAfterCFO(symbolStart:symbolEnd);
                
                % Remove cyclic prefix
                payloadNoCP = payloadTime(obj.cpLength+1:end);
                
                % Convert to frequency domain (normalized)
                payloadFreq = fftshift(fft(payloadNoCP)) / obj.Nc;
                
                % Apply equalizer
                payloadFreqEq = payloadFreq ./ (H_est + eps);
                
                % Extract data subcarriers and pilots
                dataSymbolsNoEq = payloadFreq(obj.dataIndices);
                dataSymbolsEq = payloadFreqEq(obj.dataIndices);
                pilotSymbols = payloadFreqEq(obj.pilotIndices);
                
                % Store for debugging
                rxPayloads.noEqualizer{i} = dataSymbolsNoEq;
                rxPayloads.withEqualizer{i} = dataSymbolsEq;
                rxPayloads.pilots{i} = pilotSymbols;
                
                % Demodulate
                if obj.modulationOrder == 2
                    decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, 0, 'gray');
                else
                    decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, pi/4, 'gray');
                end
            end
        end
        
        function [SNR_est, EVM] = calculateQualityMetrics(obj, rxPayloads, H_est)
            % Calculate SNR and EVM from equalized constellations
            
            % Get ideal constellation points
            if obj.modulationOrder == 2
                idealConstellation = pskmod(0:obj.modulationOrder-1, obj.modulationOrder, 0, 'gray');
            else
                idealConstellation = pskmod(0:obj.modulationOrder-1, obj.modulationOrder, pi/4, 'gray');
            end
            
            % Accumulate error
            totalSignalPower = 0;
            totalNoisePower = 0;
            totalEVMSquared = 0;
            totalSymbols = 0;
            
            for i = 1:length(rxPayloads.withEqualizer)
                rxSymbols = rxPayloads.withEqualizer{i};
                
                % Find nearest constellation point for each symbol
                for j = 1:length(rxSymbols)
                    [~, idx] = min(abs(rxSymbols(j) - idealConstellation));
                    idealSymbol = idealConstellation(idx);
                    
                    error = rxSymbols(j) - idealSymbol;
                    totalSignalPower = totalSignalPower + abs(idealSymbol)^2;
                    totalNoisePower = totalNoisePower + abs(error)^2;
                    totalEVMSquared = totalEVMSquared + abs(error)^2;
                end
                totalSymbols = totalSymbols + length(rxSymbols);
            end
            
            % Calculate SNR in dB
            SNR_est = 10*log10(totalSignalPower / (totalNoisePower + eps));
            
            % Calculate EVM in percent
            avgSignalPower = totalSignalPower / totalSymbols;
            EVM = 100 * sqrt(totalEVMSquared / totalSymbols) / sqrt(avgSignalPower);
        end
        
        function BER = calculateBER(~, decodedBits, referenceData)
            % Calculate bit error rate
            if length(decodedBits) ~= length(referenceData)
                warning('Number of payload symbols mismatch');
                BER = NaN;
                return;
            end
            
            totalErrors = 0;
            totalBits = 0;
            
            for i = 1:length(decodedBits)
                if length(decodedBits{i}) ~= length(referenceData{i})
                    warning('Payload %d length mismatch', i);
                    continue;
                end
                errors = sum(decodedBits{i} ~= referenceData{i});
                totalErrors = totalErrors + errors;
                totalBits = totalBits + length(referenceData{i});
            end
            
            BER = totalErrors / (totalBits + eps);
        end
        
        function plotRxComprehensive(obj, rxSignal, rxMetrics, decodedBits)
            % Comprehensive receiver analysis plots
            
            % Create large figure
            fig = figure('Name', 'OFDM Receiver Analysis', 'NumberTitle', 'off');
            fig.Position = [50, 50, 1600, 1000];
            
            % Color scheme
            colorData = [0.2, 0.6, 0.8];
            colorPilot = [0.8, 0.3, 0.3];
            colorIdeal = [0.3, 0.7, 0.3];
            
            %% Row 1: Input Signal Analysis
            subplot(3,5,1);
            plot(real(rxSignal), 'LineWidth', 0.5);
            hold on;
            plot(imag(rxSignal), 'LineWidth', 0.5);
            title('RX Signal - Time Domain');
            xlabel('Sample');
            ylabel('Amplitude');
            legend('I', 'Q');
            grid on;
            xlim([1 min(3000, length(rxSignal))]);
            
            subplot(3,5,2);
            plot(rxSignal, '.', 'MarkerSize', 2);
            title('RX Signal - Constellation');
            xlabel('In-Phase');
            ylabel('Quadrature');
            axis equal;
            grid on;
            
            subplot(3,5,3);
            [psd, f] = pwelch(rxSignal, hamming(256), 128, 512, 1/obj.Ts, 'centered');
            plot(f/1e6, 10*log10(psd), 'LineWidth', 1.5);
            title('Power Spectral Density');
            xlabel('Frequency (MHz)');
            ylabel('PSD (dB/Hz)');
            grid on;
            
            subplot(3,5,4);
            spectrogram(rxSignal, 128, 120, 128, 1/obj.Ts, 'yaxis');
            title('Spectrogram');
            colormap(jet);
            
            subplot(3,5,5);
            histogram(abs(rxSignal), 50, 'Normalization', 'pdf');
            title('Amplitude Distribution');
            xlabel('Magnitude');
            ylabel('PDF');
            grid on;
            
            %% Row 2: Synchronization and Channel
            subplot(3,5,6);
            plot(rxMetrics.M_n, 'LineWidth', 1.5, 'Color', colorData);
            hold on;
            plot(rxMetrics.thresholdGraph, '--r', 'LineWidth', 2);
            xline(rxMetrics.packetStart, 'g--', 'LineWidth', 2);
            title(sprintf('Packet Detection (Start: %d)', rxMetrics.packetStart));
            xlabel('Sample');
            ylabel('Metric M_n');
            legend('M_n', 'Threshold', 'Detected Start');
            grid on;
            xlim([1 length(rxMetrics.M_n)]);
            
            subplot(3,5,7);
            plot(abs(rxMetrics.H_est_time), 'LineWidth', 2, 'Color', colorData);
            title('Channel Impulse Response');
            xlabel('Tap');
            ylabel('Magnitude');
            grid on;
            xlim([1 obj.Nc]);
            
            subplot(3,5,8);
            plot(abs(rxMetrics.H_est), 'LineWidth', 2, 'Color', colorData);
            hold on;
            plot(angle(rxMetrics.H_est), 'LineWidth', 2, 'Color', colorPilot);
            title('Channel Frequency Response');
            xlabel('Subcarrier');
            ylabel('Value');
            legend('Magnitude', 'Phase');
            grid on;
            xlim([1 obj.Nc]);
            
            subplot(3,5,9);
            bar([rxMetrics.coarseCFO, rxMetrics.fineCFO, rxMetrics.totalCFO]);
            title('CFO Estimation');
            ylabel('Frequency Offset (Hz)');
            set(gca, 'XTickLabel', {'Coarse', 'Fine', 'Total'});
            grid on;
            
            subplot(3,5,10);
            % SNR and EVM text display
            axis off;
            textStr = {
                sprintf('\\fontsize{12}\\bf Performance Metrics');
                '';
                sprintf('SNR: %.2f dB', rxMetrics.SNR_est);
                sprintf('EVM: %.2f %%', rxMetrics.EVM);
                sprintf('BER: %.2e', rxMetrics.BER);
                '';
                sprintf('Coarse CFO: %.2f Hz', rxMetrics.coarseCFO);
                sprintf('Fine CFO: %.2f Hz', rxMetrics.fineCFO);
                sprintf('Total CFO: %.2f Hz', rxMetrics.totalCFO);
            };
            text(0.1, 0.9, textStr, 'VerticalAlignment', 'top', 'FontSize', 10);
            
            %% Row 3: Constellation Analysis
            subplot(3,5,11);
            % Before equalization - all symbols
            hold on;
            for i = 1:obj.nPayloadSymbols
                plot(rxMetrics.rxPayloads.noEqualizer{i}, 'o', ...
                    'MarkerSize', 6, 'LineWidth', 1.5);
            end
            title('Before Equalization');
            xlabel('In-Phase');
            ylabel('Quadrature');
            axis equal;
            grid on;
            legend(arrayfun(@(x) sprintf('Symbol %d', x), 1:obj.nPayloadSymbols, 'UniformOutput', false));
            
            subplot(3,5,12);
            % After equalization with ideal constellation
            hold on;
            colors = lines(obj.nPayloadSymbols);
            for i = 1:obj.nPayloadSymbols
                scatter(real(rxMetrics.rxPayloads.withEqualizer{i}), ...
                       imag(rxMetrics.rxPayloads.withEqualizer{i}), ...
                       50, colors(i,:), 'o', 'LineWidth', 1.5);
            end
            
            % Plot ideal constellation
            if obj.modulationOrder == 2
                idealConst = pskmod(0:obj.modulationOrder-1, obj.modulationOrder, 0, 'gray');
            else
                idealConst = pskmod(0:obj.modulationOrder-1, obj.modulationOrder, pi/4, 'gray');
            end
            plot(real(idealConst), imag(idealConst), 'kx', 'MarkerSize', 15, 'LineWidth', 3);
            
            title(sprintf('After Equalization (%d-PSK)', obj.modulationOrder));
            xlabel('In-Phase');
            ylabel('Quadrature');
            axis equal;
            grid on;
            legend([arrayfun(@(x) sprintf('Sym %d', x), 1:obj.nPayloadSymbols, 'UniformOutput', false), {'Ideal'}]);
            
            subplot(3,5,13);
            % EVM per symbol
            evmPerSymbol = zeros(1, obj.nPayloadSymbols);
            for i = 1:obj.nPayloadSymbols
                rxSymbols = rxMetrics.rxPayloads.withEqualizer{i};
                errorVec = zeros(size(rxSymbols));
                for j = 1:length(rxSymbols)
                    [~, idx] = min(abs(rxSymbols(j) - idealConst));
                    errorVec(j) = abs(rxSymbols(j) - idealConst(idx));
                end
                evmPerSymbol(i) = 100 * sqrt(mean(abs(errorVec).^2)) / sqrt(mean(abs(idealConst).^2));
            end
            bar(evmPerSymbol, 'FaceColor', colorData);
            title('EVM per OFDM Symbol');
            xlabel('OFDM Symbol Index');
            ylabel('EVM (%)');
            grid on;
            
            subplot(3,5,14);
            % Phase error across symbols
            hold on;
            for i = 1:obj.nPayloadSymbols
                rxSymbols = rxMetrics.rxPayloads.withEqualizer{i};
                phaseErrors = zeros(size(rxSymbols));
                for j = 1:length(rxSymbols)
                    [~, idx] = min(abs(rxSymbols(j) - idealConst));
                    phaseErrors(j) = angle(rxSymbols(j) * conj(idealConst(idx))) * 180/pi;
                end
                plot(phaseErrors, 'o-', 'LineWidth', 1.5);
            end
            title('Phase Error per Subcarrier');
            xlabel('Data Subcarrier Index');
            ylabel('Phase Error (degrees)');
            grid on;
            legend(arrayfun(@(x) sprintf('Sym %d', x), 1:obj.nPayloadSymbols, 'UniformOutput', false));
            
            subplot(3,5,15);
            % Pilot tracking
            hold on;
            for i = 1:obj.nPayloadSymbols
                if isfield(rxMetrics.rxPayloads, 'pilots') && ~isempty(rxMetrics.rxPayloads.pilots{i})
                    plot(real(rxMetrics.rxPayloads.pilots{i}), ...
                         imag(rxMetrics.rxPayloads.pilots{i}), ...
                         'o', 'MarkerSize', 10, 'LineWidth', 2);
                end
            end
            % Plot expected pilot values
            expectedPilots = obj.pilotSeq;
            plot(real(expectedPilots), imag(expectedPilots), 'kx', 'MarkerSize', 15, 'LineWidth', 3);
            title('Pilot Subcarriers');
            xlabel('In-Phase');
            ylabel('Quadrature');
            axis equal;
            grid on;
            legend([arrayfun(@(x) sprintf('Sym %d', x), 1:obj.nPayloadSymbols, 'UniformOutput', false), {'Expected'}]);
            
            % Add overall title
            sgtitle(sprintf('OFDM Receiver Analysis | SNR: %.2f dB | EVM: %.2f%% | BER: %.2e', ...
                rxMetrics.SNR_est, rxMetrics.EVM, rxMetrics.BER), 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotTxSignal(obj, txSignal, txData)
            % Plot transmit signal characteristics
            
            fig = figure('Name', 'OFDM Transmitter Analysis', 'NumberTitle', 'off');
            fig.Position = [100, 100, 1400, 800];
            
            % Time domain
            subplot(2,4,1);
            t = (0:length(txSignal)-1) * obj.Ts * 1e6; % in microseconds
            plot(t, real(txSignal), 'LineWidth', 1);
            hold on;
            plot(t, imag(txSignal), 'LineWidth', 1);
            title('TX Signal - Time Domain');
            xlabel('Time (\mus)');
            ylabel('Amplitude');
            legend('I', 'Q');
            grid on;
            
            % Constellation
            subplot(2,4,2);
            plot(txSignal, '.', 'MarkerSize', 3);
            title('TX Signal - Constellation');
            xlabel('In-Phase');
            ylabel('Quadrature');
            axis equal;
            grid on;
            
            % Power spectral density
            subplot(2,4,3);
            [psd, f] = pwelch(txSignal, hamming(512), 256, 1024, 1/obj.Ts, 'centered');
            plot(f/1e6, 10*log10(psd), 'LineWidth', 1.5);
            title('Power Spectral Density');
            xlabel('Frequency (MHz)');
            ylabel('PSD (dB/Hz)');
            grid on;
            
            % PAPR analysis
            subplot(2,4,4);
            instantPower = abs(txSignal).^2;
            avgPower = mean(instantPower);
            peakPower = max(instantPower);
            PAPR = 10*log10(peakPower/avgPower);
            
            histogram(10*log10(instantPower/avgPower), 50);
            xline(10*log10(peakPower/avgPower), 'r--', 'LineWidth', 2);
            title(sprintf('Power Distribution (PAPR: %.2f dB)', PAPR));
            xlabel('Power (dB relative to average)');
            ylabel('Count');
            grid on;
            
            % Frame structure visualization - FIXED
            subplot(2,4,5);
            % Calculate frame length before oversampling and pulse shaping
            baseFrameLength = obj.shortPreambleLength + obj.longPreambleLength + ...
                            obj.nPayloadSymbols * obj.payloadSymbolLength;
            
            % Create structure indicators
            shortPreambleMarker = ones(1, obj.shortPreambleLength);
            longPreambleMarker = 2 * ones(1, obj.longPreambleLength);
            payloadMarker = 3 * ones(1, obj.nPayloadSymbols * obj.payloadSymbolLength);
            
            % Concatenate
            frameStructure = [shortPreambleMarker, longPreambleMarker, payloadMarker];
            
            imagesc(frameStructure);
            colormap(gca, [0.7 0.7 1; 1 0.7 0.7; 0.7 1 0.7]);
            title('Frame Structure');
            xlabel('Sample');
            ylabel('Section');
            yticks([]);
            cb = colorbar;
            cb.Ticks = [1.33, 2, 2.67];
            cb.TickLabels = {'Short Preamble', 'Long Preamble', 'Payload'};
            
            % Spectrogram
            subplot(2,4,6);
            spectrogram(txSignal, 128, 120, 128, 1/obj.Ts, 'yaxis');
            title('Spectrogram');
            colormap(gca, jet);
            
            % Amplitude envelope
            subplot(2,4,7);
            envelope = abs(txSignal);
            plot(t, envelope, 'LineWidth', 1.5);
            title('Amplitude Envelope');
            xlabel('Time (\mus)');
            ylabel('Amplitude');
            grid on;
            
            % Phase trajectory
            subplot(2,4,8);
            phase = unwrap(angle(txSignal)) * 180/pi;
            plot(t, phase, 'LineWidth', 1);
            title('Phase Trajectory');
            xlabel('Time (\mus)');
            ylabel('Phase (degrees)');
            grid on;
            
            sgtitle('OFDM Transmitter Signal Analysis', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function plotFrequencyDomain(obj, frame)
            % Detailed frequency domain analysis
            
            figure('Name', 'Frequency Domain Analysis', 'NumberTitle', 'off');
            
            % Extract different parts of frame
            shortPreamble = frame(1:obj.shortPreambleLength);
            longPreamble = frame(obj.shortPreambleLength+1:obj.shortPreambleLength+obj.longPreambleLength);
            
            % Short preamble spectrum
            subplot(2,3,1);
            shortFreq = fftshift(fft(shortPreamble));
            f = linspace(-1/(2*obj.Ts), 1/(2*obj.Ts), length(shortFreq));
            plot(f/1e6, abs(shortFreq), 'LineWidth', 1.5);
            title('Short Preamble Spectrum');
            xlabel('Frequency (MHz)');
            ylabel('Magnitude');
            grid on;
            
            % Long preamble spectrum
            subplot(2,3,2);
            longFreq = fftshift(fft(longPreamble));
            f = linspace(-1/(2*obj.Ts), 1/(2*obj.Ts), length(longFreq));
            plot(f/1e6, abs(longFreq), 'LineWidth', 1.5);
            title('Long Preamble Spectrum');
            xlabel('Frequency (MHz)');
            ylabel('Magnitude');
            grid on;
            
            % Payload spectrum
            subplot(2,3,3);
            payloadStart = obj.shortPreambleLength + obj.longPreambleLength + 1;
            payloadEnd = min(payloadStart + obj.payloadSymbolLength - 1, length(frame));
            payload = frame(payloadStart:payloadEnd);
            payloadFreq = fftshift(fft(payload));
            f = linspace(-1/(2*obj.Ts), 1/(2*obj.Ts), length(payloadFreq));
            plot(f/1e6, abs(payloadFreq), 'LineWidth', 1.5);
            title('Payload Symbol Spectrum');
            xlabel('Frequency (MHz)');
            ylabel('Magnitude');
            grid on;
            
            % Subcarrier allocation
            subplot(2,3,4);
            subcarrierMap = zeros(1, obj.Nc);
            subcarrierMap(obj.dataIndices) = 1;
            subcarrierMap(obj.pilotIndices) = 2;
            subcarrierMap(obj.nullIndices) = 0;
            stem(1:obj.Nc, subcarrierMap, 'filled');
            title('Subcarrier Allocation');
            xlabel('Subcarrier Index');
            ylabel('Type');
            yticks([0 1 2]);
            yticklabels({'Null', 'Data', 'Pilot'});
            grid on;
            
            % Occupied bandwidth
            subplot(2,3,5);
            fullFrameFreq = fftshift(fft(frame));
            f = linspace(-1/(2*obj.Ts), 1/(2*obj.Ts), length(fullFrameFreq));
            plot(f/1e6, 20*log10(abs(fullFrameFreq)), 'LineWidth', 1.5);
            title('Full Frame Spectrum (dB)');
            xlabel('Frequency (MHz)');
            ylabel('Magnitude (dB)');
            grid on;
            
            % Occupied bandwidth calculation
            totalPower = sum(abs(fullFrameFreq).^2);
            cumulativePower = cumsum(abs(fullFrameFreq).^2);
            idx99 = find(cumulativePower >= 0.99*totalPower, 1);
            bw99 = 2 * abs(f(idx99));
            
            xline(f(idx99)/1e6, 'r--', sprintf('99%% BW: %.2f MHz', bw99/1e6));
            xline(-f(idx99)/1e6, 'r--');
            
            % Spectral efficiency
            subplot(2,3,6);
            axis off;
            textStr = {
                sprintf('\\fontsize{12}\\bf Spectral Characteristics');
                '';
                sprintf('Sample Rate: %.2f MHz', (1/obj.Ts)/1e6);
                sprintf('Oversampling: %dx', obj.ovsamplingFactor);
                sprintf('Baseband Rate: %.2f MHz', obj.bbSampleRate/1e6);
                '';
                sprintf('Total Subcarriers: %d', obj.Nc);
                sprintf('Data Subcarriers: %d', obj.Ndata);
                sprintf('Pilot Subcarriers: %d', obj.Npilot);
                sprintf('Null Subcarriers: %d', length(obj.nullIndices));
                '';
                sprintf('99%% Bandwidth: %.2f MHz', bw99/1e6);
                sprintf('Spectral Efficiency: %.2f bits/s/Hz', ...
                    log2(obj.modulationOrder) * obj.Ndata / (bw99/1e6) / 1e6);
            };
            text(0.1, 0.9, textStr, 'VerticalAlignment', 'top', 'FontSize', 10);
            
            sgtitle('Frequency Domain Analysis', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function visualizeFrame(obj, frame)
            % Interactive frame visualization
            
            figure('Name', 'Frame Structure Visualization', 'NumberTitle', 'off');
            
            % Time-domain frame with annotations
            subplot(3,1,1);
            t = (0:length(frame)-1) * obj.Ts * 1e6;
            plot(t, abs(frame), 'LineWidth', 1.5);
            hold on;
            
            % Mark different sections
            xline(obj.shortPreambleLength * obj.Ts * 1e6, 'r--', 'Short Preamble End', 'LineWidth', 2);
            xline((obj.shortPreambleLength + obj.longPreambleLength) * obj.Ts * 1e6, ...
                'g--', 'Long Preamble End', 'LineWidth', 2);
            
            for i = 1:obj.nPayloadSymbols
                symbolStart = (obj.shortPreambleLength + obj.longPreambleLength + ...
                              (i-1) * obj.payloadSymbolLength) * obj.Ts * 1e6;
                xline(symbolStart, 'b:', sprintf('Symbol %d', i), 'LineWidth', 1.5);
            end
            
            title('Frame Structure (Amplitude)');
            xlabel('Time (\mus)');
            ylabel('Amplitude');
            grid on;
            
            % Phase
            subplot(3,1,2);
            plot(t, angle(frame) * 180/pi, 'LineWidth', 1);
            title('Phase');
            xlabel('Time (\mus)');
            ylabel('Phase (degrees)');
            grid on;
            
            % Waterfall (time-frequency)
            subplot(3,1,3);
            [s, f, t_spec] = spectrogram(frame, 64, 60, 64, 1/obj.Ts);
            imagesc(t_spec*1e6, f/1e6, 20*log10(abs(s)));
            axis xy;
            colormap(jet);
            colorbar;
            title('Time-Frequency Representation');
            xlabel('Time (\mus)');
            ylabel('Frequency (MHz)');
            
            sgtitle('Frame Structure Visualization', 'FontSize', 14, 'FontWeight', 'bold');
        end
        
        function printSummary(obj)
            % Print configuration summary
            fprintf('\n========== OFDM Configuration Summary ==========\n');
            fprintf('FFT Size (Nc):              %d\n', obj.Nc);
            fprintf('Data Subcarriers (Ndata):   %d\n', obj.Ndata);
            fprintf('Pilot Subcarriers (Npilot): %d\n', obj.Npilot);
            fprintf('Null Subcarriers:           %d\n', length(obj.nullIndices));
            fprintf('CP Length:                  %d samples\n', obj.cpLength);
            fprintf('Modulation:                 %d-PSK\n', obj.modulationOrder);
            fprintf('Payload Symbols:            %d\n', obj.nPayloadSymbols);
            fprintf('\n--- Timing ---\n');
            fprintf('Sample Time (Ts):           %.3f ns\n', obj.Ts * 1e9);
            fprintf('Baseband Sample Rate:       %.2f MHz\n', obj.bbSampleRate/1e6);
            fprintf('Oversampling Factor:        %dx\n', obj.ovsamplingFactor);
            fprintf('Effective Sample Rate:      %.2f MHz\n', (1/obj.Ts)/1e6);
            fprintf('\n--- Frame Structure ---\n');
            fprintf('Short Preamble:             %d samples (%.2f μs)\n', ...
                obj.shortPreambleLength, obj.shortPreambleLength * obj.Ts * 1e6);
            fprintf('Long Preamble:              %d samples (%.2f μs)\n', ...
                obj.longPreambleLength, obj.longPreambleLength * obj.Ts * 1e6);
            fprintf('Payload Symbol:             %d samples (%.2f μs)\n', ...
                obj.payloadSymbolLength, obj.payloadSymbolLength * obj.Ts * 1e6);
            fprintf('Total Frame:                %d samples (%.2f μs)\n', ...
                obj.frameLength, obj.frameLength * obj.Ts * 1e6);
            fprintf('\n--- Data Rate ---\n');
            bitsPerSymbol = log2(obj.modulationOrder) * obj.Ndata;
            symbolDuration = obj.payloadSymbolLength * obj.Ts;
            dataRate = bitsPerSymbol / symbolDuration;
            fprintf('Bits per OFDM Symbol:       %d bits\n', bitsPerSymbol);
            fprintf('Symbol Duration:            %.2f μs\n', symbolDuration * 1e6);
            fprintf('Data Rate:                  %.2f Mbps\n', dataRate/1e6);
            fprintf('================================================\n\n');
        end
    end
end
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
            obj.Nvirtual = obj.Nc - obj.Ndata - obj.Npilot - 1; % -1 for DC, as it is handled later.
            
            % Standard subcarrier allocation
            virtualLeft = floor(obj.Nvirtual/2);
            virtualRight = obj.Nvirtual - virtualLeft;
            
            % DC is at center (index Nc/2 + 1 in 1-indexed)
            dcIndex = obj.Nc/2 + 1;
            
            obj.nullIndices = [1:virtualLeft, dcIndex, (obj.Nc-virtualRight+1):obj.Nc]; % [1 x (Nvirtual + 1)]
            
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
            shortPreambleSlotTime = ifft(ifftshift(shortPreambleSlotFreq));
            shortPreamble = repmat(shortPreambleSlotTime(1:16), 1, 10);
        end
        
        function longPreamble = generateLongPreamble(obj)
            virtualSubcarrier = zeros(1, obj.Nc - length(obj.longPreambleSeq));
            longPreambleSlotFreq = [virtualSubcarrier(1:6), obj.longPreambleSeq, virtualSubcarrier(7:11)];
            longPreambleSlotTime = ifft(ifftshift(longPreambleSlotFreq));
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
            timeSymbol = ifft(ifftshift(freqSymbol));
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
            txSignal = conv(frameOversampled, RRC);
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
            parse(p, varargin{:});
            
            debugMode = p.Results.debug;
            threshold = p.Results.threshold;
            

            rxFiltered = obj.applyMatchedFilter(rxSignal);
            [packetStart, M_n, thresholdGraph] = obj.detectPacket(rxFiltered, threshold);
            frameDownsampled = obj.downsampleFrame(rxFiltered, packetStart);
            [frameAfterCFO, coarseCFO, fineCFO] = obj.correctCFO(frameDownsampled);
            [H_est, H_est_time] = obj.estimateChannel(frameAfterCFO);
            [decodedBits, rxPayloads] = obj.demodulatePayloads(frameAfterCFO, H_est);
            
            rxMetrics = struct();
            rxMetrics.packetStart = packetStart;
            rxMetrics.M_n = M_n;
            rxMetrics.thresholdGraph = thresholdGraph;
            rxMetrics.coarseCFO = coarseCFO;
            rxMetrics.fineCFO = fineCFO;
            rxMetrics.H_est = H_est;
            rxMetrics.H_est_time = H_est_time;
            rxMetrics.rxPayloads = rxPayloads;
            
            % Debug plotting
            if debugMode
                obj.plotRxDebug(rxSignal, rxMetrics, decodedBits);
            end
        end
        
        function rxFiltered = applyMatchedFilter(obj, rxSignal)
            rolloff = 0.5;
            L = 6;
            RRC = rcosdesign(rolloff, L, obj.ovsamplingFactor, 'sqrt');
            rxFiltered = conv(rxSignal, RRC);
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
                    C_k(k) = rxSignal(n + k - 1) * conj(rxSignal(n + k - 1 + D));
                    P_k(k) = abs(rxSignal(n + k - 1 + D))^2;
                end
                C_n(n) = sum(C_k);
                P_n(n) = sum(P_k);
            end
            
            % Calculate detection metric
            M_n = (abs(C_n).^2) ./ (P_n.^2);
            
            % Find packet start
            loc = find(M_n > threshold);
            if isempty(loc)
                error('No packet detected above threshold');
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
                error('Frame extends beyond received signal length');
            end
            frameDownsampled = rxSignal(packetStart:obj.ovsamplingFactor:endIdx);
        end
        
        function [frameAfterCFO, coarseCFO, fineCFO] = correctCFO(obj, frameDownsampled)
            shortPreambleSlotLength = 16;
            % Coarse CFO estimation using short preamble
            z1 = frameDownsampled(shortPreambleSlotLength*5+1:shortPreambleSlotLength*6) * conj(frameDownsampled(shortPreambleSlotLength*6+1:shortPreambleSlotLength*7)).';
            coarseCFO = (-1/(2*pi*shortPreambleSlotLength*obj.Ts)) * angle(z1);
            % Apply coarse correction
            timeVector = obj.Ts * (0:obj.frameLength-1);
            frameAfterCoarse = frameDownsampled .* exp(-1j*2*pi*coarseCFO*timeVector); 
            % Fine CFO estimation using long preamble
            z2 = frameAfterCoarse(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16) * frameAfterCoarse(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20)';
            fineCFO = (-1/(2*pi*64*obj.Ts)) * angle(z2);
            
            frameAfterCFO = frameAfterCoarse .* exp(-1j*2*pi*fineCFO*timeVector);
        end
        
        function [H_est, H_est_time] = estimateChannel(obj, frameAfterCFO)
            % Channel estimation using long preamble symbols
            shortPreambleSlotLength = 16;
            
            % Extract long preamble symbols
            longPreamble1 = frameAfterCFO(shortPreambleSlotLength*12+1:shortPreambleSlotLength*16);
            longPreamble2 = frameAfterCFO(shortPreambleSlotLength*16+1:shortPreambleSlotLength*20);
            
            % Convert to frequency domain
            longPreamble1FFT = fftshift(fft(longPreamble1));
            longPreamble2FFT = fftshift(fft(longPreamble2));
            
            % Average the two symbols and compare with known sequence
            longPreambleRef = obj.generateLongPreambleFreqDomain();
            H_est = 0.5 * (longPreamble1FFT + longPreamble2FFT) .* conj(longPreambleRef);
            
            H_est_time = ifft(ifftshift(H_est));
        end
        
        function [decodedBits, rxPayloads] = demodulatePayloads(obj, frameAfterCFO, H_est)
            % Demodulate all payload symbols
            payloadStartIdx = obj.shortPreambleLength + obj.longPreambleLength + 1;
            
            decodedBits = cell(1, obj.nPayloadSymbols);
            rxPayloads = struct();
            rxPayloads.noEqualizer = cell(1, obj.nPayloadSymbols);
            rxPayloads.withEqualizer = cell(1, obj.nPayloadSymbols);
            
            for i = 1:obj.nPayloadSymbols
                % Extract payload symbol
                symbolStart = payloadStartIdx + (i-1) * obj.payloadSymbolLength;
                symbolEnd = symbolStart + obj.payloadSymbolLength - 1;
                payloadTime = frameAfterCFO(symbolStart:symbolEnd);
                
                % Remove cyclic prefix
                payloadNoCP = payloadTime(obj.cpLength+1:end);
                
                % Convert to frequency domain
                payloadFreq = fftshift(fft(payloadNoCP));
                
                % Apply equalizer
                payloadFreqEq = payloadFreq ./ H_est;
                
                % Extract data subcarriers (remove pilots and nulls)
                dataSymbolsNoEq = payloadFreq(obj.dataIndices);
                dataSymbolsEq = payloadFreqEq(obj.dataIndices);
                
                % Store for debugging
                rxPayloads.noEqualizer{i} = dataSymbolsNoEq;
                rxPayloads.withEqualizer{i} = dataSymbolsEq;
                
                % Demodulate
                if obj.modulationOrder == 2
                    decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, 0);
                else
                    decodedBits{i} = pskdemod(dataSymbolsEq, obj.modulationOrder, pi/4);
                end
            end
        end
        
        function BER = calculateBER(~, decodedBits, referenceData)
            % Calculate bit error rate
            if length(decodedBits) ~= length(referenceData)
                error('Number of payload symbols mismatch');
            end
            
            totalErrors = 0;
            totalBits = 0;
            
            for i = 1:length(decodedBits)
                errors = sum(abs(sign(referenceData{i} - decodedBits{i})));
                totalErrors = totalErrors + errors;
                totalBits = totalBits + length(referenceData{i});
            end
            
            BER = totalErrors / totalBits;
        end
        
        function plotRxDebug(obj, rxSignal, rxMetrics)
            % Debug plotting similar to original OFDM_RX
            figure('Units', 'centimeters', 'Position', [1 2 49 24]);
            
            % Raw received signal constellation
            subplot(2,4,1);
            plot(rxSignal, '.');
            title('RX-Raw');
            axis([-1.5 1.5 -1.5 1.5]);
            axis square;
            
            % I component
            subplot(2,4,2);
            plot(real(rxSignal));
            title('I');
            axis([1 3000 -1.5 1.5]);
            axis square;
            
            % Q component
            subplot(2,4,3);
            plot(imag(rxSignal));
            title('Q');
            axis([1 3000 -1.5 1.5]);
            axis square;
            
            % Power spectral density
            subplot(2,4,4);
            [spectrumWaveform, welchFreq] = pwelch(rxSignal, [], [], [], 1/obj.Ts, 'centered', 'power');
            plot(welchFreq, pow2db(spectrumWaveform));
            title('Welch Power Spectral Density');
            axis square;
            
            % Packet detection
            subplot(2,4,5);
            plot(1:length(rxMetrics.M_n), rxMetrics.M_n, 1:length(rxMetrics.M_n), rxMetrics.thresholdGraph);
            title('Packet Detection');
            axis([1, length(rxMetrics.M_n), 0, 1.2]);
            axis square;
            
            % Channel estimation
            subplot(2,4,6);
            plot(abs(rxMetrics.H_est_time));
            title('Channel Estimation');
            axis([1 64 0 7]);
            axis square;
            xlabel('Time');
            
            % Before equalizer
            subplot(2,4,7);
            hold on;
            for i = 1:obj.nPayloadSymbols
                plot(rxMetrics.rxPayloads.noEqualizer{i}, '*');
            end
            title('Before Equalizer');
            axis([-8 8 -8 8]);
            axis square;
            hold off;
            
            % After equalizer (demodulation)
            subplot(2,4,8);
            hold on;
            for i = 1:obj.nPayloadSymbols
                plot(rxMetrics.rxPayloads.withEqualizer{i}, '*');
            end
            title('Demodulation');
            axis([-1.5 1.5 -1.5 1.5]);
            axis square;
            hold off;
        end
    end
end
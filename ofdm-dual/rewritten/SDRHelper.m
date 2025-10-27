classdef SDRHelper < handle
    properties
        sampleRate = 1e6;           
        centerFrequency = 2.4e9;    
        txGain = -20;               
        rxGain = 40;
        radioID = "usb:0";
    
        txFrameLength = 4096;       
        rxFrameLength = 8192;       
        txRepeat = false;           
    
        txSDR = [];                 
        rxSDR = [];                 
    
        isConnected = false;        
        isTxConfigured = false;     
        isRxConfigured = false;     
    
        txScaleFactor = 0.5;        
        enableFiltering = true;
        
        % Statistics
        txUnderflowCount = 0;
        rxOverflowCount = 0;
    end
    
    methods
        function obj = SDRHelper(varargin)
            % Constructor with optional parameter pairs
            % Usage: sdr = SDRHelper('sampleRate', 2e6, 'centerFrequency', 915e6)
            
            p = inputParser;
            addParameter(p, 'sampleRate', 1e6, @isnumeric);
            addParameter(p, 'centerFrequency', 0.915e9, @isnumeric);
            addParameter(p, 'txGain', -20, @isnumeric);
            addParameter(p, 'rxGain', 40, @isnumeric);
            addParameter(p, 'txFrameLength', 4096, @isnumeric);
            addParameter(p, 'rxFrameLength', 8192, @isnumeric);
            addParameter(p, 'radioID', "usb:0");
            parse(p, varargin{:});
            
            obj.sampleRate = p.Results.sampleRate;
            obj.centerFrequency = p.Results.centerFrequency;
            obj.txGain = p.Results.txGain;
            obj.rxGain = p.Results.rxGain;
            obj.txFrameLength = p.Results.txFrameLength;
            obj.rxFrameLength = p.Results.rxFrameLength;
            obj.radioID = p.Results.radioID;
        end
        
        function success = connect(obj)
            try
                if ~license('test', 'Communication_Toolbox')
                    error('Communications Toolbox required for ADALM PLUTO support');
                end
                devices = findPlutoRadio();
                if isempty(devices)
                    error('No ADALM PLUTO devices found. Check USB connection.');
                end
                obj.isConnected = true;
                success = true;
                
            catch ME
                obj.isConnected = false;
                rethrow(ME);
            end
        end
        
        function success = configureTx(obj)
            if ~obj.isConnected
                obj.connect();
            end

            try
                obj.txSDR = sdrtx('Pluto', ...
                    'RadioID', obj.radioID, ...
                    'CenterFrequency', obj.centerFrequency, ...
                    'BasebandSampleRate', obj.sampleRate, ...
                    'Gain', obj.txGain, ...
                    'ShowAdvancedProperties', true);

                obj.isTxConfigured = true;
                success = true;
            catch ME
                obj.isTxConfigured = false;
                rethrow(ME);
            end
        end

        function success = configureRx(obj)
            if ~obj.isConnected
                obj.connect();
            end

            try
                % Create receiver object
                obj.rxSDR = sdrrx('Pluto', ...
                    'RadioID', obj.radioID, ...
                    'CenterFrequency', obj.centerFrequency, ...
                    'BasebandSampleRate', obj.sampleRate, ...
                    'GainSource', 'Manual', ...
                    'Gain', obj.rxGain, ...
                    'SamplesPerFrame', obj.rxFrameLength, ...
                    'OutputDataType', 'double', ...
                    'ShowAdvancedProperties', true);

                obj.isRxConfigured = true;
                success = true;
                
            catch ME
                obj.isRxConfigured = false;
                rethrow(ME);
            end
        end
        
        function txSignalOut = conditionTxSignal(obj, txSignal)
            % Condition and prepare TX signal
            % Now includes repetition to fill frame length
            
            % Scale signal
            txSignalOut = txSignal * obj.txScaleFactor;
            
            % Normalize to prevent clipping
            maxVal = max(abs(txSignalOut));
            if maxVal > 1
                txSignalOut = txSignalOut / maxVal;
            end

            % Ensure column vector
            if isrow(txSignalOut)
                txSignalOut = txSignalOut.';
            end

            % Handle frame length - REPEAT SHORT SIGNALS
            signalLength = length(txSignalOut);
            
            if signalLength > obj.txFrameLength
                % Truncate if too long
                txSignalOut = txSignalOut(1:obj.txFrameLength);
                warning('SDRHelper:SignalTruncated', ...
                    'TX signal truncated from %d to %d samples', ...
                    signalLength, obj.txFrameLength);
                
            elseif signalLength < obj.txFrameLength
                % REPEAT SIGNAL UNTIL IT FILLS OR EXCEEDS FRAME LENGTH
                numRepeats = ceil(obj.txFrameLength / signalLength);
                txSignalOut = repmat(txSignalOut, numRepeats, 1);
                
                % Truncate to exact frame length
                txSignalOut = txSignalOut(1:obj.txFrameLength);
                
                fprintf('TX signal repeated %d times to fill frame (%d -> %d samples)\n', ...
                    numRepeats, signalLength, obj.txFrameLength);
            end
        end

        function [success, underflow] = transmit(obj, txSignal, varargin)
            % Transmit signal
            % Optional parameters: 
            %   'repeat', true/false - use transmitRepeat for continuous TX
            %   'duration', seconds - how long to transmit in repeat mode
            %
            % Returns:
            %   success - boolean indicating successful transmission
            %   underflow - underflow count/indicator
            
            if ~obj.isTxConfigured
                obj.configureTx();
            end
            
            p = inputParser;
            addParameter(p, 'repeat', false, @islogical);
            addParameter(p, 'duration', inf, @isnumeric);
            parse(p, varargin{:});
            
            try
                txSignalConditioned = obj.conditionTxSignal(txSignal);
                
                if p.Results.repeat
                    % Use transmitRepeat for continuous transmission
                    transmitRepeat(obj.txSDR, txSignalConditioned);
                    fprintf('Transmission started in repeat mode...\n');
                    
                    if isinf(p.Results.duration)
                        fprintf('Press Enter to stop transmission...\n');
                        input('', 's');
                    else
                        fprintf('Transmitting for %.2f seconds...\n', p.Results.duration);
                        pause(p.Results.duration);
                    end
                    
                    % Release to stop transmission
                    release(obj.txSDR);
                    fprintf('Transmission stopped.\n');
                    underflow = 0; % transmitRepeat doesn't return underflow
                    
                else
                    % Single transmission using correct API
                    underflow = obj.txSDR(txSignalConditioned);
                    
                    if underflow
                        obj.txUnderflowCount = obj.txUnderflowCount + 1;
                        warning('SDRHelper:TxUnderflow', ...
                            'TX underflow detected (total: %d)', obj.txUnderflowCount);
                    end
                end
                
                success = true;
                
            catch ME
                success = false;
                underflow = -1;
                rethrow(ME);
            end
        end

        function [rxFrame, overflow] = step(obj)
            % Receive a single frame 
            % Returns: 
            %   rxFrame - column vector of received samples
            %   overflow - overflow indicator (true if samples were lost)
            
            if ~obj.isRxConfigured
                error('RX not configured. Call configureRx() first.');
            end
            
            [rxFrame, ~, overflow] = obj.rxSDR();
            
            if overflow
                obj.rxOverflowCount = obj.rxOverflowCount + 1;
                warning('SDRHelper:RxOverflow', ...
                    'RX overflow detected (total: %d)', obj.rxOverflowCount);
            end
        end
        
        function [rxSignal, success, stats] = receive(obj, varargin)
            % Receive signal
            % Optional parameters: 
            %   'duration', seconds - how long to receive
            %   'numFrames', integer - specific number of frames
            %   'autoRelease', true/false - release after receiving (default: true)
            %   'callback', function handle - called after each frame
            %
            % Returns:
            %   rxSignal - received signal
            %   success - boolean
            %   stats - structure with overflow count and other info
            
            if ~obj.isRxConfigured
                error('RX not configured. Call configureRx() first.');
            end
            
            % Parse options
            p = inputParser;
            addParameter(p, 'duration', 1, @isnumeric);
            addParameter(p, 'numFrames', [], @isnumeric);
            addParameter(p, 'autoRelease', true, @islogical);
            addParameter(p, 'callback', [], @(x) isa(x, 'function_handle') || isempty(x));
            parse(p, varargin{:});
            
            try
                if ~isempty(p.Results.numFrames)
                    % Receive specific number of frames
                    numFrames = p.Results.numFrames;
                else
                    % Receive for specified duration
                    duration = p.Results.duration;
                    numFrames = ceil(duration * obj.sampleRate / obj.rxFrameLength);
                end
                
                % Pre-allocate
                rxSignal = zeros(numFrames * obj.rxFrameLength, 1);
                
                % Statistics
                overflowFrames = false(numFrames, 1);
                startTime = tic;
                
                % Receive frames
                for i = 1:numFrames
                    startIdx = (i-1) * obj.rxFrameLength + 1;
                    endIdx = i * obj.rxFrameLength;
                    
                    % Receive with overflow detection
                    [rxFrame, ~, overflow] = obj.rxSDR();
                    rxSignal(startIdx:endIdx) = rxFrame;
                    overflowFrames(i) = overflow;
                    
                    if overflow
                        obj.rxOverflowCount = obj.rxOverflowCount + 1;
                    end
                    
                    % Call user callback if provided
                    if ~isempty(p.Results.callback)
                        p.Results.callback(rxFrame, i, numFrames);
                    end
                end
                
                elapsedTime = toc(startTime);
                
                % Optional release
                if p.Results.autoRelease
                    release(obj.rxSDR);
                end
                
                % Build statistics
                stats.numFrames = numFrames;
                stats.totalSamples = length(rxSignal);
                stats.overflowCount = sum(overflowFrames);
                stats.overflowFrames = find(overflowFrames);
                stats.elapsedTime = elapsedTime;
                stats.effectiveRate = length(rxSignal) / elapsedTime;
                
                success = true;
                
                % Report if overflows occurred
                if stats.overflowCount > 0
                    warning('SDRHelper:RxOverflow', ...
                        '%d overflows in %d frames (%.1f%%)', ...
                        stats.overflowCount, numFrames, ...
                        100*stats.overflowCount/numFrames);
                end
                
            catch ME
                success = false;
                stats = struct();
                rethrow(ME);
            end
        end
        
        function [success] = receiveRealtime(obj, callback, varargin)
            % Real-time receiver with callback processing
            % 
            % Parameters:
            %   callback - function handle(rxFrame, frameNum) for processing each frame
            %   'duration', seconds - how long to receive (default: inf)
            %   'maxFrames', integer - maximum frames to receive
            %
            % Example:
            %   sdr.receiveRealtime(@(frame,n) plot(abs(fft(frame))), 'duration', 10);
            
            if ~obj.isRxConfigured
                error('RX not configured. Call configureRx() first.');
            end
            
            p = inputParser;
            addParameter(p, 'duration', inf, @isnumeric);
            addParameter(p, 'maxFrames', inf, @isnumeric);
            parse(p, varargin{:});
            
            try
                frameCount = 0;
                startTime = tic;
                
                fprintf('Real-time reception started. Press Ctrl+C to stop.\n');
                
                while toc(startTime) < p.Results.duration && frameCount < p.Results.maxFrames
                    [rxFrame, overflow] = obj.step();
                    frameCount = frameCount + 1;
                    
                    % Call user callback
                    callback(rxFrame, frameCount);
                    
                    % Optional: display overflow warnings
                    if overflow && mod(frameCount, 100) == 0
                        fprintf('Overflow detected at frame %d\n', frameCount);
                    end
                end
                
                release(obj.rxSDR);
                fprintf('Received %d frames in %.2f seconds\n', frameCount, toc(startTime));
                success = true;
                
            catch ME
                if strcmp(ME.identifier, 'MATLAB:pilstack')
                    fprintf('\nReception interrupted by user.\n');
                    release(obj.rxSDR);
                    success = true;
                else
                    success = false;
                    rethrow(ME);
                end
            end
        end
                
        function plotSpectrum(obj, signal, varargin)
            % Plot signal spectrum
            p = inputParser;
            addParameter(p, 'title', 'Signal Spectrum', @ischar);
            addParameter(p, 'fftSize', 1024, @isnumeric);
            parse(p, varargin{:});
            
            [psd, f] = pwelch(signal, [], [], p.Results.fftSize, obj.sampleRate, 'centered');
            
            figure;
            plot(f/1e6, 10*log10(psd));
            title(p.Results.title);
            xlabel('Frequency (MHz)');
            ylabel('PSD (dB/Hz)');
            grid on;
        end
        
        function printStats(obj)
            % Print transmission/reception statistics
            fprintf('\n=== SDR Statistics ===\n');
            fprintf('TX Underflows: %d\n', obj.txUnderflowCount);
            fprintf('RX Overflows: %d\n', obj.rxOverflowCount);
            fprintf('Sample Rate: %.2f MHz\n', obj.sampleRate/1e6);
            fprintf('Center Frequency: %.2f MHz\n', obj.centerFrequency/1e6);
            fprintf('TX Gain: %.2f dB\n', obj.txGain);
            fprintf('RX Gain: %.2f dB\n', obj.rxGain);
            fprintf('=====================\n\n');
        end

        function cleanup(obj)
            try
                if ~isempty(obj.txSDR)
                    release(obj.txSDR);
                    obj.txSDR = [];
                end
                if ~isempty(obj.rxSDR)
                    release(obj.rxSDR);
                    obj.rxSDR = [];
                end
                obj.isConnected = false;
                obj.isTxConfigured = false;
                obj.isRxConfigured = false;
            catch
                % Silent cleanup
            end
        end
    end
    
    methods (Access = private)
        function delete(obj)
            obj.cleanup();
        end
    end
end
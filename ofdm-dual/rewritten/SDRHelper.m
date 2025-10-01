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
    end
    
    methods
        function obj = SDRHelper(varargin)
            % Constructor with optional parameter pairs
            % Usage: sdr = SDRHelper('sampleRate', 2e6, 'centerFrequency', 915e6)
            
            p = inputParser;
            addParameter(p, 'sampleRate', 1e6, @isnumeric);
            addParameter(p, 'centerFrequency', 2.4e9, @isnumeric);
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
                    'RadioID', 'usb:0', ...
                    'CenterFrequency', obj.centerFrequency, ...
                    'BasebandSampleRate', obj.sampleRate, ...
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
            
            txSignalOut = txSignal * obj.txScaleFactor;
            
            maxVal = max(abs(txSignalOut));
            if maxVal > 1
                txSignalOut = txSignalOut / maxVal;
            end
            
            if isrow(txSignalOut)
                txSignalOut = txSignalOut.';
            end
            
            % Pad or truncate to frame length if needed
            if length(txSignalOut) > obj.txFrameLength
                txSignalOut = txSignalOut(1:obj.txFrameLength);
            elseif length(txSignalOut) < obj.txFrameLength
                padding = zeros(obj.txFrameLength - length(txSignalOut), 1);
                txSignalOut = [txSignalOut; padding];
            end
        end
        
        function success = transmit(obj, txSignal, varargin)
            % Transmit signal
            % Optional parameters: 'repeat', true/false, 'duration', seconds
            
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
                    obj.txSDR.transmitRepeat(txSignalConditioned);
                    if isinf(p.Results.duration)
                        input('Press Enter to stop transmission...\n');
                    else
                        pause(p.Results.duration);
                    end
                    release(obj.txSDR);
                else
                    obj.txSDR.step(txSignalConditioned);
                    release(obj.txSDR);
                end
                
                success = true;
                
            catch ME
                rethrow(ME);
            end
        end

        function rxFrame = step(obj)
            % Receive a single frame (mimics legacy System Object API)
            % Returns: rxFrame - column vector of received samples
            
            if ~obj.isRxConfigured
                error('RX not configured. Call configureRx() first.');
            end
            
            rxFrame = obj.rxSDR();
        end
        
        function [rxSignal, success] = receive(obj, varargin)
            % Receive signal
            % Optional parameters: 'duration', seconds, 'numFrames', integer
            
            if ~obj.isRxConfigured
                error('RX not configured. Call configureRx() first.');
            end
            
            % Parse options
            p = inputParser;
            addParameter(p, 'duration', 1, @isnumeric);
            addParameter(p, 'numFrames', [], @isnumeric);
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
                
                % Receive frames
                for i = 1:numFrames
                    startIdx = (i-1) * obj.rxFrameLength + 1;
                    endIdx = i * obj.rxFrameLength;
                    rxSignal(startIdx:endIdx) = obj.rxSDR();
                end
                
                release(obj.rxSDR);
                
                success = true;
                
            catch ME
                rethrow(ME);
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

            end
        end
    end
    
    methods (Access = private)
        function delete(obj)
            obj.cleanup();
        end
    end
end
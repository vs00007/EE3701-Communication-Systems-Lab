clear; close all; clc;

%% Initialize
sampleRate = 1e6;
centerFrequency = 0.915e9;
rxGain = 40;
samplesPerFrame = 3000;
scale = 1024;

% Initialize OFDM helper
ofdm = OFDMHelper();
ofdm = ofdm.loadConfig('ofdm_config.mat');

% Initialize SDR
sdr = SDRHelper('sampleRate', sampleRate, ...
                'centerFrequency', centerFrequency, ...
                'rxGain', rxGain, ...
                'radioID', "usb:1",...
                'rxFrameLength', samplesPerFrame);

%% Setup GUI
figure('Name', 'OFDM Real-time Receiver', 'NumberTitle', 'off');
set(gcf, 'Units', 'centimeters', 'position', [1 2 49 24]);

%% Connect and configure SDR
try
    sdr.connect();
    sdr.configureRx();
    fprintf('SDR connected and configured\n');
catch ME
    fprintf('SDR setup failed: %s\n', ME.message);
    return;
end

%% Main reception loop
fprintf('Starting real-time reception (close figure to stop)...\n');
runNumber = 1;
readyTime = 5; % Skip first few frames

while ishandle(gcf)
        % Receive one frame (replace with your actual SDRHelper method)
        rxData = step(sdr.rxSDR);
        
        if runNumber > readyTime
            % Scale and format data
            rxScaled = double(rxData) / scale;
            RX = rxScaled.'; % Convert to row vector [1 x 3000]
            
            % Plot raw received signal
            subplot(2,4,1);
            plot(RX, '.'); 
            title('RX-Raw'); 
            axis([-1.5 1.5 -1.5 1.5]); 
            axis square;
            
            subplot(2,4,2);
            plot(real(RX)); 
            title('I'); 
            axis([1 3000 -1.5 1.5]); 
            axis square;
            
            subplot(2,4,3);
            plot(imag(RX)); 
            title('Q'); 
            axis([1 3000 -1.5 1.5]); 
            axis square;
            
            % Power spectral density
            [psd, f] = pwelch(RX, [], [], [], sampleRate, 'centered', 'power');
            subplot(2,4,4);
            plot(f, pow2db(psd));
            title('Welch Power Spectral Density');
            axis([-sampleRate/2 sampleRate/2 -100 -10]); 
            axis square;
            
            % OFDM demodulation
            try
                [decodedBits, metrics] = ofdm.receiveFrame(RX);
                
                % Plot packet detection metric and threshold
                subplot(2,4,5);
                plot(1:length(metrics.M_n), metrics.M_n, ...
                     1:length(metrics.M_n), metrics.thresholdGraph);
                title('Packet Detection');
                axis([1, length(metrics.M_n), 0, 1.2]); 
                axis square;
                
                % Plot channel impulse response
                subplot(2,4,6);
                plot(abs(metrics.H_est_time));
                title('Channel Estimation');
                axis([1 64 0 7]); 
                axis square;
                xlabel('Time');
                
                % Plot both payload symbols before equalization
                subplot(2,4,7);
                plot([metrics.rxPayloads.noEqualizer{:}], '*');
                title('Before Equalizer');
                axis([-8 8 -8 8]); 
                axis square;
                
                % Plot both payload symbols after equalization
                subplot(2,4,8);
                plot([metrics.rxPayloads.withEqualizer{:}], '*');
                
                % Calculate BER if reference data exists
                BER = 0;
                if exist('tx_payload_data.mat', 'file')
                    load('tx_payload_data.mat', 'payloadData');
                    BER = ofdm.calculateBER(decodedBits, payloadData);
                end
                
                title({'Demodulation'; ['BER = ', num2str(BER)]});
                axis([-1.5 1.5 -1.5 1.5]); 
                axis square;
                
            catch demodError
                % If demodulation fails, just continue
                fprintf('Demod failed: %s\n', demodError.message);
            end
            
            drawnow; % Update plots
        end
        
        runNumber = runNumber + 1;
        
    
    % Small pause to prevent overwhelming
    pause(0.01);
end

%% Cleanup
sdr.cleanup();
fprintf('Real-time reception stopped\n');
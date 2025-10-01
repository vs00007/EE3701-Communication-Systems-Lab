clear; clc; close all;

%% Parameters
sampleRate = 1e6;
centerFrequency = 0.915e9;
txGain = -20;
rxGain = 40;

% OFDM Parameters
Nc = 64;
Ndata = 48;
Npilot = 4;
cpLength = 16;
modulationOrder = 4;
nPayloadSymbols = 2;
ovsamplingFactor = 2;

%% Initialize Objects
ofdm = OFDMHelper(Nc, Ndata, Npilot, cpLength, modulationOrder, ...
                  nPayloadSymbols, sampleRate, ovsamplingFactor);

sdr = SDRHelper('sampleRate', sampleRate, ...
                'centerFrequency', centerFrequency, ...
                'txGain', txGain, ...
                'rxGain', rxGain, ...
                'radioID', "usb:0",...
                'txFrameLength', 4096, ...
                'rxFrameLength', 8192);

% Save configurations
ofdm.saveConfig('ofdm_config.mat');

%% Mode Selection
% 1 for TX, 2 for RX
mode = 1;

switch mode
    case 1
        %% TRANSMIT MODE
        fprintf('\n=== TRANSMIT MODE ===\n');
       
            % Connect and configure SDR for TX
            sdr.connect();
            sdr.configureTx();
            
            % Generate random data for payload symbols
            payloadData = cell(1, nPayloadSymbols);
            for i = 1:nPayloadSymbols
                payloadData{i} = randi([0 modulationOrder-1], 1, Ndata);
            end
            
            % Save payload data for RX reference
            save('tx_payload_data.mat', 'payloadData');
            
            % Generate OFDM frame
            txSignal = ofdm.getTransmitFrame(payloadData{:});
            
            fprintf('Frame: %d samples, %.2f ms duration\n', ...
                    length(txSignal), length(txSignal)/(sampleRate*ovsamplingFactor)*1000);
            
            % Visualization
            figure('Name', 'TX Signal Analysis');
            subplot(2,2,1);
            t = (0:length(txSignal)-1) * (1/(sampleRate*ovsamplingFactor)) * 1e6;
            plot(t, real(txSignal));
            title('TX Signal - Real Part');
            xlabel('Time (μs)'); ylabel('Amplitude'); grid on;
            
            subplot(2,2,2);
            plot(t, imag(txSignal));
            title('TX Signal - Imaginary Part');
            xlabel('Time (μs)'); ylabel('Amplitude'); grid on;
            
            subplot(2,2,3);
            plot(txSignal, '.');
            title('TX Constellation');
            xlabel('In-phase'); ylabel('Quadrature');
            axis equal; grid on;
            
            subplot(2,2,4);
            [psd, f] = pwelch(txSignal, [], [], [], sampleRate*ovsamplingFactor, 'centered');
            plot(f/1e6, 10*log10(psd));
            title('TX Signal PSD');
            xlabel('Frequency (MHz)'); ylabel('PSD (dB/Hz)'); grid on;
            
            % Start transmission
            fprintf('Starting transmission (press Enter to stop)...\n');
            sdr.transmit(txSignal, 'repeat', true);
           
        
    case 2
        %% RECEIVE MODE
        fprintf('\n=== RECEIVE MODE ===\n');
        
        try
            % Load OFDM configuration
            ofdm_rx = OFDMHelper();
            ofdm_rx = ofdm_rx.loadConfig('ofdm_config.mat');
            
            % Configure SDR for RX
            sdr.connect();
            sdr.configureRx();
            
            % Receive signal
            rxDuration = 3;
            fprintf('Receiving for %d seconds...\n', rxDuration);
            [rxSignal, success] = step(sdr.rxSDR);
            
            if success
                fprintf('Received %d samples\n', length(rxSignal));
                
                % Basic RX signal analysis
                figure('Name', 'Received Signal Overview');
                subplot(2,2,1);
                plot(real(rxSignal(1:min(2000, end))));
                title('RX Signal - Real Part');
                xlabel('Sample'); ylabel('Amplitude'); grid on;
                
                subplot(2,2,2);
                plot(imag(rxSignal(1:min(2000, end))));
                title('RX Signal - Imaginary Part');
                xlabel('Sample'); ylabel('Amplitude'); grid on;
                
                subplot(2,2,3);
                plot(rxSignal(1:min(2000, end)), '.');
                title('RX Constellation');
                xlabel('Real'); ylabel('Imaginary');
                axis equal; grid on;
                
                subplot(2,2,4);
                [psd, f] = pwelch(rxSignal, [], [], [], sampleRate, 'centered');
                plot(f/1e6, 10*log10(psd));
                title('RX Signal PSD');
                xlabel('Frequency (MHz)'); ylabel('PSD (dB/Hz)'); grid on;
                
                    % OFDM demodulation
                    fprintf('Attempting OFDM demodulation...\n');
                    try
                        [decodedBits, rxMetrics] = ofdm_rx.receiveFrame(rxSignal);
                        
                        fprintf('Decoded %d payload symbols\n', length(decodedBits));
                        fprintf('Coarse CFO: %.2f Hz, Fine CFO: %.2f Hz\n', ...
                                rxMetrics.coarseCFO, rxMetrics.fineCFO);
                        fprintf('Packet start: %d\n', rxMetrics.packetStart);
                        
                        % Show basic RX processing plots
                        figure('Name', 'RX Processing Results');
                        subplot(2,2,1);
                        plot(rxMetrics.M_n);
                        title('Packet Detection Metric');
                        xlabel('Sample'); ylabel('Correlation'); grid on;
                        
                        subplot(2,2,2);
                        plot(abs(rxMetrics.H_est));
                        title('Channel Estimate');
                        xlabel('Subcarrier'); ylabel('|H|'); grid on;
                        
                        subplot(2,2,3);
                        plot(rxMetrics.rxPayloads.withEqualizer{1}, 'o');
                        title('RX Constellation (Symbol 1)');
                        xlabel('In-phase'); ylabel('Quadrature');
                        axis equal; grid on;
                        
                        subplot(2,2,4);
                        bar([rxMetrics.coarseCFO, rxMetrics.fineCFO]);
                        title('CFO Estimates');
                        set(gca, 'XTickLabel', {'Coarse', 'Fine'});
                        ylabel('Frequency (Hz)'); grid on;
                        
                        % BER calculation if TX data available
                        if exist('tx_payload_data.mat', 'file')
                            load('tx_payload_data.mat', 'payloadData');
                            BER = ofdm_rx.calculateBER(decodedBits, payloadData);
                            fprintf('Bit Error Rate: %.4f\n', BER);
                        end
                    
                catch demodError
                    fprintf('OFDM demodulation failed: %s\n', demodError.message);
                    fprintf('Possible causes: No signal, low SNR, timing issues\n');
                end
            else
                fprintf('Reception failed\n');
            end
            
        catch ME
            fprintf('RX Error: %s\n', ME.message);
        end
        
    otherwise
        fprintf('Invalid mode selection\n');
end

sdr.cleanup();
fprintf('Done.\n');
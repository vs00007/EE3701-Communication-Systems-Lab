clear;close all;clc;j=1i;
Global_Parameters;
%% Button setting
figure('Name','RX','NumberTitle','off');
button = uicontrol; % Generate GUI button
set(button,'String','Stop !','Position',[1475 15 100 60]); % Add "Stop !" text
set(gcf,'Units','centimeters','position',[1 2 49 24]); % Set the postion of GUI
channel_sim = 1;
Fs = 40e6;
%% Hardware Parameters
rx_object = sdrrx('Pluto','Gain',30, 'CenterFrequency', 0.915e9, 'SamplesPerFrame', 3000);
tx_object = sdrtx('Pluto','Gain',-10, 'CenterFrequency', 1e9);

back_payload_QPSK = 'Mod Order 4.';
back_payload_BPSK = 'Mod Order 2.';

back_payload_BPSK = int2bit(double(back_payload_BPSK), 8);
back_payload_QPSK = int2bit(double(back_payload_QPSK), 8);

data_BPSK = zeros(96, 1);
data_QPSK = zeros(96, 1);
% hest_check;
for i = 1:8
    idx = i:8:96;
    data_BPSK(idx) = back_payload_BPSK(i, :);
    data_QPSK(idx) = back_payload_QPSK(i, :);
end

save back_payload_1.mat data_QPSK data_QPSK

function TX_signal = generate_OFDM_signal(data1_slot_Frequency, data2_slot_Frequency, data3_slot_Frequency, data4_slot_Frequency)
    %% Parameter
    N_FFT = 64;
    %% Short_preamble
    S_k = sqrt(13/6)*[0,0,1+j,0,0,0,-1-j,0,0,0,1+j,0,0,0,-1-j,0,0,0,-1-j,0,0,0,1+j,0,0,0,0,0,0,0,-1-j,0,0,0,-1-j,0,0,0,1+j,0,0,0,1+j,0,0,0,1+j,0,0,0,1+j,0,0]; % [1x53]
    virtual_subcarrier = zeros(1,N_FFT-length(S_k)); % [1x11]
    Short_preamble_slot_Frequency = [virtual_subcarrier(1:6),S_k,virtual_subcarrier(7:11)]; % [1x64]
    Short_preamble_slot_Time = ifft(ifftshift(Short_preamble_slot_Frequency)); % [1x64]
    Short_preamble = repmat(Short_preamble_slot_Time(1:16),1,10); % [1x160]
    %% Long_preamble
    L_k = [1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,1,1,-1,-1,1,1,-1,1,-1,1,1,1,1,0,1,-1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,-1,1,1,-1,-1,1,-1,1,-1,1,1,1,1]; % [1x53]
    virtual_subcarrier = zeros(1,N_FFT-length(L_k)); % [1x11]
    Long_preamble_slot_Frequency = [virtual_subcarrier(1:6),L_k,virtual_subcarrier(7:11)]; % [1x64]
    Long_preamble_slot_Time = ifft(ifftshift(Long_preamble_slot_Frequency)); % [1x64]
    Long_preamble = [Long_preamble_slot_Time(33:64),Long_preamble_slot_Time,Long_preamble_slot_Time]; % [1x160]
    %% Data

    % data1_slot_Frequency = pskmod(data_Payload_1,M,pi/4);  % [1x48]
    % data2_slot_Frequency = pskmod(data_Payload_2,M,pi/4);  % [1x48]
    pilot = [1,1,1,-1]; % [1x4]
    virtual_subcarrier = zeros(1,11); % [1x11]
    data11_slot_Frequency = [virtual_subcarrier(1:6),data1_slot_Frequency(1:5),pilot(1),data1_slot_Frequency(6:18),pilot(2),data1_slot_Frequency(19:24),0,data1_slot_Frequency(25:30),pilot(3),data1_slot_Frequency(31:43),pilot(4),data1_slot_Frequency(44:48),virtual_subcarrier(7:11)]; % [1x64]
    data22_slot_Frequency = [virtual_subcarrier(1:6),data2_slot_Frequency(1:5),pilot(1),data2_slot_Frequency(6:18),pilot(2),data2_slot_Frequency(19:24),0,data2_slot_Frequency(25:30),pilot(3),data2_slot_Frequency(31:43),pilot(4),data2_slot_Frequency(44:48),virtual_subcarrier(7:11)]; % [1x64]
    data33_slot_Frequency = [virtual_subcarrier(1:6),data3_slot_Frequency(1:5),pilot(1),data3_slot_Frequency(6:18),pilot(2),data3_slot_Frequency(19:24),0,data3_slot_Frequency(25:30),pilot(3),data3_slot_Frequency(31:43),pilot(4),data3_slot_Frequency(44:48),virtual_subcarrier(7:11)]; % [1x64]
    % assemble frequency slot for data4 (48 data subcarriers + pilots & virtuals)
    data44_slot_Frequency = [virtual_subcarrier(1:6), ...
        data4_slot_Frequency(1:5), pilot(1), ...
        data4_slot_Frequency(6:18), pilot(2), ...
        data4_slot_Frequency(19:24), 0, ...
        data4_slot_Frequency(25:30), pilot(3), ...
        data4_slot_Frequency(31:43), pilot(4), ...
        data4_slot_Frequency(44:48), virtual_subcarrier(7:11)]; % [1x64]

    % time-domain conversion and transmit payload for slot 4 (with CP)
    data4_slot_Time = ifft(ifftshift(data44_slot_Frequency)); % [1x64]
    data4_TX_payload = [data4_slot_Time(49:64), data4_slot_Time]; % [1x80]

    data1_slot_Time = ifft(ifftshift(data11_slot_Frequency)); % [1x64]
    data2_slot_Time = ifft(ifftshift(data22_slot_Frequency)); % [1x64]
    data3_slot_Time = ifft(ifftshift(data33_slot_Frequency)); % [1x64]
    data1_TX_payload = [data1_slot_Time(49:64),data1_slot_Time]; % [1x80]
    data2_TX_payload = [data2_slot_Time(49:64),data2_slot_Time]; % [1x80]
    data3_TX_payload = [data3_slot_Time(49:64),data3_slot_Time];
    %% Frame Combination
    Frame = [Short_preamble,Long_preamble,data1_TX_payload,data2_TX_payload,data3_TX_payload,data4_TX_payload]; % [1x(160+160+80+80)]=[1x480]
    %% Oversampling
    OVR = 2;
    Frame_OVR_sampling = oversamp(Frame,length(Frame),OVR); % [1x960]
    %% Root Raised Cosine filter
    rolloff = 0.5;
    L = 6;
    RRC = rcosdesign(rolloff,L,OVR,'sqrt');
    %% TX
    TX_signal = conv(Frame_OVR_sampling,RRC); % [1x972]


end

% start with QPSK, then switch based on BER
t = (0:2999) / tx_object.BasebandSampleRate;
f1 = 250e3; % BPSK peak frequency
f2 = 450e3; % QPSK peak frequency
tx_sine_BPSK = exp(1j * 2* pi * f1 * t);
tx_sine_QPSK = exp(1j * 2* pi * f2 * t);

times = 100;
H_est_time_vector = zeros(times, 64);
counter = 0;

tx_zeros = complex(zeros(3000, 1));
transmitRepeat(tx_object, tx_zeros);
od = 1 : 2 : 192;
even = 2 : 2 : 192;
Ready_Time = 0;
scale = 1024;
% packets_detected = 0;
%% Main
state = 1; % status Start
Run_time_number = 1;
while(state == 1)
    try
    % transmitRepeat(tx_object, tx_zeros);
    [data_rx_raw, dataLength, lostSample] = step(rx_object);
    if Run_time_number > Ready_Time
        
        % ----- RX Raw -----%
        data_rx_scaled = double(data_rx_raw)./scale; % [3000x1]
        RX = data_rx_scaled.'; % [1x3000]
        % length(RX)
        
        % subplot(2,4,1),plot(RX,'.');title('RX-Raw');axis([-1.5 1.5 -1.5 1.5]);axis square;
        subplot(2,4,2),plot(real(RX));title('I');axis([1 3000 -1.5 1.5]);axis square;
        subplot(2,4,3),plot(imag(RX));title('Q');axis([1 3000 -1.5 1.5]);axis square;
        
        [Spectrum_waveform,Welch_Spectrum_frequency] = pwelch(RX,[],[],[],rx_object.BasebandSampleRate,'centered','power');
        subplot(2,4,4),plot(Welch_Spectrum_frequency,pow2db(Spectrum_waveform));
        title('Welch Power Spectral Density');axis([-rx_object.BasebandSampleRate/2 rx_object.BasebandSampleRate/2 -100 -10]);axis square;
        
        drawnow;
        
        % ----- Demodulation -----%
        % try
        release(tx_object);
        [M_n,Threshold_graph,H_est_time,RX_Payload_1_no_Equalizer,RX_Payload_2_no_Equalizer,RX_Payload_1_no_pilot,RX_Payload_2_no_pilot, RX_Payload_3_no_equalizer, RX_Payload_3_no_pilot, BER, H_est, packets_detected] = OFDM_rx2(RX,Parameters_struct);
        format = numerictype(1, 16, 11);

        hest_time_real = real(H_est_time);
        hest_time_imag = imag(H_est_time);
        
        hest_time_real = hest_time_real(1:6);
        hest_time_imag = hest_time_imag(1:6);
        % hest_time_real = hest_time_imag(1:6);
        % hest_time_imag = imag(H_est_time(1:6));

        hest_time_realq = fi(hest_time_real, format);
        hest_time_imagq = fi(hest_time_imag, format);

        disp(complex(hest_time_realq, hest_time_imagq));

        % disp('Estimated Channel Impulse Response (first 6 taps):');
        % disp([hest_time_realq.bin; hest_time_imagq.bin]);
        % break
        
        data_channel = reshape([hest_time_realq.bin'; hest_time_imagq.bin.'], 1, []);
        % disp(data_channel);
        data_channel = reshape(data_channel, 1, []);
        % data_channel = data_channel - '0';
        data_channel = data_channel(~isspace(data_channel)); % remove space chars
        data_channel = int8(data_channel - '0');
        % disp(data_channel);
        % break;
        % catch
            % packets_detected = 0;
        % end
        subplot(2,4,5),plot(1:length(M_n),M_n,1:length(M_n),Threshold_graph);title('Packet Detection');axis([1,length(M_n),0,1.2]);axis square;
        subplot(2,4,6),plot(abs(H_est_time));title('Channel Estimation');axis([1 64 0 7]);axis square;xlabel('Time');
        subplot(2,4,1),plot([RX_Payload_2_no_Equalizer,RX_Payload_1_no_Equalizer],'.');title('RX-Before Equalization');axis square;
        subplot(2,4,7),plot([RX_Payload_1_no_pilot,RX_Payload_2_no_pilot],'*');
        title('Constellation');axis([-1 1 -1 1]);axis square;
        
        subplot(2,4,8),plot(abs(H_est).^2);
        title({'Channel';['BER = ',num2str(BER)]});axis square;
        % catch
            % packets_detected = 0;
        % end
        Run_time_number = Run_time_number+1;
        
        if(packets_detected == 0)
            release(tx_object);
            data_modulated = pskmod(data_channel, 2).';
            size(data_modulated)
            tx_back_BPSK = generate_OFDM_signal(data_modulated(1:48).', data_modulated(49:96).', data_modulated(97:144).', data_modulated(145:192).');

            if (channel_sim)
                rayleighchan = comm.RayleighChannel(...
                'SampleRate', Fs, ...
                'PathDelays', [0 1e-6 1.7e-6 2.2e-6], ...  % 4-tap channel
                'AveragePathGains', [0 -14 -15 -16], ...        % dB (exponential decay)
                'MaximumDopplerShift', 100, ...               % 50 Hz ~ walking speed
                'NormalizePathGains', true, ...              % Preserve average power
                'PathGainsOutputPort', true);

                %% Apply Channel
                [TX_signal_faded, PathGains] = rayleighchan(tx_back_BPSK.');
                tx_back_BPSK = TX_signal_faded.';
            end

            transmitRepeat(tx_object, tx_back_BPSK.');
            % pause(1);
            % release(tx_object);
            % transmitRepeat(tx_object, tx_zeros);
        elseif(packets_detected >= 1)
            release(tx_object);
            % data_mod = pskmod(data_QPSK, 2);
            % tx_back_QPSK = generate_OFDM_signal(data_mod(1:48).', data_mod(49:96).');
            data_modulated = pskmod(data_channel, 2);
            size(data_modulated);
            tx_back = generate_OFDM_signal(data_modulated(1:48), data_modulated(49:96), data_modulated(97:144), data_modulated(145:192));
            if (channel_sim)
            rayleighchan = comm.RayleighChannel(...
                'SampleRate', Fs, ...
                'PathDelays', [0 1e-6 1.7e-6 2.2e-6], ...  % 4-tap channel
                'AveragePathGains', [0 -14 -15 -16], ...        % dB (exponential decay)
                'MaximumDopplerShift', 100, ...               % 50 Hz ~ walking speed
                'NormalizePathGains', true, ...              % Preserve average power
                'PathGainsOutputPort', true);

            %% Apply Channel
            [TX_signal_faded, PathGains] = rayleighchan(tx_back.');
            tx_back = TX_signal_faded.';
            end
            transmitRepeat(tx_object, tx_back.');
            pause(1);
            % release(tx_object);
            % transmitRepeat(tx_object, tx_zeros);
            % hest_check = abs(H_est_time);
            % break;
            % break;
            H_est_time_vector = circshift(H_est_time_vector,1);
            H_est_time_vector(1, :) = (H_est_time);
            counter = counter + 1;
            if (counter == 4000)
                % estimate_coherence_time(H_est_time_vector);
                break;
            end
        end
    end % Start
    
    if Run_time_number <= Ready_Time  % Ready
        % disp('Ready');
    end
    Run_time_number = Run_time_number+1;
    
    % ----- Button Behavior -----%
    set(button,'Callback','setstate0_RX'); % Set the reaction of pushing button
    
    catch
        ErrorMessage = lasterr;
        fprintf('Error Message : \n');
        disp(ErrorMessage);
    
        fprintf(2,'Error occurred & Stop Hardware\n');
        
        % ----- Error Handling -----%
        % release(rx_object);
        % state=0;

    end % Error control
end % While

estimate_coherence_time(H_est_time_vector(1:10, :), 560*1/tx_object.BasebandSampleRate); 

release(rx_object);
release(tx_object);
close all;
disp('Software Complete');
clear;close all;clc;j=1i;
Global_Parameters;
%% Hardware Parameters
Mode='transmitRepeat'; % Select Mode
tx_object = sdrtx('Pluto','Gain',-10, 'CenterFrequency', 0.915e9);
%          'EnableBurstMode',1,...

%% Button Setting
figure('Name','TX','NumberTitle','off');
TransmittingDisplay = uicontrol('Style', 'text', 'Position',[55,150,155,35],'String', 'Transmitting','FontSize',20,'HorizontalAlignment','left','BackgroundColor',[0.937 0.867 0.867]);
button = uicontrol; % Generate GUI button
set(button,'String','Stop !','Position',[80 50 100 60]); % Add "Stop !" text
set(gcf,'Units','centimeters','position',[3 3 7 6]); % Set the postion of GUI


load('data_Payload_1');
load('data_Payload_2');

function ser = convertToBPSK(data)
    arr = int2bit(data, 2);
    odd = 1:2:95;
    even = 2:2:96;
    ser(odd) = arr(1,:);
    ser(even) = arr(2,:);
end

function TX_signal = generate_OFDM_signal(data1_slot_Frequency, data2_slot_Frequency)
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
    data1_slot_Time = ifft(ifftshift(data11_slot_Frequency)); % [1x64]
    data2_slot_Time = ifft(ifftshift(data22_slot_Frequency)); % [1x64]
    data1_TX_payload = [data1_slot_Time(49:64),data1_slot_Time]; % [1x80]
    data2_TX_payload = [data2_slot_Time(49:64),data2_slot_Time]; % [1x80]
    %% Frame Combination
    Frame = [Short_preamble,Long_preamble,data1_TX_payload,data2_TX_payload]; % [1x(160+160+80+80)]=[1x480]
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
% transmitRepeat Mode
df1 = pskmod(data_Payload_1, 4, pi/4);
df2 = pskmod(data_Payload_2, 4, pi/4);
TX_signal = generate_OFDM_signal(df1, df2);

BasebandSampleRate = 10e6;  % Adjust based on your Global_Parameters
OVR = 2;
Fs = BasebandSampleRate * OVR;  % Actual sample rate

%% Create Rayleigh Fading Channel
% This is to simulate a multi-path channel to test whether equalisation
% works, and it does!!
channel_sim = 1;
if (channel_sim)
    rayleighchan = comm.RayleighChannel(...
        'SampleRate', Fs, ...
        'PathDelays', [0 0.9e-6 1.5e-6 5.2e-6], ...  % 4-tap channel
        'AveragePathGains', [0 -15 -10 -15], ...        % dB (exponential decay)
        'MaximumDopplerShift', 50, ...               % 50 Hz ~ walking speed
        'NormalizePathGains', true, ...              % Preserve average power
        'PathGainsOutputPort', true);

    %% Apply Channel
    [TX_signal_faded, PathGains] = rayleighchan(TX_signal.');
    TX_signal = TX_signal_faded.';
end
TX_Hardware = repmat(TX_signal.',5,1); % Transmit Data must be >= 4096 % [4860x1]

transmitRepeat(tx_object,TX_Hardware);

%% Button setting
figure('Name','RX','NumberTitle','off');
button = uicontrol; % Generate GUI button
set(button,'String','Stop !','Position',[1475 15 100 60]); % Add "Stop !" text
set(gcf,'Units','centimeters','position',[1 2 49 24]); % Set the postion of GUI
%% Hardware Parameters
rx_object = sdrrx('Pluto','Gain',40, 'CenterFrequency', 1e9, 'SamplesPerFrame', 3000);

Ready_Time = 0;
scale = 1024;
%% Main
state = 1; % status Start
Run_time_number = 1;
while(state == 1)
    try
    
    [data_rx_raw, dataLength, lostSample] = step(rx_object);
    if Run_time_number > Ready_Time
        
        % ----- RX Raw -----%
        data_rx_scaled = double(data_rx_raw)./scale; % [3000x1]
        RX = data_rx_scaled.'; % [1x3000]
        
        subplot(2,4,1),plot(RX,'.');title('RX-Raw');axis([-1.5 1.5 -1.5 1.5]);axis square;
        subplot(2,4,2),plot(real(RX));title('I');axis([1 3000 -1.5 1.5]);axis square;
        subplot(2,4,3),plot(imag(RX));title('Q');axis([1 3000 -1.5 1.5]);axis square;
        
        [Spectrum_waveform,Welch_Spectrum_frequency] = pwelch(RX,[],[],[],rx_object.BasebandSampleRate,'centered','power');
        subplot(2,4,4),plot(Welch_Spectrum_frequency,pow2db(Spectrum_waveform));
        % title('Welch Power Spectral Density');axis([-rx_object.BasebandSampleRate/2 rx_object.BasebandSampleRate/2 -100 -10]);axis square;
        title('Welch Power Spectral Density');axis([100e3 300e3 -100 -10]);axis square;
        drawnow;
        
        % Check for frequency of Peak
        [~, peakIndex] = max(Spectrum_waveform);
        peakFrequency = Welch_Spectrum_frequency(peakIndex);
        disp(peakFrequency);
        if (abs(peakFrequency - 250e3) < 10e3) 
            arr = convertToBPSK(data_Payload_1);
            df1 = pskmod(arr(1:48), 2);
            df2 = pskmod(arr(49:96), 2);
            tx_sig = generate_OFDM_signal(df1, df2);
            disp("Switching to BPSK")
            release(tx_object);
            transmitRepeat(tx_object, tx_sig.');
        elseif (abs(peakFrequency - 450e3) < 10e3)
            df1 = pskmod(data_Payload_1, 4, pi/4);
            df2 = pskmod(data_Payload_2, 4, pi/4);
            tx_sig = generate_OFDM_signal(df1, df2);
            disp("Switching to QPSK");
            transmitRepeat(tx_object, tx_sig.');
        else 
            continue;
        end 
        

        Run_time_number = Run_time_number+1;
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

release(rx_object);
close all;
disp('Software Complete');
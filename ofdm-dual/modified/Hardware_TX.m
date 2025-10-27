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
%% TX Load
load('TX_signal'); % [1x972]
% transmitRepeat Mode

BasebandSampleRate = 10e6;  % Adjust based on your Global_Parameters
OVR = 2;
Fs = BasebandSampleRate * OVR;  % Actual sample rate

%% Create Rayleigh Fading Channel
% This is to simulate a multi-path channel to test whether equalisation
% works, and it does!!
rayleighchan = comm.RayleighChannel(...
    'SampleRate', Fs, ...
    'PathDelays', [0 0.5e-6 1.5e-6 3.2e-6], ...  % 4-tap channel
    'AveragePathGains', [0 -15 -18 -10], ...        % dB (exponential decay)
    'MaximumDopplerShift', 50, ...               % 50 Hz ~ walking speed
    'NormalizePathGains', true, ...              % Preserve average power
    'PathGainsOutputPort', true);

%% Apply Channel
[TX_signal_faded, PathGains] = rayleighchan(TX_signal.');
TX_signal = TX_signal_faded.';

TX_Hardware = repmat(TX_signal.',5,1); % Transmit Data must be >= 4096 % [4860x1]
state = 1;
%% Main
switch Mode
    case 'step'
        while(state == 1)
           step(tx_object,TX_Hardware);
           % ----- Button Behavior -----%
           set(button,'Callback','setstate0_TX'); % Set the reaction of pushing button
           drawnow;
        end
        release(tx_object);

    case 'transmitRepeat'
        transmitRepeat(tx_object,TX_Hardware);
        % ----- Button Behavior -----%
        set(button,'Callback','setstate0_TX'); % Set the reaction of pushing button
end



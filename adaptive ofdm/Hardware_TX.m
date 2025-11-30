clear;close all;clc;j=1i;
Global_Parameters;
%% Hardware Parameters
Mode='transmitRepeat'; % Select Mode
tx_object = sdrtx('Pluto','Gain',-10, 'CenterFrequency', 1e9);
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
%% TX_with_Stop.m
% clear; close all; clc;
% j = 1i;
% Global_Parameters;
% 
% Mode = 'transmitRepeat';
% tx_object = sdrtx('Pluto','Gain',-10, 'CenterFrequency', 915e6);
% 
% % Button Setting
% hFigTX = figure('Name','TX','NumberTitle','off','Units','centimeters','Position',[3 3 7 6]);
% uicontrol('Style','text','Position',[55,150,155,35],'String','Transmitting','FontSize',12,'HorizontalAlignment','left','BackgroundColor',[0.937 0.867 0.867]);
% hStopTX = uicontrol('Style','pushbutton','String','Stop !','Position',[80 50 100 60],'Units','pixels');
% setappdata(hFigTX,'runFlag',1);
% set(hStopTX,'Callback',@(src,ev) setappdata(hFigTX,'runFlag',0));
% 
% % TX Load
% load('TX_signal'); % expected [1 x L]
% TX_Hardware = repmat(TX_signal.',5,1); % make it long enough
% state = 1;
% 
% switch Mode
%     case 'step'
%         while getappdata(hFigTX,'runFlag') && isvalid(hFigTX)
%             step(tx_object,TX_Hardware);
%             drawnow;
%         end
%         release(tx_object);
%     case 'transmitRepeat'
%         % Start a background transmission
%         transmitRepeat(tx_object,TX_Hardware);
%         % Wait until stop pushed
%         while getappdata(hFigTX,'runFlag') && isvalid(hFigTX)
%             pause(0.1);
%             drawnow;
%         end
%         % Stop transmission
%         stop(tx_object);
%         release(tx_object);
% end
% 
% if isvalid(hFigTX), close(hFigTX); end
% disp('TX software complete.');
%% Parameters
Parameters_struct.CenterFrequency = 5e9; % 5 GHz
Parameters_struct.Bandwidth = 20e6; % 20 MHz
Parameters_struct.Ts = 1/Parameters_struct.Bandwidth; % 50 ns
%% Load Given data
load('Long_preamble_slot_Frequency'); % [1x64]
load('data_Payload_1'); % [1x48]
load('data_Payload_2'); % [1x48]
Parameters_struct.Long_preamble_slot_Frequency = Long_preamble_slot_Frequency;
Parameters_struct.data_Payload_1 = data_Payload_1;
Parameters_struct.data_Payload_2 = data_Payload_2;

arr = int2bit(data_Payload_1, 2);
odd = 1:2:95;
even = 2:2:96;
ser(odd) = arr(1,:);
ser(even) = arr(2,:);

Parameters_struct.data_Payload_1_BPSK = ser(1:48);
Parameters_struct.data_Payload_2_BPSK = ser(49:96);
clc; clear;

%% Defined Values
p_max = 50; % W
Vg_min = 20; % in Volts
Vg_max = 30;
Vo = 5;
Io_max = p_max / Vo;
    Io_min = 1;

fs = 100000; % Hz
delta_vo = 1; %percent
delta_vo = Vo * (delta_vo/100);

%% Calculate n
n = Vo / ((Vg_min + Vg_max) / 2)

%% Calculate D range
D2 = 0.45
D1 = (1 - sqrt(1 - 4 * ( (Io_min * D2 * (1 - D2) ) / (Io_max) )))/2

%% Calculate L_leakage
L = (Vg_min * D2 * (1 - D2)) / (2 * Io_max * fs * n); %in H

%% Calculate Cout
Dp = 1 - D2;
a = (Vg_max * (1 - 2 * D2 * Dp) + Vo/n); %for better readability
q1 = (Dp/(8 * L * (fs^2) * n)) * a;
q2 = ((Dp^2) / (8 * L * (fs^2) * n)) * (Vg_max - Vo/n);
q3 = ((D2 * Dp) / (16 * L * (fs^2) * n)) * ((a * Vg_max) / (Vg_max + Vo/n));
delta_Q = q1 + q2 + q3;

Cout = delta_Q/(2 * delta_vo) * 1000000 % in uF

Final_L = L * 1000 %in mH
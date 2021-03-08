f_Hz = 50;              % Frequency in Hz
f_rad = f_Hz*2*pi;      % Frequency in rad/s

V_g = 230;              % Grid Amplitude


var_noise = 0;
t_frq_step = 0.2;       % [s] Previously: 0.2
t_phase_step = 0.3;     % [s] Previously: 0.3
A_frq_step = 1.1;       % %-size of frequency step
A_phase_step = 20*pi/180;  % [rad]

% abEPMAFPLL:
%--------------

fs = 10000;
Ts = 1/fs;

% MAF
T_MAF = 0.02;
N = T_MAF/Ts;

T_st = 0.02*2; % "Fast" response time according to ali2018a p. 133
zeta = 0.7;

k_phi = T_MAF-Ts;
ki = (4.6/(zeta*T_st))^2;
kp = 2*zeta*sqrt(ki)+ki*k_phi;

% Temporary!!
%--------------
% a = 1;
% b = 1/N*ones(N,1);
% figure
% freqz(b,a,10000)
% 
% z = tf('z', Ts)
% H_MAF = 1/N * (1-z^-N)/(1-z^-1)
% 
% figure
% bode(H_MAF)
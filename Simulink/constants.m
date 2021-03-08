f_Hz = 50;              % Frequency in Hz
f_rad = f_Hz*2*pi;      % Frequency in rad/s

V_g = 230;              % Grid Amplitude


var_noise = 2;
t_frq_step = 0.2;       % [s]
t_phase_step = 0.3;     % [s]
A_frq_step = 1.0;       % %-size of frequency step
A_phase_step = 0*pi/180;  % [rad]

% abEPMAFPLL:
%--------------

fs = 10000;
Ts = 1/fs;

% MAF
T_MAF = 0.02;


T_st = 0.02*2; % "Fast" response time according to ali2018a p. 133
zeta = 0.7;

k_phi = T_MAF-Ts;
ki = (4.6/(zeta*T_st))^2;
kp = 2*zeta*sqrt(ki)+ki*k_phi;


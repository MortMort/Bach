clc;clear;close all

fs = 200;
Ts = 1/fs;
N = 10;

% z and transfer function:
% z = exp(j*omega*Ts)
z = tf('z',Ts)
F_maf = 1/N * (1-z^-N)/(1-z^-1)


% Angle and length from transfer function:
% angle(F_maf)
% abs(F_maf)

% Angle and length directly
% F_maf_abs = abs((sin(omega*N*Ts/2))/(N*sin(omega*Ts/2)))
% F_maf_angle = -(omega*(N-1)*Ts)/2


figure
bode(F_maf)
hold on
bode(F_maf_2)
legend('F_{maf}','F_{maf_2}')



%% 

fs = 10000;
Ts = 1/fs;

T_MAF = 0.02;
N_bins = round(T_MAF/Ts)
% N_bins = 20

z = tf('z',Ts)
N = N_bins
F_maf = 1/N * (1-z^-N)/(1-z^-1)

figure
bode(F_maf)
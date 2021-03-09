clc; clear; close all;


fs = 10000;
Ts = 1/fs;
T_MAF = 0.02;
N = T_MAF/Ts;

% transfer function based on: short definition:
z = tf('z', Ts)
H_MAF = 1/N * (1-z^-N)/(1-z^-1)

figure
bode(H_MAF)

% transfer function based on: filter koefficients
a = 1;
b = 1/N*ones(N,1);
figure
freqz(b,a,10000)

% fs = 1000;
T = 50;
t = 1/fs*(0:(fs*T)-1);
x = chirp(t,0,T,100);


y = filter(b,a,x);
figure
plot(y)
clear;close all;clc;

n = 0:0.01:5*2*pi;
% n = 1;
q_pll = mod(n,2*pi);

% syms q_pll
% dq_t = sqrt(2/3)*[cos(q_pll) cos(q_pll-2*pi/3) cos(q_pll+2*pi/3); 
%     -sin(q_pll) -sin(q_pll-2*pi/3) sin(q_pll+2*pi/3)]

Vabc = 230*[sin(q_pll); sin(q_pll+2*pi/3); sin(q_pll-2*pi/3)];

Vqd = sqrt(2/3)*[cos(q_pll) cos(q_pll-2*pi/3) cos(q_pll+2*pi/3); 
     -sin(q_pll) -sin(q_pll-2*pi/3) -sin(q_pll+2*pi/3)] * Vabc;


figure
plot(Vabc')

% figure
% plot(dq_t')

figure
plot(Vqd')
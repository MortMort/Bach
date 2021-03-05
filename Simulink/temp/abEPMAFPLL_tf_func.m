clc; clear; close all

syms z1 e ki kq s q_pll q_g kp
% eq = z1 == e + z1*ki/s*kq

% solve(eq,z1)

eq2 = q_pll == ((q_g-q_pll)*s)/s-ki*kq*(kp+ki/s)*1/s

solve(eq2,(q_ge))
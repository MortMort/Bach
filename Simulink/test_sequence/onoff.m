function y = onoff(t,t_on,t_off,dim)

x1 = on(t,t_on,dim);
x2 = off(t,t_off,dim);

y = x1 .* x2;
end
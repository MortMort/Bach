function y = off(t,t_off,dim)

x = 1 - heaviside(t-t_off);
y = zeros(length(t),dim);
for i=1:dim
    y(:,i) = x;
end
end
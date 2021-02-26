function y = on(t,t_on,dim)

x = heaviside(t-t_on);
y = zeros(length(t),dim);
for i=1:dim
    y(:,i) = x;
end
end
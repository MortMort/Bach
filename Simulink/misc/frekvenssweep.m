% Generer Chirp
fs = 2000;         % Samplefrekvens
t = 1/fs;           % Sampletid
T = 3
N = round(fs*T);           % Antal samples
n_t = (0:N-1)*t;    % sampleindex i tid
f_max = 53
f_min = 47

y = chirp(n_t,f_min,max(n_t),f_max);
figure
plot(n_t,y)

n_f = (0:(N-1))*fs/N
Y = fft(y)
figure
plot(n_f,abs(Y))
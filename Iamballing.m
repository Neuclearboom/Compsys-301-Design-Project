fs = 20000; % sampling frequency
f0 = 50; % fundamental frequency
T = 1/ f0 ; % period
t = 0:1/ fs :2* T ; % two periods
N = 1:2:15; % odd harmonics
x = zeros ( size ( t ));
for n = N
x = x + sin (2* pi* n * f0 * t )/ n ;
end
x = x * 4/ pi;
plot (t , x )
title ('Fourier ␣ Series ␣ Partial ␣Sum ␣( Square ␣ Wave )')
xlabel ('Time ␣[s]')
ylabel ('Amplitude ')

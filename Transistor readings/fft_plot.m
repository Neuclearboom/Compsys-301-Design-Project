% FFT of time-voltage data from a CSV file
% First column = time, second column = voltage

clear; clc; close all;

fileName = '13l1d1.csv';

% Read the CSV, skipping the two header rows
M = csvread(fileName, 2, 0);

time = M(:, 1);
voltage = M(:, 2);

% Check that data was read correctly
if isempty(time) || isempty(voltage)
    error('No data found in the CSV file.');
end

% Calculate sampling period from the time vector
if numel(time) > 1
    dt = median(diff(time));
else
    error('Not enough samples for FFT analysis.');
end

if dt <= 0
    error('Time values must be increasing.');
end

Fs = 1 / dt;              % sampling frequency
N = length(voltage);
Y = fft(voltage);

% Single-sided amplitude spectrum
P2 = abs(Y / N);
P1 = P2(1:floor(N/2)+1);
P1(2:end-1) = 2 * P1(2:end-1);

f = (0:floor(N/2)) * Fs / N;

% Plot the original signal
figure;
subplot(2,1,1);
plot(time, voltage, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Voltage (V)');
grid on;
title('Time-domain Signal');

% Plot the FFT magnitude spectrum
subplot(2,1,2);
plot(f, P1, 'r', 'LineWidth', 1.5);
xlabel('Frequency (Hz)');
ylabel('|V(f)|');
grid on;
title('FFT Magnitude Spectrum');

% Show dominant frequency
[~, idx] = max(P1(2:end));
dominantFreq = f(idx + 1);
fprintf('Dominant frequency = %.6f Hz\n', dominantFreq);

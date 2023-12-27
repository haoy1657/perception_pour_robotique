function [sine_wave, white_noise, linear_sweep, t] = generate_signals(T, A, f0, f1, mu, sigma, fs)
    % Time vector
    t = 0:1/fs:T-1/fs;
    
    % Sine wave
    sine_wave = A * sin(2 * pi * f0 * t);
    
    % Gaussian white noise
    white_noise = mu + sigma * randn(size(t));
    
    % Linear sweep (chirp signal)
    linear_sweep = A * chirp(t, f0, T, f1);
end

function plot_signals(t, sine_wave, white_noise, linear_sweep)
    % Plot sine wave
    subplot(3, 1, 1);
    plot(t, sine_wave);
    title('Sine Wave');
    
    % Plot white noise
    subplot(3, 1, 2);
    plot(t, white_noise);
    title('White Noise');
    
    % Plot linear sweep
    subplot(3, 1, 3);
    plot(t, linear_sweep);
    title('Linear Sweep');
end

% Example usage:
T = 1; % Duration of 1 second
A = 1; % Amplitude of 1
f0 = 5; % Start frequency of 5 Hz
f1 = 20; % Stop frequency of 20 Hz
mu = 0; % Mean of 0 for white noise
sigma = 0.1; % Standard deviation of 0.1 for white noise
fs = 44100; % Sampling frequency of 44.1 kHz

[sine_wave, white_noise, linear_sweep, t] = generate_signals(T, A, f0, f1, mu, sigma, fs);
plot_signals(t, sine_wave, white_noise, linear_sweep);

% To play the signals, you can use the sound function in MATLAB:
sound(sine_wave, fs);
pause(T);
sound(white_noise, fs);
pause(T);
sound(linear_sweep, fs);
% Sine wave
sine_wave = A * sin(2 * pi * f0 * t);

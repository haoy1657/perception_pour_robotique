function [track_fil] = filter_low_pass(track,L,fcut)

track=double(track);
%  30 FPS
Fs=30;
x = track(1,:);
y = track(2,:);
z=track(3,:); 

x_resampled = x(1:L:end);
y_resampled = y(1:L:end);
z_resampled = z(1:L:end);
Fcut = fcut;
Fs_new = Fs / L;
x_filtered = lowpass(x_resampled, Fcut, Fs_new);
y_filtered = lowpass(y_resampled, Fcut, Fs_new);
z_filtered = lowpass(z_resampled, Fcut, Fs_new);
track_fil=[x_filtered;y_filtered;z_filtered];


end
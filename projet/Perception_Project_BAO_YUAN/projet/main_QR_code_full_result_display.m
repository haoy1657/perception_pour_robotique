% Because the complete display of the camera's trajectory requires 
% the processing of all the frames, i.e. 460 frames, 
% which can take a lot of time, I saved the runtime data in advance, 
% and here we directly show the camera's trajectory, 
% which is filtered by applying a low-pass filter,
% to get a trajectory as smooth as possible and similar to the real situation.
% % % % % % % % % 
% The full video is in a folder where you can view and compare it to the track.
% We simulate a circular trajectory
close all
clear all
clc;
load('data_QR_code_615frames.mat')
track=position_camera;
% R_camera=R_camera;
% T_camer=;
track=double(track.track);

track=track(:,50:400);
error=Error_reprojection.Error_reprojection;
error=double(error);


%[track_fil] = filter_low_pass(track,L,fcut)
% A low-pass filter is applied to smooth the trajectory, 
% L is the downsampling multiplier,
% and Fcut defines the cutoff frequency of the low-pass filter, 
% which filters the noise after the cutoff frequency


track_fil=filter_low_pass(track,20,0.1);

figure
plot3_position_camera(track,'track without filtered');
hold on;
plot3_position_camera(track_fil,'track filtered');
figure
plot_error(error);
figure
plot3_position_camera(track,'track without filtered');
figure
plot3_position_camera(track_fil,'track filtered');


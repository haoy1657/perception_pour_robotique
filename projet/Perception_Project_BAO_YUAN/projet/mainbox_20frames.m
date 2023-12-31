% 
close all
clear all
clc;

% load the parameter of camera
load('calibrationSession.mat');
CameraParameters = calibrationSession.CameraParameters;
K=CameraParameters.K;%% data loading
disp('1ere partie: chargement des donnees.');
% videos->frames
V1=Video2Frames('box.mp4',20);
disp('finished');
%% frames displaying
disp('2eme partie: affichage des videos.');
N1=size(V1,3);
% An empty list is created here, which will later be filled with the R T matrix of the camera's motion process
R_camera=[];
T_camera=[];
Error_reprojetion=[];

% We process the first frame of the image to locate the initial position of the camera,
% we have manually selected six points, the pixel coordinates of the six points 
% and the coordinates under the world coordinate system are already known
% box
Pixel_coord=[[285;312;1] [521;337;1] [254;477;1] [518;511;1] [393;403;1] [261;643;1] [485;667;1] [379;588;1]];
World_coord=[[0;153;153;1] [153;153;153;1] [0;0;153;1] [153;0;153;1] [71.5;71.5;153;1] [0;0;0;1] [153;0;0;1] [71.5;0;0;1]];
% Initial position of the camera relative to the world coordinate system
[R_initial,T_initail]=initial_camera_position(Pixel_coord,World_coord,K);
R_camera=[R_camera R_initial];
T_camera=[T_camera T_initail];
% We then use the epipolar geometry to estimate the next motion of the camera
% First we use the surf algorithm to quickly match the feature points of two adjacent frames

% Detect feature points between two images and return the Homogeneous coordinates of the pixels
% feature_point=feature_extraction(V1(:, :, 1),V1(:, :, 2));

figure(1)
for inc=1:(N1-1)
    % imagesc(V1(:,:,inc)),colormap('gray');hold on;
    % title(['checker img=',num2str(inc),'/',num2str(N1-1)])
    % hold off;
    % pause(0.1)
    clf;
    % pause(0.1);
    feature_point=feature_extraction(V1(:, :, inc),V1(:, :, inc+1),inc);
    [R_epipolar,T_epipolar,Error_epipolar]=epipolar_geometry(feature_point,K);
    [R_H,T_H,Error_H] = homography(feature_point,K);
    if Error_H >=Error_epipolar
        R=R_epipolar;
        T=T_epipolar;
        Error=Error_epipolar;
    else
        R=R_H;
        T=T_H;
        Error=Error_H;
        
    end
    disp('Actual Reprojection Error')
    disp(Error);
    R_camera=[R_camera R];
    T_camera=[T_camera T];
    Error_reprojection(inc)=Error;


end
track=camera_track(R_camera,T_camera);
track_fil=filter_low_pass(track,1,0.5);
figure
plot3_position_camera(track,'track without filter');
hold on;
plot3_position_camera(track_fil,'track filtered');


figure;
plot_error(Error_reprojection);

















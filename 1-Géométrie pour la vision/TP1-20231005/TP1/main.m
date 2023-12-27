close all
clear all
clc

%% data loading
disp('1ere partie: chargement des donnees.');
% videos->frames
V1=Video2Frames('checker.mp4');
V2=Video2Frames('QR.mp4');
% intrinsic matrix
load('K.mat', 'K')

%% frames displaying
disp('2eme partie: affichage des videos.');
N1=size(V1,3);
figure(1)
for inc=1:N1,
    imagesc(V1(:,:,inc)),colormap('gray');hold on;
    title(['checker img=',num2str(inc),'/',num2str(N1)])
    hold off;
    pause(0.1)
end
N2=size(V2,3);
figure(2)
for inc=1:N2,
    imagesc(V2(:,:,inc)),colormap('gray');hold on;
    title(['QR img=',num2str(inc),'/',num2str(N2)])
    hold off;
    pause(0.1)
end

%% interest point extraction
disp('3eme partie: extraction manuelle des points d''interets.');
disp('             ** ne pas hesiter a agrandir la fen?tre **');
Npts=6; % number of interest pts for example
u_m=[]; % list of int. pt coordinates (in image)
v_m=[]; % 
figure(3)
for inc=1:N1,
    imagesc(V1(:,:,inc)),colormap('gray');hold on;
    title(['checker img=',num2str(inc),'/',num2str(N1)])
    % display the pts in previous frame (if exist), to avoid errors
    if inc>1,
        for inc_number=1:Npts,text(u_curr,v_curr,num2str(inc_number),'FontSize',8),end;
    end
    % pts in current frame
    [u_curr,v_curr]=ginput(Npts);
    % -> save in list
    u_m=[u_m,u_curr];
    v_m=[v_m,v_curr];
    hold off;
    pause(0.1)
end

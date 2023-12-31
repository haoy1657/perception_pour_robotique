function [track] = camera_track(R_camera,T_camera)

% initialisation
T=[R_camera(:,1:3) T_camera(:,1); 0 0 0 1];
track=[T(1:3,4)];
for i=2:size(T_camera,2)
    % T_im1=[R_camera(:,3*(i-2)+1:3*(i-2)+3) T_camera(:,i-1)];
    T_i=[R_camera(:,3*(i-1)+1:3*(i-1)+3) T_camera(:,i); 0 0 0 1];
    T=T*T_i;
    track=[track T(1:3,4)];

end


end

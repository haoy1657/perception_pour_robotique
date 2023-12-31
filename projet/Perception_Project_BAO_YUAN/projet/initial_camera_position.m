function [R_initial,T_initial]=initial_camera_position(pixel_coord,world_coord,K)
% This function deals with the first frame of the video, 
% used to allow the camera to locate its initial position, 
% because after the epipolar geometry we only know the relative motion of the camera,
% in order to draw the trajectory of the camera relative to the box, 
% we need to know its initial position, the method used here is the same as the tp1,
% except that the points dealt with here are three-dimensional points rather than two-dimensional points

% By processing the first frame a little bit, we locate the 3D coordinate positions of the six points,
% and we also know the pixel coordinates of the six points, 
% so that we can calculate the initial pose of the camera with respect to the world coordinate system
p_c=pixel_coord;
w_c=world_coord;

for i=1:size(p_c,2)
    p_c(:,i)=inv(K)*p_c(:,i);
end


M=[];
for j=1:size(p_c,2)
    Mi=[-w_c(1,i) -w_c(2,i) -w_c(3,i) -1 0 0 0 0 p_c(1,j)*w_c(1,i) p_c(1,j)*w_c(2,i) p_c(1,j)*w_c(3,i) p_c(1,j)
        0 0 0 0  -w_c(1,i) -w_c(2,i) -w_c(3,i) -1  p_c(2,j)*w_c(1,i) p_c(2,j)*w_c(2,i) p_c(2,j)*w_c(3,i) p_c(2,j)];
    M=[M;Mi];
end
% svd decomposition
[U,S,V] = svd(vpa(M));
% Extract all singular values
s_v=diag(S);
% List all non-zero singular values and indicate the index of the smallest non-zero singular value
indices = find(s_v >0.05);
if ~isempty(indices)
    max_index = max(indices);
else
    fprintf('There exists no minimal singular vector that satisfies the requirement\n');
end
% Make the H matrix H = [R T]
L = V(:,max_index);
H=reshape(L,4,3)';
% disp(H);
[Q,R]=qr(H);

if det(Q)>0;
    R_initial=Q;
else
    R_initial=[Q(:,1:2) -Q(:,3)];
end

T_initial=R_initial'*H(:,4);

end

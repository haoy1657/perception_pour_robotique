function [R_epipolar,T_epipolar,Error_epipolar] = epipolar_geometry(feature_point,K)

f_p=feature_point;
F_p=feature_point;

% rotation matrix
% The two rotation matrices given 
% here are ninety and negative ninety degrees around
% the z-axis respectively
rotz_90p=[0  -1   0;   
            1   0   0;      
            0   0   1];
rotz_90n=[0   1   0;   
            -1  0   0;      
            0   0   1];
%Convert the feature point's pixel homogeneous coordinates to normalized coordinates in the camera coordinate system
for i=1:size(f_p,2)
    f_p(1:3,i)=inv(K)*f_p(1:3,i);
    f_p(4:6,i)=inv(K)*f_p(4:6,i);
end
% essential matrix
E=[];
for i=1:size(f_p,2)
    Ei=[f_p(1,i)*f_p(4,i) f_p(1,i)*f_p(5,i) f_p(1,i) f_p(2,i)*f_p(4,i) f_p(2,i)*f_p(5,i) f_p(2,i) f_p(4,i) f_p(5,i) 1];
    E=[E;Ei];
end
% svd decomposition
[U,S,V] = svd(vpa(E));
% Extract all singular values
% s_v=diag(S);
% List all non-zero singular values and indicate the index of the smallest non-zero singular value
% indices = find(s_v >0.005);
% if ~isempty(indices)
%     max_index = max(indices);
% else
%     fprintf('There exists no minimal singular vector that satisfies the requirement\n');
% end
%essential matrix E
% L = V(:,max_index);
% L=V(:,end);
% E=reshape(L,3,3)';
L=V(:,size(S,2));
E=reshape(L,3,3)';

% Here we get the essential matrix E, 
% and then we decompose his singular values, 
% and then we are able to derive four different bit positions of the camera that may exista
[U,S,V] = svd(vpa(E));
% E, solved according to a linear equation, may not satisfy an intrinsic property of 
% E - its singular values may not necessarily be of the form σ1, σ2, 0.
% We can set the singular values of the essential matrices purposely as（(σ1+σ2)/2,(σ1+σ2)/2,0）
s_v=diag(S);
s_v=[(s_v(1)+s_v(2))/2,(s_v(1)+s_v(2))/2,0];
E=U*diag(s_v)*V';
[U,S,V] = svd(vpa(E));   
% 8 possible solution [[R1 t1] [R1 t2] [R2 t1] [R2 t2] [R1 -t1] [R1 -t2] [R2 -t1] [R2 -t2]];

T1=U*rotz_90p*S*U';
t1=[T1(3,2);T1(1,3);T1(2,1)];
R1=U*rotz_90p'*V';
if det(R1)<=0
    R1(:,1:3)=-R1(:,1:3);
else
    R1(:,1:3)=R1(:,1:3);
end
T2=U*rotz_90n*S*U'; 
t2=[T2(3,2);T2(1,3);T2(2,1)];
R2=U*rotz_90n'*V';
if det(R1)<=0
    R2(:,1:3)=-R2(:,1:3);
else
    R2(:,1:3)=R2(:,1:3);
end

P=[[R1 t1] [R1 t2] [R2 t1] [R2 t2] [R1 -t1] [R1 -t2] [R2 -t1] [R2 -t2]];
% verification

% Triangulation to calculate the depth of 3D coordinate points 
% to filter out reasonable camera movements

P1 = K * [eye(3), zeros(3, 1)];
points3D = zeros(4, 3);
positive_depths = zeros(4, 1);
P2_options = {
    double(K * [R1, t1]);
    double(K * [R1, t2]);
    double(K * [R2, t1]);
    double(K * [R2, t2]);
    double(K * [R1, -t1]);
    double(K * [R1, -t2]);
    double(K * [R2, -t1]);
    double(K * [R2, -t2]);
};

positive_depth_counts=zeros(8,1);
% Here we check the depth of all feature points for all four solutions,
% and we choose the solution with the highest number of feature points with depth greater than zero.
for i = 1:8
    P2 = P2_options{i};
    % Detect the depth of all feature points size(F_p,2)
    for j=1:size(f_p,2)
        % triangularisation
        points3D = triangulate(F_p(1:2,j)',F_p(4:5,j)', P1, P2);
        % disp(points3D);
        % Check that the depth is positive
        if points3D(3)>=0
            %If the depth of the feature point is greater than zero
            % we add one point to the corresponding pose, and finally we obtain the pose with the most points. 
            positive_depth_counts(i) = positive_depth_counts(i)+1;
        end
    end

end
% Select the combination with the largest of the first two depth counters
P_positive_depth=[];

[sortedValues, sortedIndices] = sort(positive_depth_counts, 'descend');
% [max_value, max_index] = max(positive_depth_counts);
for i=1:2
    P_positive_depth=[P_positive_depth  P(:,(4*(sortedIndices(i)-1)+1):(4*(sortedIndices(i)-1)+3)) P(:,4*(sortedIndices(i)-1)+4)];
end
% R=P(:,(4*(max_index-1)+1):(4*(max_index-1)+3));
% T=P(:,4*(max_index-1)+4);

% Calculate the reprojection error among these two combinations to 
% select the combination with the smallest impulse projection error
Error = [];
P1_positive_depth = K * [eye(3), zeros(3, 1)];
P2_positive_depth_options={double(K*P_positive_depth(:,1:4))
    double(K*P_positive_depth(:,5:8))};
P_positive_depth_options={[P_positive_depth(:,1:4)]
    P_positive_depth(:,5:8)};


for i = 1:2
    P2_positive_depth=P2_positive_depth_options{i};
    total_error = 0;
    % Eight feature points
    for j=1:size(f_p,2)
        p1 = [F_p(1:2, j); 1];  % Convert to homogeneous  coordinates
        p2 = [F_p(4:5, j); 1];

        % Transform p1 to normalized camera 
        p1_normalized = inv(K) * p1;

        % Apply the transformation of the second camera.
        % p2_transformed_homogeneous = K * (P2_options{i} * [p1_normalized; 1]);
        p2_transformed_homogeneous =P2_positive_depth * [p1_normalized; 1];

        % Convert the projected point back to homogeneous coordinates
        p2_transformed = p2_transformed_homogeneous(1:2) / p2_transformed_homogeneous(3);

        % Calculate and totalise the error
        error = norm(p2_transformed - p2(1:2));
        total_error = total_error + error;

    end
    % disp(total_error);
    mean_error = total_error / (2*size(F_p, 2));
    Error(i)=mean_error;
    Error=double(Error);

end
[min_value, min_index] = min(Error);
Error_epipolar=min_value;
R_epipolar=P_positive_depth_options{min_index}(:,1:3);
T_epipolar=P_positive_depth_options{min_index}(:,4);

   
end
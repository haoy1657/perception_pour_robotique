function [R_H,T_H,Error_H] = homography(feature_point,K)

f_p=feature_point;
F_p=feature_point;
%Convert the feature point's pixel homogeneous coordinates to normalized coordinates in the camera coordinate system
for i=1:size(f_p,2)
    f_p(1:3,i)=inv(K)*f_p(1:3,i);
    f_p(4:6,i)=inv(K)*f_p(4:6,i);
end
H=[];
b=[];
% build matrix
for i=1:size(f_p,2)
    Hi=[f_p(1,i) f_p(2,i) 1 0 0 0 -f_p(1,i)*f_p(4,i) -f_p(2,i)*f_p(4,i)
        0 0 0 f_p(1,i) f_p(2,i) 1 -f_p(1,i)*f_p(5,i) -f_p(2,i)*f_p(5,i)];
    b(i)=f_p(4,i);
    b(2*i)=f_p(5,i);
    H=[H;Hi];
end
b=b';

[U,S,V] = svd(vpa(H));
% Take the rows and columns where the non-zero singularities are located.
S=S(1:size(S,2),1:size(S,2));
% least square solution
h= V*inv(S)*U(:,1:size(S,2))'*b;
h(size(h,1)+1)=1;
% matrix H
H=reshape(h,3,3)';
% svd of H
[U,S,V] = svd(vpa(H));
% Constructing Candidate Solutions.
R1=U*[1 0 0;0 1 0 ;0 0 det(U*V')]*V';
R2=U*[1 0 0;0 1 0 ;0 0 -det(U*V')]*V';
t1=U(:,end);
t2=-U(:,end);

% The eigenvalue of the rotation matrix is 1 
% if it is negative one we change the sign of the third column
if det(R1)<=0
    R1(:,1:3)=-R1(:,1:3);
else
    R1(:,1:3)=R1(:,1:3);
end

if det(R1)<=0
    R2(:,1:3)=-R2(:,1:3);
else
    R2(:,1:3)=R2(:,1:3);
end

% Filtering the correct solution
% Depth estimation, screening combinations with positive depth
P1 = K * [eye(3), zeros(3, 1)];
points3D = zeros(4, 3);
positive_depths = zeros(4, 1);
% All possible combinations
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
P=[[R1 t1] [R1 t2] [R2 t1] [R2 t2] [R1 -t1] [R1 -t2] [R2 -t1] [R2 -t2]];
% Initialising the counter
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

% Calculate the reprojection error and choose the one with the smallest error from the two remaining choices
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
Error_H=min_value;
R_H=P_positive_depth_options{min_index}(:,1:3);
T_H=P_positive_depth_options{min_index}(:,4);


% end
%收集已知的三维点（世界坐标）和它们在图像上的对应点（图像坐标）。

% 将这些点对表示为矩阵形式，通常使用齐次坐标（homogeneous coordinates），其中每个点对在矩阵中表示为一个4x1的列向量。
% 
% 构建一个大矩阵A，其中每个点对会产生两行数据，形成一个2n x 12的矩阵，其中n是点对的数量。
% 
% 使用奇异值分解（Singular Value Decomposition，SVD）来分解矩阵A，得到最小二乘解。
% 
% 最小二乘解通常是一个12x1的列向量，可以将其重塑为3x4的投影矩阵P。
% 
% 所以，DLT算法中涉及到的矩阵主要是矩阵A，以及通过SVD分解得到的最小二乘解，最终得到的投影矩阵P
m_p=[]; 

cor_monde=[0 15 0
    0 10 0
    0 5 0
    0 0 0
    35 15 0 
    35 10 0
    35 5 0 
    35 0 0];
[num_rows, num_cols] = size(u_m);
for i=1:num_cols
    A=[];
    for j=1:num_rows
        Ai=[cor_monde(j,1) cor_monde(j,2) cor_monde(j,3) 1 0 0 0 0 -u_m(j,i)*cor_monde(j,1) -u_m(j,i)*cor_monde(j,2) -u_m(j,i)*cor_monde(j,3) -u_m(j,i) 
            0 0 0 0 cor_monde(j,1) cor_monde(j,2) cor_monde(j,3) 1 -v_m(j,i)*cor_monde(j,1) -v_m(j,i)*cor_monde(j,2) -v_m(j,i)*cor_monde(j,3) -v_m(j,i)];
        A=[A;Ai];
        % disp(Ai);
    end
    [U,S,V] = svd(vpa(A));
    % [U,S,V]=[vpa(U),vpa(S),vpa(V)];
    L = V(:,end);
    disp(L);
        
    

end


% Points=[cor_monde(:,1) , cor_monde(:,2),cor_monde(:,3) u_m(:,1), v_m(:,1)];
% C = zeros(16,12);
% for i = 1:8
%     X = Points(i,1);Y = Points(i,2);Z = Points(i,3);x = Points(i,4);y = Points(i,5);
%     C(i*2-1,:) = [X,Y,Z,1,0,0,0,0,-x*X,-x*Y,-x*Z,-x];
%     C(i*2,:)   = [0,0,0,0,X,Y,Z,1,-y*X,-y*Y,-y*Z,-y];   
% end
% 
% 
% [U,S,V] = svd(C);
% % 最小的奇异值对应的列向量
% L = V(:,end);
% P = reshape(L,[4,3])';
% P = P/P(3,3);
% disp(P);
% 
% 
% K_invH = inv(K)*P ; 
% disp("K_inv*H") 
% display(K_invH) 
% R_1_estimate = K_invH(:,1) ; % direction de Xc
% R_2_estimate = K_invH(:,2) ; % Direction de Yc 
% R_3_estimate =  cross(R_1_estimate , R_2_estimate) ;
% 
% R_estimate = [R_1_estimate , R_2_estimate , R_3_estimate] ; 
% deter = det(R_estimate) ;
% a = nthroot(deter, 4) ; 
% disp("le facteur d'echelle a vaut : " ) ; disp(a) ; 
% 
% % Calcul de la vraie matrice de rotation 
% 
% R_1 = R_1_estimate/a ; 
% R_2 = R_2_estimate/a ; 
% R_3 = R_3_estimate/(a^2) ; 
% 
% disp("La matrice de rotation : ") 
% R = [R_1 , R_2 , R_3] ; 
% display(R) 
% 
% % Vecteur de translation (pose de la camera dans le WCF, lié à la dernière image)
% disp("Le vecteur de translation : ") 
% T_estimate = K_invH(:,3) ;
% T = T_estimate/a ; 
% disp(T) ;
% %如何验证所估算的旋转矩阵和位移矩阵是否符合实际，我们把世界坐标系的点做固定的旋转和位移然后图像上的成像点和图片做对比



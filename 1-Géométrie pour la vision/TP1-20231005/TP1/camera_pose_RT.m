function [RT] = camera_pose_RT(U,V,WCF,K)

%% Trajectoire de la camera 

%% Les 10 premières images correspondent aux 0.17 premières secondes 
%%

% On cherche à reconstuire la pose de la camèra (du moins son repère) en 3D
% dans le WCF au cours des 10 prises
% Il faut pour cela enregister la matrice [R|T] pour les 10 images et
% tracer l'évolution du repère de la caméra. 

% Exemple de tracé d'un repère orthonormé en 3D
% x0= [1 0 0] ; y0 = [0 1 0] ; z0 = cross(x0,y0) ; 

% point1 = x0;
% point2 = y0;
% point3 = z0;
% origin = [0,0,0];
% figure(1);hold on;
% plot3([origin(1) point1(1)],[origin(2) point1(2)],[origin(3) point1(3)],'r-^', 'LineWidth',2);
% plot3([origin(1) point2(1)],[origin(2) point2(2)],[origin(3) point2(3)],'g-^', 'LineWidth',2);
% plot3([origin(1) point3(1)],[origin(2) point3(2)],[origin(3) point3(3)],'b-^', 'LineWidth',2);
% grid on;
% xlabel('X axis'), ylabel('Y axis'), zlabel('Z axis')
% lim =10 ; 
% xlim([0 lim])  ; ylim([0 lim]) ; zlim([0 lim]) ; 
% set(gca,'CameraPosition',[1 2 3]);

% Pour chaque image, on applique au repere x0,y0,z0 la rotation et translation 
C = zeros(2,9) ; 
RT = zeros(3,4); % Stockage des [R|T] ; 

for i = 1:length(U) % Parcours des 10 images 
    
    for j = 1:length(WCF) % Parcours des 8 points
        u = U(j,i); v = V(j,i) ; 
        world_pose = WCF(j,:) ; 
        x = world_pose(1) ;  y = world_pose(2) ; 
        A_i = [0 0  0 , -x -y  -1,  v*x v*y  v ; x y  1 , 0 0  0 , -u*x -u*y  -u] ; 
        C = [C;A_i] ;
        
        
    end
    
    [U,S,V_svd] = svd(C(3:end,:)) ; 
    V_svd = V_svd(:,end) ;  % dernière colonne 
    V_sol = V_svd/(V_svd(end)) ; 

    H = [V_sol(1:3)  V_sol(4:6)  V_sol(7:9)]' ;
    K_inverseH = inv(K)*H ;

    % Vecteurs colonne
    % On estime les directions du repère de la caméra dans le WCF
    R_1_estim = K_inverseH(:,1) ; % direction de Xc
    R_2_estim = K_inverseH(:,2) ; % Direction de Yc 
    R_3_estim =  cross(R_1_estim , R_2_estim) ; % Direction de Zc (qui est aussi le produit vectoriel de x^y) 

    % Facteur d'echelle : 

    R_estim = [R_1_estim , R_2_estim , R_3_estim] ; 
    deter = det(R_estim) ;
    alpha = nthroot(deter, 4) ;

    % Calcul de la vraie matrice de rotation 

    R_1 = R_1_estim/(alpha) ; 
    R_2 = R_2_estim/(alpha) ; 
    R_3 = R_3_estim/(alpha^2) ; 
    R = [R_1 , R_2 , R_3] ; 
    T_estim = K_inverseH(:,3) ;
    T = T_estim/(alpha) ; 

    RT = [RT ; [R,T]] ; 
    
    C = zeros(2,9) ; 
    
end
 
    RT = RT(4:end,:);  
    
   %% PLOT
   a=1 ; b = 3 ;
   for i = 1:10 
        
        translation = RT(a:b,4); 
        rotation = RT(a:b,1:3); 
        % x = translation(1) ; y = translation(2) ; z= translation(3) ; 
        figure(1);hold on;
        %pose3(x, y, z, '+k', 'MarkerSize', 20);
        poseplot(rotation,translation); 
        a = a + 3 ; b = b + 3; 
   end
        grid on;
        xlabel('X axis'), ylabel('Y axis'), zlabel('Z axis') ; 
        set(gca,'CameraPosition',[1 2 3]);
end
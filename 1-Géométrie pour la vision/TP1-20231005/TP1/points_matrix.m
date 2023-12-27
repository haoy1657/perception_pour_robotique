function [B] = points_matrix(U,V,WCF)
% U, V sont les vecteurs contenant les coordonéees pixels de chaque set (plan image)
% WCF == world coordinate frame 
 B = zeros(2,9) ; 
for i= 1:length(U)% Parcours des mesures (10) 
    
    for j = 1:length(WCF) % Parcours des 8 points
        u = U(j,i); v = V(j,i) ; 
        world_pose = WCF(j,:) ; 
        x = world_pose(1) ;  y = world_pose(2) ; 
        A_i = [0 0  0 , -x -y  -1,  v*x v*y  v ; x y  1 , 0 0  0 , -u*x -u*y  -u] ; 
        
        %V2
        %A_i = [ x y 1 0 0 0 -u*x -u*y -u ; 0 0 0 x y 1 -v*x -v*y -v] ; % Un point genère deux lignes
        
        
        B = [B;A_i] ; 
     
    
    end
   
    
end

    B = B(3:end ,:) ; 

end

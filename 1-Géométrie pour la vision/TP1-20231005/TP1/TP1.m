clear all 
close all 
load('workspace.mat')
clc 
%% Application de la DLT pour trouver l'Homographie

% On dispose de 10 sets (images), et pour chaqune d'elles on a pris les
% coordonnées de 8 points (toujours les memes). Ces points sont renvoyés
% par MATLAB dans le repère image en pixels. chaque point sera associé à un
% (x,y) dans le repère monde qui commence dans le coin inferieur gauche de
% la mire.

% Coordonnées (x,y) des points 
wcf_x = [5 , 5 , 5 , 30 , 30 , 30 ]' ; 
wcf_y = [ 5 , 10 , 15 , 5 , 10 , 15 ]'; 

WCF = [ wcf_x , wcf_y] ; 
B = points_matrix(u_m,v_m,WCF); %  DLT 
display(B)

% Pour le calcul de la SVD == svd(matrix)
%%
SVD = svd(B) ; 
disp("Decomposition en valeurs singulières") ; 
display(SVD)

% On extrait la matrice V du SVD , on prend la dernière colonne puis
% normalisée par rapport au dernier element 

[U,S,V_svd] = svd(B) ;  
V_svd = V_svd(:,end) ;  % dernière colonne 
V_solution = V_svd/(V_svd(end)) ; 

disp("Solution pour H issue de la Decomposition en valeurs singulières") ; 
display(V_solution) ; 

disp("H sous forme 3x3") ; 
H = [V_solution(1:3)  V_solution(4:6)  V_solution(7:9)]' ;
disp(H) ; 

%% Trouver la matrice de rotation 
%------ On travaille avec la dernière image pour l'instant donc la dernière valeur de H ------%
% [R|T] est proportionelle à un facteur prés à K^-1 * H 
 
K_invH = inv(K)*H ; 
disp("K_inv*H") 
display(K_invH) 

% Vecteurs colonne
% On estime les directions du repère de la caméra dans le WCF
R_1_estimate = K_invH(:,1) ; % direction de Xc
R_2_estimate = K_invH(:,2) ; % Direction de Yc 
R_3_estimate =  cross(R_1_estimate , R_2_estimate) ; % Direction de Zc (qui est aussi le produit vectoriel de x^y) 

% Facteur d'echelle : 

% IMPORTANT 
% Les matrices de rotations sont orthogonales (R^-1 = R^T) et de determinant
% unitaire (pas de déformations). On definit chauque colonne estimée valant
% la vraie valeur à un facteur prés, alpha.(Le alpha carré pour R3 apparait
% à cause du produit vectoriel). 

R_estimate = [R_1_estimate , R_2_estimate , R_3_estimate] ; 
deter = det(R_estimate) ;
a = nthroot(deter, 4) ; 
disp("le facteur d'echelle a vaut : " ) ; disp(a) ; 

% Calcul de la vraie matrice de rotation 

R_1 = R_1_estimate/a ; 
R_2 = R_2_estimate/a ; 
R_3 = R_3_estimate/(a^2) ; 

disp("La matrice de rotation : ") 
R = [R_1 , R_2 , R_3] ; 
display(R) 

% Vecteur de translation (pose de la camera dans le WCF, lié à la dernière image)
disp("Le vecteur de translation : ") 
T_estimate = K_invH(:,3) ;
T = T_estimate/a ; 
disp(T) ; 

% prendre la nieme ligne : A(n,:)
% prendre la nieme ligne avec les elements compris entre les colonnes a et b  : A(n,a:b)
% prendre la nieme colonne :  A(:,n)
% prendre la nieme colonne avec les elements compris entre les ligns a et b  : A(a:b,n)

RT = camera_pose_RT(u_m,v_m,WCF,K) ; 


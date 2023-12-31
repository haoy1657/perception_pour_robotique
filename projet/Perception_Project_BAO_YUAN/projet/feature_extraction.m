function [feature_point]=feature_extraction(img1, img2,frame)
% Detect feature points between two images and return the Homogeneous coordinates of the pixels
% Load images
feature_point=[];
% img1=V1(:, :, 38);
% img2=V1(:, :, 39);
I1=img1;
I2=img2;
% Get the Key Points
Options.upright=true;
Options.tresh=0.0001;
Ipts1=OpenSurf(I1,Options);
Ipts2=OpenSurf(I2,Options);
% Put the landmark descriptors in a matrix
D1 = reshape([Ipts1.descriptor],64,[]); 
D2 = reshape([Ipts2.descriptor],64,[]); 
% Find the best matches
err=zeros(1,length(Ipts1));
cor1=1:length(Ipts1); 
cor2=zeros(1,length(Ipts1));
for i=1:length(Ipts1),
  distance=sum((D2-repmat(D1(:,i),[1 length(Ipts2)])).^2,1);
  [err(i),cor2(i)]=min(distance);
end
% Sort matches on vector distance
[err, ind]=sort(err); 
cor1=cor1(ind); 
cor2=cor2(ind);
% Show both images

I = zeros([size(I1,1) size(I1,2)*2 size(I1,3)]);
I(:,1:size(I1,2),:)=I1; I(:,size(I1,2)+1:size(I1,2)+size(I2,2),:)=I2;
figure(1);
imshow(I/255);
title(['box img=',num2str(frame),'/',num2str(frame+1)])
hold on;
% Show the best matches
for i=1:100
  c=rand(1,3);
  plot([Ipts1(cor1(i)).x Ipts2(cor2(i)).x+size(I1,2)],[Ipts1(cor1(i)).y Ipts2(cor2(i)).y],'-','Color',c)
  plot([Ipts1(cor1(i)).x Ipts2(cor2(i)).x+size(I1,2)],[Ipts1(cor1(i)).y Ipts2(cor2(i)).y],'o','Color',c)
end
pause(0.1)
hold off;
% Extract the first eight feature points
for i=1:100
  feature_point(1,i)=Ipts1(cor1(i)).x;
  feature_point(2,i)=Ipts1(cor1(i)).y;
  feature_point(3,i)=1;
  feature_point(4,i)=Ipts2(cor2(i)).x;
  feature_point(5,i)=Ipts2(cor2(i)).y;
  feature_point(6,i)=1;
end

end
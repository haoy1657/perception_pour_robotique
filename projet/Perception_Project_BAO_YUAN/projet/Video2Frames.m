function V=Video2Frames(pathtofile,n)

v=VideoReader(pathtofile);
N=v.NumberOfFrames;
disp('totalFrames');
disp(N);
N=n;
V=zeros(v.height,v.width,N);

%seems to be the recommanded method rather than the next one, used for
%readability 
%%%while hasFrame(v)
%%%    f=readFrame(v);
%%%    V2(:,:,k)=rgb2gray(f);
%%%    k=k+1;
%%%end

for i=1:N
    im=read(v,i);
    V(:,:,i)=double(rgb2gray(im));
end
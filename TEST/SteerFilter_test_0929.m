clear all;
close all;
clc;

sigma=0.5;
h=floor(2*sigma+1); 
w=h; 
[x y]=meshgrid(-w:w,-h:h);
Ga=0.9213*(-2.254*x+x.^3).*exp(-(x.^2+y.^2)/(2*sigma^2));   %各种三阶的幅度系数
Gb=1.843*(-0.7515+x.^2).*y.*exp(-(x.^2+y.^2)/(2*sigma^2));
Gc=0.9780*(-0.7515+y.^2).*x.*exp(-(x.^2+y.^2)/(2*sigma^2));
Gd=0.9780*(-2.254*y+y.^3).*exp(-(x.^2+y.^2)/(2*sigma^2));

img=double(imread('00001697.jpg'));
img_g = rgb2gray(img);
[m n]=size(img_g);
edge=zeros(m,n);

for i=0:10:360              %一次转过30度角
    theta=(i/180)*pi;
    Ka=cos(theta)^3;            %各种三阶的角度系数
    Kb=-3*cos(theta)^2*sin(theta);
    Kc=3*cos(theta)*sin(theta)^2;
    Kd=-sin(theta)^3;
    G=Ka*Ga+Kb*Gb+Kc*Gc+Kd*Gd;      %待卷积模板
    
    imgn=imfilter(img_g,G,'replicate');
    
    figure(1)
    imshow(imgn,[]);
    edge=sqrt(edge.^2+imgn.^2);
end

imshow(edge,[])
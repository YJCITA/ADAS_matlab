% syms r l f
% x=r*cos(l)*cos(f);
% y=r*cos(l)*sin(f);
% z=r*sin(l);
% J=jacobian([x;y;z],[r l f])

syms py vx bx fai bfai
x=vx*sin(fai);
y = -bx;
z = 0;
m = -bfai;
n = 0;

J=jacobian([x;y;z;m;n],[py vx bx fai bfai])
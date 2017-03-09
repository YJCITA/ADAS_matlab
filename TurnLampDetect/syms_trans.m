% syms roll pitch yaw
% C1=[cos(yaw) sin(yaw) 0;
%     -sin(yaw) cos(yaw) 0;
%        0       0      1];
% 
% C2=[cos(pitch) 0 -sin(pitch);
%         0    1    0;
%     sin(pitch) 0  cos(pitch)];
% 
% C3=[1  0  0;
%     0 cos(roll) sin(roll);
%     0 -sin(roll) cos(roll)];
% 
% R = C3*C2*C1

syms A[2] B[2]
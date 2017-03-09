clear;clc
syms roll pitch yaw  g real
% Z
C1=[cos(yaw) sin(yaw) 0;
    -sin(yaw) cos(yaw) 0;
       0       0      1];

% Y
C2=[cos(pitch) 0 -sin(pitch);
        0    1    0;
    sin(pitch) 0  cos(pitch)];

% X
C3=[1  0  0;
    0 cos(roll) sin(roll);
    0 -sin(roll) cos(roll)];

C=C3*C2*C1

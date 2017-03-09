clear;clc
syms roll pitch  yaw  i j k
A1 = [cos(yaw/2); 0; 0; sin(yaw/2)];
A2 = [cos(pitch/2); 0; sin(pitch/2); 0];
A3 = [cos(roll/2); sin(roll/2); 0; 0];

Q  = funQqCross(funQqCross(A1,A2), A3)

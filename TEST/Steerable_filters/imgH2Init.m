% Interpolation functions (note that theta is in radians):
%    H2_ka = cos(theta)^3; 
%    H2_kb = -3*(cos(theta)^2)*sin(theta);
%    H2_kc =  3*cos(theta)*(sin(theta)^2);
%    H2_kd = -sin(theta)^3;
%
% H2 at theta is defined as follows:
%    H_2 = H2_ka*H2a + H2_kb*H2b + H2_kc*H2c
function [H2a, H2b, H2c, H2d] = imgH2Init(img)

F1_H2 = [-0.0098  -0.0618  0.0998  0.7551  0.0    -0.7551  -0.0998  0.0618  0.0098];
F2_H2 = [ 0.0008   0.0176  0.1660  0.6383  1.0     0.6383   0.1660  0.0176  0.0008];
F3_H2 = [-0.0020  -0.0354 -0.2225 -0.4277  0.0     0.4277   0.2225  0.0354  0.0020];
F4_H2 = [ 0.0048   0.0566  0.1695 -0.1889 -0.7349 -0.1889   0.1695  0.0566  0.0048];


H2a = conv2(F2_H2,F1_H2,img,'same');
H2b = conv2(F3_H2,F4_H2,img,'same');
H2c = conv2(F4_H2,F3_H2,img,'same');
H2d = conv2(F1_H2,F2_H2,img,'same');
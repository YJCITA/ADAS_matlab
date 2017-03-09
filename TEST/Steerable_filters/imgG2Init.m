% Interpolation functions (note that theta is in radians):
%    G2_ka = cos(theta)^2;
%    G2_kb = -2*cos(theta)*sin(theta); 
%    G2_kc = sin(theta)^2; 
%
% G2 at theta is defined as follows:
%    G_2 = G2_ka*G2a + G2_kb*G2b + G2_kc*G2c
function [G2a, G2b, G2c] = imgG2Init(img)
 
 F1_G2 = [ 0.0094  0.1148  0.3964 -0.0601 -0.9213 -0.0601 0.3964 0.1148 0.0094];
 F2_G2 = [ 0.0008  0.0176  0.1660  0.6383  1.0     0.6383 0.1660 0.0176 0.0008];
 F3_G2 = [-0.0028 -0.0480 -0.3020 -0.5806  0.0     0.5806 0.3020 0.0480 0.0028]; 
 
 G2a = conv2(F2_G2,F1_G2,img,'same');
 G2b = conv2(F3_G2,F3_G2,img,'same');
 G2c = conv2(F1_G2,F2_G2,img,'same');
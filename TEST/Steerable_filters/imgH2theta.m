% - Assumption: theta given in degrees

function [H2_theta] = imgH2theta(H2a, H2b, H2c, H2d, theta_input)
syms theta

% --- H2 interpolation functions ---
H2_ka = cos((pi/180)*theta)^3; 
H2_kb = -3*(cos((pi/180)*theta)^2)*sin((pi/180)*theta);
H2_kc =  3*cos((pi/180)*theta)*(sin((pi/180)*theta)^2);
H2_kd = -sin((pi/180)*theta)^3;
% ---

H2_theta = subs(H2_ka,theta_input)*H2a...
           + subs(H2_kb,theta_input)*H2b...
           + subs(H2_kc,theta_input)*H2c...
           + subs(H2_kd,theta_input)*H2d; 
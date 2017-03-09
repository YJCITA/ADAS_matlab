% - Assumption: theta given in degrees
function [G2_theta] = imgG2theta(G2a, G2b, G2c, theta_input)
syms theta

% --- G2 interpolation functions ---
G2_ka = cos((pi/180)*theta)^2;
G2_kb = -2*cos((pi/180)*theta)*sin((pi/180)*theta); 
G2_kc = sin((pi/180)*theta)^2;
% ---     

G2_theta = subs(G2_ka,theta_input)*G2a...
           + subs(G2_kb,theta_input)*G2b...
           + subs(G2_kc,theta_input)*G2c; 

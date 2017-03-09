
% Ðý×ªË³Ðò Z-Y-X:yaw-pitch-roll
function [ euler ] = funQuat2Euler_ZXY( Q )
    Q = Q/(sqrt(sum(Q.^2)));
    q0 = Q(1);
    q1 = Q(2);
    q2 = Q(3);
    q3 = Q(4);
   % Cb2n
    C = [q0^2+q1^2-q2^2-q3^2   2*(q1*q2-q0*q3)       2*(q1*q3+q0*q2);
          2*(q1*q2+q0*q3)      q0^2-q1^2+q2^2-q3^2    2*(q2*q3-q0*q1);
          2*(q1*q3-q0*q2)       2*(q2*q3+q0*q1)       q0^2-q1^2-q2^2+q3^2];
    C = C';  % Cn2b
      
   
    roll = asin(C(2,3));
    pitch = atan2(-C(1,3),C(3,3));
    yaw = atan2(-C(2,1), C(2,2));
    
    
    euler = [roll, pitch, yaw]';


end


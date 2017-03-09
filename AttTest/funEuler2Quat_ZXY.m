% Cb2n
function [ Q ] = funEuler2Quat_ZXY( euler )
    roll = euler(1);
    pitch  = euler(2);
    yaw  = euler(3);
    
     Q = [cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
         cos(pitch/2)*cos(yaw/2)*sin(roll/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
         cos(roll/2)*cos(yaw/2)*sin(pitch/2) + cos(pitch/2)*sin(roll/2)*sin(yaw/2);
         cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2) ];
end


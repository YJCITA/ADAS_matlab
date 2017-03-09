% Cb2n
function [ Q ] = funEuler2Quat( euler )
    roll = euler(1);
    pitch  = euler(2);
    yaw  = euler(3);
    
    sin_roll_2 = sin(roll/2);
    cos_roll_2 = cos(roll/2);
    sin_pitch_2 = sin(pitch/2);
    cos_pitch_2 = cos(pitch/2);
    sin_yaw_2 = sin(yaw/2);
    cos_yaw_2 = cos(yaw/2);
%     Q = [cos_roll_2*cos_pitch_2*cos_yaw_2 + sin_roll_2*sin_pitch_2*sin_yaw_2;
%          sin_roll_2*cos_pitch_2*cos_yaw_2 - cos_roll_2*sin_pitch_2*sin_yaw_2;
%          cos_roll_2*sin_pitch_2*cos_yaw_2 + sin_roll_2*cos_pitch_2*sin_yaw_2;
%          cos_roll_2*cos_pitch_2*sin_yaw_2 - sin_roll_2*sin_pitch_2*cos_yaw_2 ];
     Q = [   cos(pitch/2)*cos(roll/2)*cos(yaw/2) + sin(pitch/2)*sin(roll/2)*sin(yaw/2)
             cos(pitch/2)*cos(yaw/2)*sin(roll/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
             cos(roll/2)*cos(yaw/2)*sin(pitch/2) + cos(pitch/2)*sin(roll/2)*sin(yaw/2)
             cos(pitch/2)*cos(roll/2)*sin(yaw/2) - cos(yaw/2)*sin(pitch/2)*sin(roll/2) ];
     

end


function [ R ] = funAtt2Rnb( att )
    roll = att(1);
    pitch = att(3);
    yaw = att(2);
    
    C1=[cos(yaw) sin(yaw) 0;
    -sin(yaw) cos(yaw) 0;
       0       0      1];

    C2=[cos(pitch) 0 -sin(pitch);
            0    1    0;
        sin(pitch) 0  cos(pitch)];

    C3=[1  0  0;
        0 cos(roll) sin(roll);
        0 -sin(roll) cos(roll)];
    
    R = C3*C2*C1;


end


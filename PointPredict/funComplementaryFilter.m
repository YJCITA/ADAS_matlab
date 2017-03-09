function [ att, AccAngle ] = funComplementaryFilter(att, acc, gyro, dt, mFactorAccAngle )
%     mFactorAccAngle = 0.5; % 加速度计角度的权值，现在先使用固定值，后期因根据加速度波动的情况采用变系数
    roll = att(1);
    pitch = att(2);
    yaw = att(3);
    
    % acc angle
    AccAngle = zeros(2,1);
    AccAngle(1) = atan2(acc(2), acc(3)); % roll
    AccAngle(2) = -atan2(acc(1), sqrt(acc(2)^2 + acc(3)^2));
    
    
    dgyro_angle1 = gyro*dt;
    
    if dt>2.5e-4 && cos(pitch)~=0
        F_w2euler = [1  sin(roll)*tan(pitch)  cos(roll)*tan(pitch);
                     0       cos(roll)            -sin(roll);
                     0  sin(roll)/cos(pitch)  cos(roll)/cos(pitch)];
        gyro_euler = F_w2euler*gyro;
        dgyro_angle = gyro_euler*dt;
        
        att(1) = (att(1) + gyro_euler(1)*dt) + mFactorAccAngle*( AccAngle(1) - att(1));
        att(2) = (att(2) + gyro_euler(2)*dt) + mFactorAccAngle*( AccAngle(2) - att(2));
        att(3) = (att(3) + gyro_euler(3)*dt);
    end

end


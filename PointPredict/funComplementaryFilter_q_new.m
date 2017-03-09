% 利用四元数进行姿态矩阵更新
function [att, Q_new, AccAngle] = funComplementaryFilter_q_new(Q, acc, gyro, dt )

    % ZYX
    AccAngle = zeros(2,1);
    AccAngle(1) = atan2(acc(2), acc(3)); % roll
    AccAngle(2) = atan2(-acc(1), sqrt(acc(2)^2 + acc(3)^2));    
    
    % 根据加速度计输出的模值来调整加速度计的系数
    % 0.8G<acc<1.2G  系数： 5*1e-3    
%     acc_length = sqrt(sum(acc.^2));
%     if acc_length > 9.5 && acc_length < 10.1
%         mFactorAccAngle = 2.5e-3;
%     else
%         mFactorAccAngle = 5e-4;
%     end
    mFactorAccAngle = 0;     
  
    dTheta = gyro*dt;
    dtheta = sqrt(sum(dTheta.^2));
    temp1 = (dTheta/dtheta)*sin(dtheta/2);
    q_h = [cos(dtheta/2), temp1(1), temp1(2), temp1(3)]';
    Q = funQqCross(Q,q_h );
    att_w = funQuat2Euler( Q ); % 到这个为止是OK的
    
    att = zeros(3,1);
    att(1) = att_w(1) + mFactorAccAngle*( AccAngle(1) - att_w(1));
    att(2) = att_w(2) + mFactorAccAngle*( AccAngle(2) - att_w(2));
    att(3) = att_w(3);        
    Q_new  = funEuler2Quat( att );

end


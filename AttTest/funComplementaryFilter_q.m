% 利用四元数进行姿态矩阵更新
function [att, Q, w_Inerl, w_P ] = funComplementaryFilter_q(Q, acc, gyro, dt )
%     mFactorAccAngle = 0.5; % 加速度计角度的权值，现在先使用固定值，后期因根据加速度波动的情况采用变系数
    persistent w_I;
    persistent Kp_error;
    if isempty(w_I)
        w_I = [0 0 0]';
    end
    
    if isempty(Kp_error)
        Kp_error = [0 0 0]';
    end
    
    Kp = 1;
    Ki = 0.0087;
   
    w = gyro;% - Kp_error; % - w_I;
    dTheta = w*dt;
    dtheta = sqrt(sum(dTheta.^2));
    temp1 = (dTheta/dtheta)*sin(dtheta/2);
    q_h = [cos(dtheta/2), temp1(1), temp1(2), temp1(3)]';
    Q = funQqCross(Q,q_h );
    att = funQuat2Euler( Q );
    %
    G_e = [0, 0, 1]';
    C = funQ2Cb2n( Q );
    G_b = acc/9.78; %sqrt(sum(acc.^2));
    error = cross(G_b, C'*G_e);
    w_I = w_I + Ki*error*dt;
    Kp_error = C*Kp*error;
    
    w_Inerl = w_I;
    w_P = Kp_error;
end


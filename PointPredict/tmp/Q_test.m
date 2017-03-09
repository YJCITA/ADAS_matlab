clc
clear all
close all

Q = [1 0 0 0]';
for i = 1:10*1e5
%     dTheta = [0.01 0.03 0]'/180*pi;
    dTheta = [0.033 0.1 0.1]'*1e-5;
    dtheta = sqrt(sum(dTheta.^2));
    temp1 = (dTheta/dtheta)*sin(dtheta/2);
    q_h = [cos(dtheta/2), temp1(1), temp1(2), temp1(3)]';
    Q = funQqCross(Q,q_h );
    att = funQuat2Euler( Q );
    att_save(:,i) = att*180/pi;
end

figure()
plot(att_save(1,:));
hold on;
plot(att_save(2,:));
plot(att_save(3,:));
legend('roll', 'pitch', 'yaw');
clc;
clear 
close all

%% 获取数据
% data = load('1.水平+pitch转动+平动.mat');
% data = load('2.平动.mat');
% data = load('3.水平静止.mat');
% data = load('IMU_testdata_pix.mat');
% data = load('4.pitch=90_再动pitch.mat');
data = load('5.pitch动.mat');


IMU_time = data.IMU(:, 2)'*1e-6; %
acc_raw =  data.IMU(:, 6:8)';
gyro_raw = data.IMU(:, 3:5)'; % rad/s

% roll pitch yaw
att_ekf = [data.ATT(:,4), data.ATT(:,6), data.ATT(:,8)]';% apm的EKF得到的姿态数据
att_q = [data.AHR2(1:end-1,3), data.AHR2(1:end-1,4), data.AHR2(1:end-1,5)]'; % apm互补滤波得到的姿态数据
att_ekf_time = data.ATT(:,2)'*1e-6;
att_q_time = data.AHR2(1:end-1,2)'*1e-6;

% 初始化变量
acc_filter = [0, 0, 0]'; % 一阶低通后的加速度计值
gyro_fiter = [0, 0, 0]';
acc_lowfilt_hz = 5; % 加速度计低通截止频率

att = zeros(3,1);
GyroAngle = zeros(3,1);
Q = [1 0 0 0]';
Q1 = Q;
j=1;

dt_err_num = 0; % dt = 0时，+1
%%
NUM = length(IMU_time);
for i = 2:NUM
%     i
    dt = (IMU_time(i) - IMU_time(i-1));
    if dt==0
        dt_err_num = dt_err_num + 1;
    end
    
% IMU数据一阶低通滤波
    acc_new =  acc_raw(: , i);
    [ acc_filter ] = funLowpassFilterVector3f( acc_filter, acc_new, dt, acc_lowfilt_hz );
    acc_filter_save(:,i-1) = acc_filter;
%互补滤波
    gyro_fiter = gyro_raw(: , i);
%     [ att, AccAngle ] = funComplementaryFilter(att, acc_filter, gyro_fiter, dt, 1e-4 );
%     [att, Q , w_I, w_P] = funComplementaryFilter_q(Q, acc_filter, gyro_fiter, dt );    
    [att, Q ,AccAngle] = funComplementaryFilter_q_new(Q, -acc_filter, gyro_fiter, dt ); 
    [GyroAngle, Q1 ] = funComplementaryFilter_q(Q1, -acc_filter, gyro_fiter,  dt );
    
    IMU_trim = [0.002248888, 0.04422353, 0]'*180/pi;
    
    att_save(:,i-1) = att*180/pi - IMU_trim;
    AccAngle_save(:,i-1) = AccAngle*180/pi - IMU_trim(1:2);
    GyroAngle_save(:,i-1) = GyroAngle*180/pi - IMU_trim;
    
%     w_I_save(:,i-1) = w_I;
%     w_P_save(:,i-1) = w_P;
end

figure()
plot(IMU_time(2:end),att_save(1, :));
hold on;
grid on;
plot(IMU_time(2:end), AccAngle_save(1, :));
plot(IMU_time(2:end), GyroAngle_save(1, :));
plot(att_q_time, att_q(1, :));
% plot(IMU_time(2:end), w_P_save(1, :));
% plot(att_ekf_time, att_ekf(1, :));
% legend('att','q', 'ekf');
legend('att', 'AccAngle', 'GyroAngle','q','ekf');

figure()
plot(IMU_time(2:end), att_save(2, :));
hold on;
grid on;
plot(IMU_time(2:end), AccAngle_save(2, :));
plot(IMU_time(2:end), GyroAngle_save(2, :));
plot(att_q_time, att_q(2, :));
% plot(IMU_time(2:end), w_P_save(2, :));
% plot(att_ekf_time, att_ekf(2, :));
% legend('att','q', 'ekf');
legend('att', 'AccAngle', 'GyroAngle','q','ekf');

figure()
plot(IMU_time(2:end), att_save(3, :));
hold on;
grid on;
plot(IMU_time(2:end), GyroAngle_save(3, :));
plot(att_q_time, att_q(3, :)- mean(att_q(3, 10:50)));
% plot(att_ekf_time, att_ekf(2, :));
% legend('att','q', 'ekf');
legend('att','GyroAngle','q');
% 
% figure()
% plot(IMU_time, gyro_raw(1, :));
% hold on;
% grid on;
% plot(IMU_time(2:end), w_I_save(1, :));
% legend('w','WI');


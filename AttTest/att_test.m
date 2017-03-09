% 在互补滤波的基础上分析直接gyro积分的性能

clc
clear all
close all

%% 读入数据
att_camera = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-att.ini')';
att_VN300 = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-VN300.ini')';
att_fmu = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-fmu.ini')';
imu_data = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-imu.ini')';

% 统一时间初始点
time0_camera = att_camera(1, 1);
time0_VN300 = att_VN300(1, 1);
time0_fmu = att_fmu(1, 1);
time0 = min(min(time0_camera, time0_VN300), time0_fmu);

att_camera(1, :) = att_camera(1, :) - time0;
att_VN300(1, :) = att_VN300(1, :) - time0;
att_fmu(1, :) = att_fmu(1, :) - time0;
imu_data(1, :) = imu_data(1, :) - time0; % acc*3 gyro*3

%% 角度对准 坐标系统一
att0_camera = [att_camera(2:3, 5); 0];
R_camera = funAtt2Rnb( att0_camera/180*pi );

att0_VN300 = [att_VN300(2:3, 5); 0];
R_VN300 = funAtt2Rnb( att0_VN300/180*pi );

att0_fmu = [att_fmu(2:3, 5); 0];
R_fmu = funAtt2Rnb( att0_fmu/180*pi );

att_camera_new = att_camera(2:4,:) - att0_camera;
att_VN300_new = att_VN300(2:4,:) - att0_VN300;
att_fmu_new = att_fmu(2:4,:) - att0_fmu;


%% 互补滤波计算
% 初值
% 用加速度的前50：100数据的平均值计算初始角度，给att赋初值
AccAngle = zeros(2,1);
acc_init_data = imu_data(2:4, 10);
AccAngle(1) = atan2(-acc_init_data(2), -acc_init_data(3)); % roll
AccAngle(2) = atan2(acc_init_data(1), sqrt(acc_init_data(2)^2 + acc_init_data(3)^2));
att = [AccAngle(1), AccAngle(2), 0]';
Q = funEuler2Quat( att); 
Q1 = Q;
dt = 0.01;
NUM_imu = length(imu_data);
for i = 2:NUM_imu
    %互补滤波    
    dt = imu_data(1, i) - imu_data(1, i-1);
    if(dt == 0)
        dt = 0.02;
    end
    acc_filter = imu_data(2:4, i);
    gyro_filter = imu_data(5:7, i);
    [GyroAngle, Q ,AccAngle] = funComplementaryFilter_q_new(Q, acc_filter, gyro_filter, dt );
%     [GyroAngle, Q1 ] = funComplementaryFilter_q(Q1, acc_filter,gyro_filter,dt );
    GyroAngle_save(:,i) = GyroAngle*180/pi;   
end

%% 画图
% 各种算法的姿态解算结果
figure()
% roll
subplot(3, 1, 1)
plot(att_camera(1, :), GyroAngle_save(1, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(1, :));
plot(att_fmu(1, :), att_fmu_new(1, :));
grid on;
legend('att-camera-roll', 'att-VN300-roll', 'att-fmu-roll');

subplot(3, 1, 2)
plot(att_camera(1, :), GyroAngle_save(2, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(2, :));
plot(att_fmu(1, :), att_fmu_new(2, :));
grid on;
legend('att-camera-pitch', 'att-VN300-pitch', 'att-fmu-pitch');

subplot(3, 1, 3)
plot(att_camera(1, :), GyroAngle_save(3, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(3, :));
plot(att_fmu(1, :), att_fmu_new(3, :));
grid on;
legend('att-camera-yaw', 'att-VN300-yaw', 'att-fmu-yaw');


% imu数据和姿态
figure()
subplot(3, 1, 1)
plot(att_camera(1, :), GyroAngle_save(1, :));
hold on;
plot(imu_data(1, :), imu_data(5, :)*180/pi); % gyro-X
plot(imu_data(1, :), imu_data(3, :)); % acc-Y
grid on;
legend('att-camera-roll', 'gyro-X', 'acc-Y');

subplot(3, 1, 2)
plot(att_camera(1, :), GyroAngle_save(2, :));
hold on;
plot(imu_data(1, :), imu_data(6, :)*180/pi); % gyro-Y
plot(imu_data(1, :), imu_data(2, :)); % acc-X
grid on;
legend('att-camera-pitch', 'gyro-Y', 'acc-X');

subplot(3, 1, 3)
plot(att_camera(1, :), GyroAngle_save(3, :));
hold on;
plot(imu_data(1, :), imu_data(7, :)*180/pi);
grid on;
legend('att-camera-yaw', 'gyro-z');






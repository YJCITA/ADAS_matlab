clc
clear all
close all

%% 读入数据
att_camera = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-att.ini')';
att_VN300 = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-VN300.ini')';
att_fmu = load('./data/0215_VN_FMU_IMU/20170215_VN_FMU_IMU-fmu.ini')';

% 统一时间初始点
time0_camera = att_camera(1, 1);
time0_VN300 = att_VN300(1, 1);
time0_fmu = att_fmu(1, 1);
time0 = min(min(time0_camera, time0_VN300), time0_fmu);

att_camera(1, :) = att_camera(1, :) - time0;
att_VN300(1, :) = att_VN300(1, :) - time0;
att_fmu(1, :) = att_fmu(1, :) - time0;

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
%% 画图
figure()
% roll
subplot(3, 1, 1)
plot(att_camera(1, :), att_camera_new(1, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(1, :));
plot(att_fmu(1, :), att_fmu_new(1, :));
grid on;
legend('att-camera-roll', 'att-VN300-roll', 'att-fmu-roll');

subplot(3, 1, 2)
plot(att_camera(1, :), att_camera_new(2, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(2, :));
plot(att_fmu(1, :), att_fmu_new(2, :));
grid on;
legend('att-camera-pitch', 'att-VN300-pitch', 'att-fmu-pitch');

subplot(3, 1, 3)
plot(att_camera(1, :), att_camera_new(3, :));
hold on;
plot(att_VN300(1, :), att_VN300_new(3, :));
plot(att_fmu(1, :), att_fmu_new(3, :));
grid on;
legend('att-camera-yaw', 'att-VN300-yaw', 'att-fmu-yaw');


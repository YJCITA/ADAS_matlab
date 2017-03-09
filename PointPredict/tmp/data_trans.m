clc;
clear all
close all

one_G = 9.80665;
% #4
% A0 = [ -0.0056, -0.0080, -0.0433]';
% A1 = [0.9992   -0.0020    0.0063
%      -0.0009    0.9999   -0.0114
%       0.0197   -0.0016    1.0090];
% A2 = [ 0.0041    0       0;
%        0     -0.0012     0;
%        0         0    0.0362];  
   
% 16号模组加速度计 52度左右
A0 = [ 0.0197, -0.0161, -0.0215]';
A1 = [ 0.9981   -0.0178    0.0014
       0.0072    0.9988   -0.0067
      -0.0346    0.0423    1.0024];
A2 = [  -0.0054         0            0
           0         -0.0050         0
           0            0      -0.0029];


%% 获取数据
% data = importdata('data_new/5.txt');
data = importdata('data_log_gsensor_2.txt');

NUM = length(data(:, 4))-1;
% time
time_s = data(1:NUM, 1)';
time_us = data(1:NUM, 2)';
IMU_time = (time_s - time_s(1)) + (time_us - time_us(1))*1e-6;

% acc 
accel_range_scale = 8/(2^15);% acc_max_g = 8g
acc_raw = data(1:NUM, 4:6)'*accel_range_scale;
for j = 1:NUM
   acc_cal_tmp(:, j) = (A0 + A1*acc_raw(:, j) + A2*(acc_raw(:, j).^2))*one_G;
%     acc_cal(:, j) = (A0 + A1*acc_raw(:, j))*one_G;
end

% gyro
gyro_range_scale = 2000/180*pi/(2^15); %acc_max_g = 2000 degree/s   rad/s
gyro_cal_tmp = data(1:NUM, 7:9)'*gyro_range_scale;

% 坐标轴变换
acc_cal(1,:) = -acc_cal_tmp(3,:);
acc_cal(2,:) = -acc_cal_tmp(2,:);
acc_cal(3,:) = -acc_cal_tmp(1,:);
gyro_cal(1,:) = -gyro_cal_tmp(3,:);
gyro_cal(2,:) = -gyro_cal_tmp(2,:);
gyro_cal(3,:) = -gyro_cal_tmp(1,:);

for i = 2:NUM  
    dt = (time_s(i)-time_s(i-1)) + (time_us(i)-time_us(i-1))*1e-6;
    dt_save(i) = dt;
end

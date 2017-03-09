% 0928: 增加彩图显示，优化结构
% 1008: 跟踪不准问题分析:
%       1)验证了在陀螺仪零偏可以估计的情况下，用IMU+speed计算运动轨迹ok
%       2)问题：VN300的姿态与速度在航向角方向有误差（此段数据是7°左右，会影响速度数据的使用）
clc;
clear all; 
close all

SAVE_R = 0; % 保存R
SHOW_IPM = 1; % 是否用于IPM显示转弯半径 
%% 数据导入
% 转弯半径
source_addr = 'data/0924_R/';
R_addr = [source_addr, 'R_data.mat'];% 转弯半径
load(R_addr);

% 相机时间
% data_resample.mat---data_gensor_resample data_steer_resample data_speed_resample time_start fs_HZ
resample_data_addr = [source_addr, 'data_resample.mat'];
load(resample_data_addr);
% 处理一下陀螺仪的零漂
data_gensor_resample_old = data_gensor_resample;
for i = 1:3
%     w_drift(i, 1) = mean(data_gensor_resample(i+3, 100000:100100));
    w_drift(i, 1) = 0;
    data_gensor_resample(i+3, :) = data_gensor_resample(i+3, :) - w_drift(i,1);
end

image_file_name = '152546';
current_image_file_num = str2num(image_file_name);
camera_log_name = '0924_VN300-camera.ini';
addr_camera_log = [source_addr, camera_log_name]; % 相机时间  time_s,time_us,name,index
raw_data_camera_log = importdata( addr_camera_log )';
time_s = raw_data_camera_log(1, :);
time_us = raw_data_camera_log(2, :);
time_camera_log = time_s + time_us *1e-6 - time_start;
image_file_name_num = raw_data_camera_log(3, :);
NUM_camera_image = length(image_file_name_num);
j = 0;
for i = 1:NUM_camera_image
    if image_file_name_num(i) == current_image_file_num
        j = j + 1;
        data_camera_log(:, j) = [time_camera_log(i); raw_data_camera_log(4, i); raw_data_camera_log(3, i)];
    end
end

% 写入txt
save_R_addr = ['./', source_addr, 'car_move_radius.ini'];
if SAVE_R
     fp = fopen(save_R_addr, 'wt');
end  

%% 初始化参数
camera_parameter.m= 720; % v (height)
camera_parameter.n = 1280; % u (width)
camera_parameter.h = 1.2; % Distance camera was above the ground (meters)
camera_parameter.theta0 = 1.8*pi/180; % 2.373; 
camera_parameter.gama0 = 0; % 水平倾角
camera_parameter.Pc =  [0 0 -camera_parameter.h]'; % 地理坐标系下相机中心坐标点  ;
fx = 1506.64297;
fy = 1504.18761;
cx = 664.30351;
cy = 340.94998;
camera_parameter.M1 = [fx  0 cx; 0  fy cy; 0  0  1 ];

h = 1.2; % Distance camera was above the ground (meters)
d = 0;	% 横向偏移
l = 0;  % 纵向偏移
theta0 = 1.8*pi/180;%2.373; 

% Y1 相机内参
M1 = [fx   0 cx;
       0  fy cy;
       0   0  1 ];
Pc = [0 0 -h]'; % 地理坐标系下相机中心坐标点  
Rc12c = [0 1 0;% 相机-图像坐标系  
         0 0 1;
         1 0 0];
% 相机姿态矩阵     
Ratt = [cos(theta0)  0  -sin(theta0);
             0        1        0;
         sin(theta0)  0  cos(theta0)];
Ratt_new = Rc12c*Ratt;
I3 = diag([1,1,1]);
R_IPM = M1*Rc12c*Ratt; % 用于IPM


% 俯视图 参数
camera_parameter.x_min = 1; % 摄像头pitch向上，导致近距离看不见。
camera_parameter.x_max = 70; % 纵向
camera_parameter.y_min = -6;
camera_parameter.y_max = 6; % 横向
camera_parameter.H1 = 400;
camera_parameter.W1 = 250;  %需要显示图像的高和宽
camera_parameter.zoom = 20;

% 汽车参数
car_parameter.L = 2.637;
car_parameter.K_s2w = 0.0752;% 方向盘转角->前轮转角
L = 2.637;
L_r = L/2;
K_s2w = 0.07; %0.0752;% 0.059; % 方向盘转角->前轮转角
fai = 0;
x_CAN = 0;
y_CAN = 0;

x_VN300 = 0;
y_VN300 = 0;
  
%% 主循环
save_i = 0;
save_yaw_i = 0;
current_index = 0; % 当前匹配的index
is_R_Camera_matched = 0; % R和图片时间是否匹配
is_FirstTimeLoop = 1;
dyaw = 0;
gyro_fiter = [0 0 0]';
% 用于计算轨迹
isHaveFirstMatch = 0; % 是否已经第一次匹配上
init_origin = 0;
yaw_xy = 0;
dt = 1/fs_HZ;

yaw_VN300_origin = data_VN300_resample(3, 24404)-7;
w_z_drift = mean(data_gensor_resample(6, 24404:24404+100));
for k = 1:1:2000
    %% 匹配R和图片的时间
    is_R_Camera_matched = 0;
    while ~is_R_Camera_matched
        current_index = current_index + 1;
        dt_now = data_camera_log(1,k) - current_index/fs_HZ; % R经过80hz的重采样
        dt_next = data_camera_log(1,k) - (current_index+1)/fs_HZ;
        if dt_now>0 && dt_next<=0
            R_current_imu_speed = R_imu_speed(current_index);   
            is_R_Camera_matched = 1;
            isHaveFirstMatch = 1;            
        end
        
        if isHaveFirstMatch && ~init_origin
            init_origin = 1;
            yaw_VN300_origin = data_VN300_resample(3, current_index);
            w_z_drift = mean(data_gensor_resample(6, current_index:current_index+100)); 
        end
        
        if isHaveFirstMatch
           %% 车子运动轨迹解算
            % 计算角度变化
            gyro_new = data_gensor_resample(4:6, current_index) - [0 , 0, w_z_drift]';
            gyro_fiter = funLowpassFilterVector3f( gyro_fiter, gyro_new, 1/fs_HZ, 20 );
            dyaw = dyaw + gyro_fiter(3)*1/fs_HZ;
            yaw_xy = yaw_xy + gyro_fiter(3)*1/fs_HZ; 

            speed_new = data_speed_resample(1, current_index);
            dP = speed_new*dt;
            x_CAN = x_CAN + dP*cos(yaw_xy);
            y_CAN = y_CAN + dP*sin(yaw_xy); 

            % 用VN300计算轨迹
            Vx_VN300 = data_VN300_resample(4, current_index);
            Vy_VN300 = data_VN300_resample(5, current_index);
            R_vn300 = [cosd(yaw_VN300_origin), sind(yaw_VN300_origin);
                        -sind(yaw_VN300_origin), cosd(yaw_VN300_origin)];
            V_Vn300_new = R_vn300*[Vx_VN300, Vy_VN300]';
            
            x_VN300 =  x_VN300 + V_Vn300_new(1)*dt;
            y_VN300 =  y_VN300 + V_Vn300_new(2)*dt;
            
            save_i = save_i + 1;    
            save_xy(:, save_i) = [x_CAN, y_CAN]';
            save_xy_VN300(:, save_i) = [x_VN300, y_VN300]';
            save_gyro_fiter(:, save_i) = gyro_fiter;
            save_speed(:, save_i) = speed_new;
            save_gyro(:, save_i) = gyro_new;
            save_yaw_xy(:, save_i) = yaw_xy;
            save_v_VN300(:, save_i) = V_Vn300_new;
            save_v_new_VN300(:, save_i) = [Vx_VN300, Vy_VN300]';
            save_yaw_VN300(:, save_i) = [data_VN300_resample(3, current_index)]';
            save_gyro_VN300(:, save_i) = data_VN300_resample(7:9, current_index);

        end
    end


end

% 运动轨迹
figure()
plot(save_xy_VN300(2, :), save_xy_VN300(1, :)); hold on;
plot(save_xy(2, :), save_xy(1, :));
legend('vn300-trajectory', 'car-trajectory')
title('VN300 轨迹')

% figure()
% plot(save_xy(2, :), save_xy(1, :));
% legend('car-trajectory')
% title('car 轨迹')



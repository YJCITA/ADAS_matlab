% 0928: TODO 增加彩图显示，优化结构
clc;
clear all; 
close all

SAVE_R = 0; % 保存R
SHOW_IPM = 0; % 是否用于IPM显示转弯半径 
%% 数据导入
source_addr = 'data/1024_nj_radius/16/';
% data_resample.mat---data_gensor_resample data_steer_resample data_speed_resample time_start fs_HZ
resample_data_addr = [source_addr, 'data_resample.mat'];
load(resample_data_addr);


%% 图像log
camera_log_name = '16log-camera.ini';
image_file_name = '163932';
current_image_file_num = str2num(image_file_name);
addr_camera_log = [source_addr, camera_log_name]; % 相机时间  time_s,time_us,name,index
raw_data_camera_log = importdata( addr_camera_log )';
time_s = raw_data_camera_log(1, :);
time_us = raw_data_camera_log(2, :);
time_camera_log = time_s + time_us *1e-6 - time_start;
image_file_name_num = raw_data_camera_log(4, :);
NUM_camera = length(image_file_name_num);
j = 0;
for i = 1:NUM_camera
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

%% 主循环
save_i = 0;
current_index = 0; % 当前匹配的index
is_R_Camera_matched = 0; % R和图片时间是否匹配
is_FirstTimeLoop = 1;
gyro_fiter = [0 0 0]';

%% 计算转弯半径
%% 相机 IMU 数据预处理 acc gyro
NUM_gsensor = length(data_gensor_resample(1,:));
NUM = length(data_gensor_resample);
clear time
for i = 1:NUM
    time(1, i) = 1/fs_HZ*i;
end
data_imu = [time; data_gensor_resample;];
imu_time = time;
NUM_imu = length(data_imu);

one_G = 9.80665; 
% Y1模组的加速度计校正参数
A0 = [0.0328    0.0079   -0.0003]'; %[0.0628    0.0079   -0.0003]';
A1 = [  1.0011    0.0028   -0.0141;
       -0.0161    1.0005    0.0181;
        0.0163   -0.0065    1.0140 ];
A2 = [ -0.0047    0       0;
         0      0.0039    0;
         0         0    0.0124];
% acc 
accel_range_scale = 8/(2^15);% acc_max_g = 8g
acc_raw = data_imu(2:4, :)*accel_range_scale;
acc_raw_G = acc_raw.*one_G;
inv_A1 = inv(A1);
for j = 1:NUM_imu
   acc_cal_tmp(:, j) = inv_A1*(acc_raw(:, j) - A0)*one_G;
end

% gyro
gyro_range_scale = 2000/180*pi/(2^15); %acc_max_g = 2000 degree/s   rad/s
gyro_cal_tmp = data_imu(5:7, :)*gyro_range_scale;

% 陀螺仪零偏
for i = 1:3
%     w_drift(i) = mean(gyro_cal_tmp(i,2450:2460)); 
    gyro_cal_tmp(i,:) = gyro_cal_tmp(i,:); % - w_drift(i);
end

% Y1模组
acc_cal(1,:) = -acc_cal_tmp(3,:);
acc_cal(2,:) = -acc_cal_tmp(2,:);
acc_cal(3,:) = -acc_cal_tmp(1,:);
gyro_cal(1,:) = -gyro_cal_tmp(3,:);
gyro_cal(2,:) = -gyro_cal_tmp(2,:);
gyro_cal(3,:) = -gyro_cal_tmp(1,:);

IMU_w = gyro_cal;

NUM_speed = length(data_speed_resample(1,:));
dt = 1/80;
IMU_w_filter = IMU_w(:, 1);
NUM_loop = length(data_speed_resample);
for i = 1 : NUM_loop  % 数据长度可能不一致      
  % 计算半径
    if i < NUM_gsensor
        IMU_w_filter = funLowpassFilterVector3f( IMU_w_filter, IMU_w(:, i), dt, 1 );
        speed_new = data_speed_resample(1, i);
        if abs(IMU_w_filter(3)) < 0.01 % 0.01
            R_imu_speed(i) = 0;
        else
            R_imu_speed(i) = speed_new/IMU_w_filter(3);
        end  
        time_t = time_start + i*(1/fs_HZ);
        R_imu_speed_time(:, i) = [time_t, R_imu_speed(i)];
    end 
end


for k = 1:NUM_camera
    save_i = save_i + 1;
    
    %% 匹配R和图片的时间
    is_R_Camera_matched = 0;
    while ~is_R_Camera_matched
        current_index = current_index + 1;
        dt_now = data_camera_log(1,k) - current_index/fs_HZ; % R经过80hz的重采样
        dt_next = data_camera_log(1,k) - (current_index+1)/fs_HZ;
        if dt_now>0 && dt_next<=0
            R_current_imu_speed = R_imu_speed(current_index);   
            R_image_file_name = raw_data_camera_log(3, current_index);
            is_R_Camera_matched = 1;
        end
 
    end
        

    %% 保存转弯半径数据
    if SAVE_R
        % 写入txt
        fprintf(fp, '%d %f %d ', k, R_current_imu_speed, R_image_file_name);
        fprintf(fp, ' \n');
    end

    if SHOW_IPM
    %% 图片 IPM      
        % 读取图片数据
        image_name = sprintf('/%08d.jpg',k);
        str_data = [source_addr, image_file_name, image_name];
        I_rgb = imread(str_data);
        I_g = rgb2gray(I_rgb);
        [m, n] = size(I_g);

        % 遍历设定的俯视图中的所有像素，线转为地理坐标系XYZ，再通过小孔成像模型映射到相机图像中的uv
        % 这样可以保证俯视图中所有的像素都会有对应的相机成像点
        % IPM变换
        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波

       %% 画线
        % R_current_imu_speed
%         rgb_value_t = 240; % 0:黑色
%         [ CC_rgb ] = fun_plot_R( R_current_imu_speed, CC_rgb, rgb_value_t, camera_parameter);

        
        figure(1);
        str_name = sprintf('frame%d俯视图',k);
        title(str_name); 
        imshow(CC_rgb);       
        
        is_FirstTimeLoop = 0;

    end
end





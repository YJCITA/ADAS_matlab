% 0928: TODO 增加彩图显示，优化结构
clc;
clear all; 
close all

SAVE_R = 0; % 保存R
SHOW_IPM = 1; % 是否用于IPM显示转弯半径 
%% 数据导入
% 转弯半径
source_addr = 'data/lane/0930_curve/';
R_addr = [source_addr, 'R_data.mat'];% 转弯半径
load(R_addr);

% 相机时间
% data_resample.mat---data_gensor_resample data_steer_resample data_speed_resample time_start fs_HZ
resample_data_addr = [source_addr, 'data_resample.mat'];
load(resample_data_addr);
% 处理一下陀螺仪的零漂
data_gensor_resample_old = data_gensor_resample;
for i = 1:3
    w_drift(i, 1) = mean(data_gensor_resample(i+3, 100000:100100));
    data_gensor_resample(i+3, :) = data_gensor_resample(i+3, :) - w_drift(i,1);
end

image_file_name = '163932';
current_image_file_num = str2num(image_file_name);
camera_log_name = 'log_G_C_0921-camera.ini';
addr_camera_log = [source_addr, camera_log_name]; % 相机时间  time_s,time_us,name,index
raw_data_camera_log = importdata( addr_camera_log )';
time_s = raw_data_camera_log(1, :);
time_us = raw_data_camera_log(2, :);
time_camera_log = time_s + time_us *1e-6 - time_start;
image_file_name_num = raw_data_camera_log(3, :);
NUM_camera = length(image_file_name_num);
j = 0;
for i = 1:NUM_camera
    if image_file_name_num(i) == current_image_file_num
        j = j + 1;
        data_camera_log(:, j) = [time_camera_log(j); raw_data_camera_log(4, j); raw_data_camera_log(3, j)];
    end
end

% 车道线特征点 左右两条，每条8个特征点
address_lane = [source_addr, 'lane_feature_163932.txt'];
lane_feature_raw_data = load(address_lane)'; % data_lane: lane index*1, 左右车道分别8个点
NUM_lane = length(lane_feature_raw_data(1, :)); % 样本数量
lane_feature_data.frame_index = lane_feature_raw_data(1, :);
for i = 1:8
    lane_feature_data.left_uv_feature(i, :, :) = lane_feature_raw_data(2*i:2*i+1, :);
    lane_feature_data.right_uv_feature(i, :, :) = lane_feature_raw_data(2*i+16:2*i+17, :);
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
  
%% 主循环
save_i = 0;
save_yaw_i = 0;
current_index = 0; % 当前匹配的index
is_R_Camera_matched = 0; % R和图片时间是否匹配
is_FirstTimeLoop = 1;
dyaw = 0;
gyro_fiter = [0 0 0]';
% 1900:5:3700
% 4600:5:7000
for k_lane = 1:4:NUM_lane
    k = lane_feature_data.frame_index(k_lane);  
    save_i = save_i + 1;
    
    XY_CAN_pre = [x_CAN, y_CAN]';
    fai_pre  = fai;
    % 保存上一帧车道线的特征点，用于预测
    if is_FirstTimeLoop == 1
        xy_L_pre = ones(2,8);
        xy_R_pre = ones(2,8);
    else
        line_p_L_pre = line_p_L;
        line_p_R_pre = line_p_R;
    end
    
    %% 匹配R和图片的时间
    is_R_Camera_matched = 0;
    while ~is_R_Camera_matched
        current_index = current_index + 1;
        dt_now = data_camera_log(1,k) - current_index/fs_HZ; % R经过80hz的重采样
        dt_next = data_camera_log(1,k) - (current_index+1)/fs_HZ;
        if dt_now>0 && dt_next<=0
            R_current_imu_speed = R_imu_speed(current_index);   
            is_R_Camera_matched = 1;
        end
        
        %% 车子运动轨迹解算
        if current_index <= 1 % 第一次进入
            beta = 0;
            fai = 0;
            steer_new = 0;
        else               
            speed_new = data_speed_resample(1, current_index)/3.6;  
            steer_new = data_steer_resample(1, current_index);
            dt_CAN = 1/fs_HZ;
            if abs(steer_new)>5 % 方向盘小角度不计算
                wheel_angle = (K_s2w*steer_new)*(pi/180); % 前轮角度 弧度                   
                beta = atan(L_r*tan(wheel_angle)/L);  % 计算 beta
                d_fai = speed_new/L*tan(wheel_angle)*cos(beta)*dt_CAN;   
            else
                wheel_angle = 0;
                d_fai = 0;
            end
            fai = fai + d_fai;
            Vx = speed_new*cos(fai + beta);
            Vy = speed_new*sin(fai + beta);
            x_CAN = x_CAN + Vx*dt_CAN;
            y_CAN = y_CAN + Vy*dt_CAN;                
        end            
   
        % 计算角度变化
        gyro_new = data_gensor_resample(4:6, current_index);
        gyro_fiter = funLowpassFilterVector3f( gyro_fiter, gyro_new, 1/fs_HZ, 20 );
        dyaw = dyaw + gyro_fiter(3)*1/fs_HZ;
        
        save_xy(:, save_i) = [x_CAN, y_CAN]';
        save_steer(:, save_i) = steer_new;
        save_gyro_fiter(:, save_i) = gyro_fiter;
    end
    
%% 预测车道线
    % IPM后的手工标注特征点在图像中的坐标
%     dyaw = -dyaw;
    dyaw_degree = dyaw*180/pi
    save_yaw_i = save_yaw_i + 1;
    save_dyaw(save_yaw_i) = dyaw_degree;
    Rn2c_kT = [ cos(dyaw) sin(dyaw);
                -sin(dyaw) cos(dyaw)];
    
%     dyaw1 = 0;
%     Rn2c_kT1 = [ cos(dyaw1) sin(dyaw1);
%         -sin(dyaw1) cos(dyaw1)];
            
    XY_CAN_cur = [x_CAN, y_CAN]';
    P_kT_t = XY_CAN_cur - XY_CAN_pre;
    dT_fai = fai_pre;
    dT_fai_deg = dT_fai*180/pi;
    R_dfai = [ cos(dT_fai) sin(dT_fai);
                -sin(dT_fai) cos(dT_fai)]; 
    P_kT = R_dfai*P_kT_t;      
    % 特征点IPM到世界坐标系上
    for j = 1:8
        % 预测
%         P_kT = [0 0]';
        XK_L_predict(:, j) = Rn2c_kT*( xy_L_pre(:, j) - P_kT); % 预测车道线在当前帧的坐标
        XK_R_predict(:, j) = Rn2c_kT*( xy_R_pre(:, j) - P_kT); % 预测车道线在当前帧的坐标
    end  
    % 参数辨识
    Y(:, 1) = XK_L_predict(2, :);
    X_t(:, 1) = XK_L_predict(1, :);
    I_8 = ones(8,1);
    X = [X_t.^2, X_t, I_8];
    line_p_L_predict(:, 1) = inv(X'*X)*X'*Y;   
    
    Y(:, 1) = XK_R_predict(2, :);
    X_t(:, 1) = XK_R_predict(1, :);
    I_8 = ones(8,1);
    X = [X_t.^2, X_t, I_8];
    line_p_R_predict(:, 1) = inv(X'*X)*X'*Y;    
    
%% 拟合当前曲线
    % IPM后的手工标注特征点在图像中的坐标
    for j = 1:8
        % 矩阵式逆投影变化
        uv_L_new = [lane_feature_data.left_uv_feature(j, 1:2, k_lane)'; 1];
        xy_L_tmp = inv(R_IPM)*uv_L_new;
        s_2 = h/(xy_L_tmp(3)); % 缩放系数（因为相机高度是固定的）
        xy_L_new = xy_L_tmp*s_2 + Pc; % 地理坐标系下坐标
        xy_L_pre(:, j) = xy_L_new(1:2, 1); % 保存当前的特征点，用于后面的预测
        lane_feature_data.left_xy_feature(j, :, k_lane) = xy_L_new(1:2, 1);        
        
        uv_R_new = [lane_feature_data.right_uv_feature(j, 1:2, k_lane)'; 1];
        xy_R_tmp = inv(R_IPM)*uv_R_new;
        s_2 = h/(xy_R_tmp(3)); % 缩放系数（因为相机高度是固定的）
        xy_R_new = xy_R_tmp*s_2 + Pc; % 地理坐标系下坐标
        xy_R_pre(:, j) = xy_R_new(1:2, 1); % 保存当前的特征点，用于后面的预测
        lane_feature_data.right_xy_feature(j, :, k_lane) = xy_R_new(1:2, 1);
    end    
    % 参数辨识
    Y(:, 1) = lane_feature_data.left_xy_feature(:, 2, k_lane)';
    X_t(:, 1) = lane_feature_data.left_xy_feature(:, 1, k_lane)';
    I_8 = ones(8,1);
    X = [X_t.^2, X_t, I_8];
    line_p_L(:, 1) = inv(X'*X)*X'*Y;   
    
    Y(:, 1) = lane_feature_data.right_xy_feature(:, 2, k_lane)';
    X_t(:, 1) = lane_feature_data.right_xy_feature(:, 1, k_lane)';
    I_8 = ones(8,1);
    X = [X_t.^2, X_t, I_8];
    line_p_R(:, 1) = inv(X'*X)*X'*Y;      
            

    %% 保存转弯半径数据
    if SAVE_R
        % 写入txt
        fprintf(fp, '%d %f %f %f ', k, R_current_imu_speed );
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
%         [ CC_rgb ] = fun_plot_lane( R_current_imu_speed, CC_rgb, rgb_value_t, camera_parameter);
        % 摄像头光轴
%         rgb_value_t = 40; % 0:黑色
%         [ CC_rgb ] = fun_plot_lane( 0, CC_rgb, rgb_value_t, camera_parameter);
        
        % 车道线 白[0 0 0]' 黑[10 50 50]'  红[238,60,10] 绿[30, 255, 30] 蓝[0 0 255]'
        [ CC_rgb ] = fun_plot_curve( line_p_L, CC_rgb, [10 50 50]', camera_parameter ); % 真实的：黑
        [ CC_rgb ] = fun_plot_curve( line_p_L_predict, CC_rgb, [30, 255, 30], camera_parameter ); % 预测的：绿
        if is_FirstTimeLoop ~= 1
            [ CC_rgb ] = fun_plot_curve( line_p_L_pre, CC_rgb, [238,60,10], camera_parameter );  % 之前的：红     
        end
        
        [ CC_rgb ] = fun_plot_curve( line_p_R, CC_rgb, [10 50 50]', camera_parameter ); % 真实的：黑
        [ CC_rgb ] = fun_plot_curve( line_p_R_predict, CC_rgb, [30, 255, 30], camera_parameter ); % 预测的：绿
        if is_FirstTimeLoop ~= 1
            [ CC_rgb ] = fun_plot_curve( line_p_R_pre, CC_rgb, [238,60,10], camera_parameter );  % 之前的：红     
        end
        
        figure(1);
        str_name = sprintf('frame%d俯视图',k);
        title(str_name); 
        imshow(CC_rgb);        
        
        % 保存ipm图
%         str_name = sprintf('data/0902_R/ipm_save/%d_IPM.png',k);
%         str_name = sprintf('ipm/%d.jpg',k);
%         imwrite(CC1,str_name)

%          CC1(:,:) = 0; % 用于刷新图
        %% save data
    %     angle_M_est =  atan(line_p_est(1,2));
    %     angle_M_camera =  atan(line_p(1,2));
    %     tracking_error(1, loopIndex) = (angle_M_est - angle_M_camera)*180/pi;
    %     tracking_error(2, loopIndex) = cos(angle_M_est)*line_p_est(2,2) - cos(angle_M_camera)*line_p(2,2);
        is_FirstTimeLoop = 0;
        dyaw = 0;
    end
end
% % tracking error
% figure()
% subplot(2,1,1)
% plot(tracking_error(1,:));
% hold on;
% plot(save_lane_change_M(1, :)); % lane本身相对车的运动
% grid on;
% legend('tracking-angle-error', 'lane-angle-change');
% 
% subplot(2,1,2)
% plot(tracking_error(2,:));
% hold on;
% plot(save_lane_change_M(2, :)); % lane本身相对车的运动
% grid on;
% legend('tracking-angle-offset-error', 'lane-offset-change');

% figure()
% plot(-save_d_angle_line_M);
% hold on;
% grid on;
% plot(save_dyaw_degree);
% plot(save_fai_pre);
% legend('d-angle-line-M', 'dyaw-degree', 'fai-pre');
% 
% figure()
% plot(-save_angle_line_M);
% hold on;
% grid on;
% plot(save_yaw_degree);
% plot(save_fai);
% legend('angle-line-M', 'yaw-degree', 'fai');





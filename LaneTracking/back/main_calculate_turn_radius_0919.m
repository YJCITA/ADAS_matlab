% 2016.09.19：利用speed + IMU计算转弯半径
clc;
clear all; 
close all

SAVE_R = 0; % 保存R
SHOW_IPM = 1; % 是否用于IPM显示转弯半径 

%% 转弯半径 测试 
% R_VNw_speed R_steer R_VN300 time_start
load data/0902_R/R_data_new.mat;
% 相机时间
raw_data_camera_log = importdata('data/0902_R/0912_VN300-camera_155524.ini')';
% raw_data_camera_log = importdata('data/0902_R/0912_VN300-camera_160025.ini')';
% raw_data_camera_log = importdata('data/0902_R/0912_VN300-camera_160526.ini')';
% raw_data_camera_log = importdata('data/0902_R/0912_VN300-camera_161027.ini')';
time_s = raw_data_camera_log(1, :);
time_us = raw_data_camera_log(2, :);
time_camera_log = time_s + time_us *1e-6 - time_start;
data_camera_log = [time_camera_log; raw_data_camera_log(3, :)];

%% 初始化参数
m = 720; % v (height)
n = 1280; % u (width)
h = 1.22; % Distance camera was above the ground (meters)
d = 0;	% 横向偏移
l = 0;  % 纵向偏移
theta0 = 1.8*pi/180; % 2.373; 
gama0 = 0; % 水平倾角

% Y1 相机内参
fx = 1506.64297;
fy = 1504.18761;
cx = 664.30351;
cy = 340.94998;
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

%俯视图 参数
y_max = 10; % 横向
y_min = -10;
x_max = 70; % 纵向
x_min = 1; % 摄像头pitch向上，导致近距离看不见。
zoom = 20;
H1 = 400;W1=250;  %需要显示图像的高和宽

%% 汽车参数
L = 2.637;
L_r = L/2;
K_s2w = 0.07; %0.0752;% 0.059; % 方向盘转角->前轮转角
fai = 0;
x_CAN = 0;
y_CAN = 0;

fai_n = 0;
x_CAN_n = 0;
y_CAN_n = 0;

% 写入txt
if SAVE_R
    fp = fopen('F:\Develop\ADAS\Code\matlab\LaneTracking_0803\data\0902_R\car_move_radius_frame_160025.ini', 'wt');
end    
%% 主循环
R_index = 0;
is_R_Camera_matched = 0; % R和图片时间是否匹配
% 1900:5:3700
% 4600:5:7000
k_R = 70;
for k = 3000:2:7734
    %% 匹配R和图片的时间
    is_R_Camera_matched = 0;
    while ~is_R_Camera_matched
        R_index = R_index + 1;
        dt_now = time_camera_log(k) - R_index/80; % R经过80hz的重采样
        dt_next = time_camera_log(k) - (R_index+1)/80;
        if dt_now>0 && dt_next<=0
            R_current_VNw_speed = R_VNw_speed(R_index);
            R_current_VN300 = R_VN300(R_index);
            R_current_steer = R_steer(R_index);            
            is_R_Camera_matched = 1;
        end            
    end
%  k
%% 保存数据
if SAVE_R
    % 写入txt
    fprintf(fp, '%d %f %f %f ', k, R_current_VNw_speed, R_current_VN300, R_current_steer);
    fprintf(fp, ' \n');
end
   
if SHOW_IPM
    %% 图片 IPM      
        % 读取图片数据
        str_data = sprintf('data/0902_R/frame_155524/%08d.jpg',k);
%         str_data = sprintf('data/0902_R/frame_160025/%08d.jpg',k);
%         str_data = sprintf('data/0902_R/frame_160526/%08d.jpg',k);
        I = imread(str_data);
        I_g = rgb2gray(I);
        [m, n] = size(I_g);
        CC = I_g;    

        % 遍历设定的俯视图中的所有像素，线转为地理坐标系XYZ，再通过小孔成像模型映射到相机图像中的uv
        % 这样可以保证俯视图中所有的像素都会有对应的相机成像点
        R_w2i =  M1*Rc12c*Ratt*[I3 -Pc];
        for M = 1:H1               %变换之后路面图像H1*W1=400*600个像素，路面宽度为7m，高度为10m
            x = -(M*x_max/H1 - x_max);
            for N=1:W1            
                y = N*2*y_max/W1 - y_max; 
                if x<x_max && x>x_min && y>y_min && y<y_max
                    % 投影变化，相机成像模型
                    Point_xyz = [x, y, 0, 1]';
                    uv_tmp = R_w2i*Point_xyz;
                    uv_new = uv_tmp/(uv_tmp(3));% z方向深度归一化
                    u = round(uv_new(1));
                    v = round(uv_new(2));                
                    if u>0.5 && v>0.5 && u<n && v<m
                        CC1(M, N) = I_g(v,u);
                    end  
                end
            end
        end     
        CC1 = medfilt2(CC1,[2,2]);% 中值滤波

        %% 画弯道
        R_current = R_current_VN300; %R_current_VNw_speed; %R_current_VN300;
        if R_current ~=0
            j = 0;  
            R_current_abs = abs(R_current);
            x_max_t = min([x_max, R_current_abs]);
            for x_line = x_min:0.1:x_max_t
                if R_current>0
                    y_line = R_current - sqrt(R_current^2 - x_line^2);
                else
                    y_line = R_current + sqrt(R_current^2 - x_line^2);
                end
                M = (-x_line + x_max)*H1/x_max; % 减少重复计算
                if x_line<x_max_t && x_line>x_min && y_line>y_min && y_line<y_max  
                    N = (y_line + y_max)*W1/(2*y_max);
                    u = round(N);
                    v = round(M);
                    if u>0.5 && v>0.5 && u<W1 && v<H1
                         CC1(v, u) = 220;
                         CC1(v+1, u) = CC1(v, u);
                         CC1(v, u+1) = CC1(v, u);
                         CC1(v+1, u+1) = CC1(v, u);
                         j = j+1;
                        line_pre(:, j) = [u, v]';
                        line_R_XY(:, j) = [x_line, y_line]';
                    end  
                end
            end
        end

        %%% R_current_steer
        %%% 画弯道
%         k_R = k_R - 1
        R_current = R_current_steer; % R_current_VN300; %R_current_steer;
        if R_current ~=0
            j = 0;    
            R_current_abs = abs(R_current);
            x_max_t = min([x_max, R_current_abs]);
            for x_line = x_min:0.1:x_max_t
                if R_current>0
                    y_line = R_current - sqrt(R_current^2 - x_line^2);
                else
                    y_line = R_current + sqrt(R_current^2 - x_line^2);
                end          
                M = (-x_line + x_max)*H1/x_max; % 减少重复计算
                if x_line<x_max_t && x_line>x_min && y_line>y_min && y_line<y_max  
                    N = (y_line + y_max)*W1/(2*y_max);
                    u = round(N);
                    v = round(M);
                    if u>0.5 && v>0.5 && u<W1 && v<H1
                         CC1(v, u) = 180; % 0:黑色
                         CC1(v+1, u) = CC1(v, u);
                         CC1(v, u+1) = CC1(v, u);
                         CC1(v+1, u+1) = CC1(v, u);
                         j = j+1;
                        line_pre(:, j) = [u, v]';
                        line_R_XY(:, j) = [x_line, y_line]';
                    end  
                end
            end
        end

        %%% 摄像头光轴
        x_max_t = x_max;
        for x_line = x_min:0.1:x_max_t
            y_line = 0;            
            M = (-x_line + x_max)*H1/x_max; % 减少重复计算
            if x_line<x_max_t && x_line>x_min && y_line>y_min && y_line<y_max  
                N = (y_line + y_max)*W1/(2*y_max);
                u = round(N);
                v = round(M);
                if u>0.5 && v>0.5 && u<W1 && v<H1
                     CC1(v, u) = 70; % 0:黑色
                     CC1(v+1, u) = CC1(v, u);
                     CC1(v, u+1) = CC1(v, u);
                     CC1(v+1, u+1) = CC1(v, u);
                end  
            end
        end
        
        figure(1);
        imshow(CC1);
        str_name = sprintf('frame%d俯视图',k);
        title(str_name); 
        
        % 保存ipm图
%         str_name = sprintf('data/0902_R/ipm_save/%d_IPM.png',k);
%         str_name = sprintf('ipm/%d.jpg',k);
%         imwrite(CC1,str_name)

         CC1(:,:) = 0; % 用于刷新图
        %% save data
    %     angle_M_est =  atan(line_p_est(1,2));
    %     angle_M_camera =  atan(line_p(1,2));
    %     tracking_error(1, loopIndex) = (angle_M_est - angle_M_camera)*180/pi;
    %     tracking_error(2, loopIndex) = cos(angle_M_est)*line_p_est(2,2) - cos(angle_M_camera)*line_p(2,2);
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


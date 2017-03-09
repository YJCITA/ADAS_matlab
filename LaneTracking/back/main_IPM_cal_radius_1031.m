%% 采用连续读取log原始数据的方式计算R
% 南京的数据
clc 
clear all
close all

SAVE_R = 1;  % 存储R半径在文本中
SHOW_IPM = 0; % 显示IPM图

%% 数据导入
origin_address = ['data/nj/1111_R_error/'];
origin_name = 'log';
address_log = [origin_address, origin_name, '.txt'];
fid = fopen(address_log,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20161016_102025';
ipm_index = 500; % 从哪一帧图片开始ipm
ipm_step = 4; % 步长

% imu数据
w_drift = [0, 0, -0.0204]';

%% 保存数据
% 写入txt
save_R_addr = ['./', origin_address, origin_name, '-car_turn_radius.ini'];
if SAVE_R
     fp = fopen(save_R_addr, 'wt');
end  

%% 初始化参数
camera_parameter.m= 720; % v (height)
camera_parameter.n = 1280; % u (width)
camera_parameter.yaw = 7.3*pi/180; % (我定义的是NE,yaw右为正，nj式左为正)
camera_parameter.pitch = 0.3*pi/180; % 2.373; 
camera_parameter.roll = 0; % 水平倾角

camera_parameter.h = 1.25; %1.2; % Distance camera was above the ground (meters)
camera_parameter.dl = 0.05; % 横向偏移 向右为正
camera_parameter.d = 0;

camera_parameter.Pc =  [camera_parameter.dl 0 -camera_parameter.h]'; % 地理坐标系下相机中心坐标点  ;
fx = 1482.0; %1506.64297;
fy = 1475.874; %1504.18761;
cx = 685.044; %664.30351;
cy = 360.02380; %340.94998;
camera_parameter.M1 = [ fx  0 cx; 
                        0  fy cy; 
                        0  0  1 ];
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

gyro_fliter = [0 0 0]';
k_camera = 0;
k_imu = 0;
line_index_t = 0; % 用来调试，看当前读到第几行
while ~feof(fid)
    line_index_t = line_index_t+1;

    %% 匹配R和图片的时间
    is_R_Camera_matched = 0;
    while ~is_R_Camera_matched
        % 到文件末尾就退出
        if feof(fid)
           break;
        end
        
        lineData = fgetl(fid);
        str_line_raw = regexp(lineData,' ','split'); %以空格为特征分割字符串
        time_s = str2num(str_line_raw{1,1});
        time_us = str2num(str_line_raw{1,2});
        time = time_s + time_us *1e-6;
        str_line_data_flag = str_line_raw(3);
        % Gsensor
        if  strcmp(str_line_data_flag, 'Gsensor')
            for i = 1:6
                imu_data_t(i, 1) = str2num(str_line_raw{1, i+3});
            end
            data_gensor_raw = [time; imu_data_t]; 
            k_imu = k_imu + 1;
            data_imu = fun_imu_data_trans( data_gensor_raw );
            data_imu_save(:, k_imu) = data_imu;

            gyro_new = data_imu(5:7) - w_drift;
            gyro_fliter = funLowpassFilterVector3f( gyro_fliter, gyro_new, 0.01, 1 );
            gyro_fliter_save(:, k_imu) = gyro_fliter;

        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6;        
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % 获取数据
            t_s = str_line_raw{1, 1};
            t_us = str_line_raw{1, 2};
            R_image_file_name = str_line_raw{1, 4}; % mp4文件路径
            length_imege_name = length(R_image_file_name);
            R_mp4_file_name_cur_search = R_image_file_name(length_imege_name-22:length_imege_name-4); 
            R_image_index_str = str_line_raw{1, 5};
            R_image_index_num = str2num(R_image_index_str) + 1; % log中图像index编号是从0开始

            % 计算半径
            if abs(gyro_fliter(3)) > 0.01 && speed_cur>15/3.6  
                R_imu_speed_cur = speed_cur/gyro_fliter(3); 
            else
                R_imu_speed_cur = 0;      
            end
            k_camera = k_camera + 1;
            R_imu_speed_save(k_camera) = R_imu_speed_cur;
%             R_imu_speed_cur

            if SAVE_R  % 写入txt                    
                fprintf(fp, '%s %s %s %s %f ', t_s, t_us, R_image_file_name, R_image_index_str, R_imu_speed_cur );
                fprintf(fp, ' \n');
            end

             % 比对当前图像的时间戳
            if SHOW_IPM
                if strcmp(R_mp4_file_name_cur_search, ipm_image_file_name) && ipm_index == R_image_index_num                    
                    ipm_index
                    R_imu_speed_cur
                  %% 图片 IPM      
                    % 读取图片数据
                    image_name = sprintf('/%08d.jpg',ipm_index);
                    str_data = [origin_address, ipm_image_file_name, image_name];
                    I_rgb = imread(str_data);
                    I_g = rgb2gray(I_rgb);
                    [m, n] = size(I_g);
                    % IPM变换
                    [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
                    CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
                    CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
                    CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波
                 %% 画线
                    rgb_value_t = 240; % 0:黑色
                    [ CC_rgb ] = fun_plot_R( R_imu_speed_cur, CC_rgb, rgb_value_t, camera_parameter);                        
%                     figure(2);
%                     imshow(I_rgb);  
                    
                    figure(1);
                    str_name = sprintf('frame%d俯视图', ipm_index);
                    title(str_name); 
                    imshow(CC_rgb); 

                    ipm_index = ipm_index + ipm_step;
                    is_R_Camera_matched = 1;
                end
            end
        end
    end
end
fclose(fid);
if SAVE_R
     fclose(fp);
end 

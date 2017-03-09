% 测试：利用陀螺仪计算角度并估计地面角度，进而计算车辆相对路面的角度（尤其是对于颠簸的时候）     
clear; 
clc;
close all
%% 数据导入
origin_address = ['./data/att_image/0216_VN300/'];
origin_name = 'log-rec_20160924_153548';
address_log = [origin_address, origin_name, '.txt'];
fid_log = fopen(address_log,'r');

% 需要进行ipm显示的image文件夹名
ipm_image_file_name = 'rec_20160924_153548';
ipm_index = 300; % 从哪一帧图片开始ipm
ipm_step = 2; % 步长

% imu数据
w_drift = [0, -2.83/180*pi, 0]';  % 通过跟对比

%% 控制参数
SHOW_IPM = 1; % 显示IPM图

%% 初始化参数
camera_parameter.m= 720; % v (height)
camera_parameter.n = 1280; % u (width)
camera_parameter.yaw = 0*pi/180; % (我定义的是NE,yaw右为正，nj式左为正)
pitch_origin = 0.5;
camera_parameter.pitch =  pitch_origin*pi/180; % 2.373; 
camera_parameter.roll = 0; % 水平倾角

camera_parameter.h = 1.3; %1.2; % Distance camera was above the ground (meters)
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
k_vn = 0;
line_index_t = 0; % 用来调试，看当前读到第几行

%% 读取数据 连续处理多帧图片
road_lean_fliter = [0 0 0]'; % 低通滤波估计道路坡度
dt = 0.01;
is_first_imu_data = 1;
vn_att_pre = [0 0 0]';

att_image_pitch_pre = 0;

is_first_vn_data = 1;
road_lean_fliter_vn = [0 0 0]';

is_first_image = 1;
pitch_new = 0;
while (1)
      %% 图片 IPM      
        % 读取图片数据
        image_name = sprintf('/%08d.jpg',300);
        str_data = [origin_address, ipm_image_file_name, image_name];
        I_rgb = imread(str_data);
        I_g = rgb2gray(I_rgb);
        [m, n] = size(I_g);
        
        pitch_new = pitch_new + 0.2/180*pi
        camera_parameter.pitch = pitch_new;
        % IPM变换
        [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter );   
        CC_rgb(:,:, 1) = medfilt2(CC_rgb(:,:, 1),[2,2]);% 中值滤波
        CC_rgb(:,:, 2) = medfilt2(CC_rgb(:,:, 2),[2,2]);% 中值滤波
        CC_rgb(:,:, 3) = medfilt2(CC_rgb(:,:, 3),[2,2]);% 中值滤波

        figure();
        str_name = sprintf('frame%d俯视图', ipm_index);
        title(str_name); 
        imshow(CC_rgb); 
        tt =1;
    
end
fclose(fid_log);
if SAVE_R
     fclose(fp);
end 

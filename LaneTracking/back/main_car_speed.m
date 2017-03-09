clear; 
clc;
% close all
%% 载入数据
dada_steer1 = load('data/0801_tracking/1/log-steer.ini');
str_name = 'data/0801_tracking/1/';
address_CAN = [str_name, 'data_CAN_0801.mat'];
load(address_CAN); % data_CAN: time*1 CAN_speed*1(m/s)  steer_angle*1(°) 
CAN_time = data_CAN(1, :);
CAN_speed = data_CAN(2, :);
CAN_steer = data_CAN(3, :);

address_lane = [str_name, 'data_lane_0801.mat'];
load(address_lane); % data_lane: lane index*1, camera_time*1, point*24(3车道，每车道4个点，每点2坐标)
lane_time = data_lane(2, :);
data_uv_feature = data_lane(3:26, :);

address_att_IMU = [str_name, 'data_att_IMU_0801.mat'];
load(address_att_IMU); % data_att_IMU: imu_time*1; att*3(°); acc_cal*3（m/s2）; gyro_cal*3(rad/s)
att_IMU_time = data_att_IMU(1, :);
att = data_att_IMU(2:4, :);
acc_cal = data_att_IMU(5:7, :);
gyro_cal = data_att_IMU(8:10, :);

%% 初始化参数
m = 720; % v (height)
n = 1280; % u (width)
h = 1.3; % Distance camera was above the ground (meters)
d = 0; %横向偏移
l = 0;% 纵向偏移
theta0 = 2*pi/180;%2.373; 
gama0 = 0; % 水平倾角

%% 汽车参数
L = 2.637;
L_r = L/2;
fai = 0;
K_s2w = 0.07; %0.0752;% 0.059; % 方向盘转角->前轮转角
x_CAN = 0;
y_CAN = 0;

NUM_CAN = length(data_CAN);
current_CAN_index = 0;
is_CAN_match_lane = 0; % 这一次CAN数据和lane数据是否已经匹配上

is_att_match_lane = 0; % 这一次att和lane数据是否已经匹配上
pre_att = [0 0 0]'; % 上一次匹配的att
current_att_index = 0;
is_FirstTime_att = 1;

match_time_counter = 0;

%% 车子运动轨迹解算
    for i = 110:370
        current_CAN_index = i;
        
        if current_CAN_index <= 1 % 第一次进入
%             current_CAN_index = current_CAN_index + 1;
            d_alpha = 0;
            c = 0;
            beta = 0;
            d_alpha1 = 0;
        else               
            speed_new = CAN_speed(1, current_CAN_index);  
            steer_new = CAN_steer(1, current_CAN_index);
            dt_CAN = CAN_time(current_CAN_index) - CAN_time(current_CAN_index-1);

            if abs(steer_new)>1 % 方向盘小角度不计算
                wheel_angle = (K_s2w*steer_new)*(pi/180); % 前轮角度 弧度                   
                beta = atan(L_r*tan(wheel_angle)/L);  % 计算 beta
                d_fai = speed_new/L*tan(wheel_angle)*cos(beta)*dt_CAN;   
            else
                wheel_angle = 0;
                d_alpha = 0;
                d_fai = 0;
                c = speed_new*dt_CAN;
            end
            fai = fai + d_fai;
            Vx = speed_new*cos(fai + beta);
            Vy = speed_new*sin(fai + beta);
            x_CAN = x_CAN + Vx*dt_CAN;
            y_CAN = y_CAN + Vy*dt_CAN;                
            XY_CAN(:,current_CAN_index) = [x_CAN, y_CAN]'; 
            log_steer_speed(:,current_CAN_index) = [steer_new, speed_new]';
            save_fai(:,current_CAN_index) = fai*180/pi;
        end 
    end
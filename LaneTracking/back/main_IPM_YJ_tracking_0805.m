% 0804:利用0801采集的IMU、CAN、IMU、Camera数据，对图像进行车道线人工标注后，进行lane tracking仿真分析
% 0805：实现了车道线的tracking,目前初步精度还可以。就是对于(0,0)样本点这种异常处理还待处理

clear; 
clc;
% close all
%% 载入数据
str_name = 'data/0801_tracking/';
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
d = 0;	% 横向偏移
l = 0;  % 纵向偏移
theta0 = 1.8*pi/180;%2.373; 
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
y_max = 7; % 横向
y_min = -7;
x_max = 50; % 纵向
x_min = 6; % 摄像头pitch向上，导致近距离看不见。
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

NUM_CAN = length(data_CAN);
current_CAN_index = 0;
is_CAN_match_lane = 0; % 这一次CAN数据和lane数据是否已经匹配上
is_att_match_lane = 0; % 这一次att和lane数据是否已经匹配上
pre_att = [0 0 0]'; % 上一次匹配的att
current_att_index = 0;
is_FirstTime_att = 1;
is_FirstTimeLoop = 1; % 第一次进入lane预测
loopIndex = 0;

 %% 主循环
% 6: 6330:10:6600
% 5: 5800:10:6070
% 1: 40:10:170
for k = 1:10:170
    % 新循环
    loopIndex = loopIndex + 1;
    is_CAN_match_lane = 0;
    is_att_match_lane = 0;

    % 保存上一帧车道线的特征点，用于预测
    if is_FirstTimeLoop == 1
        data_uv_feature_pre = data_uv_feature(:, k);
    else
        data_uv_feature_pre = data_uv_feature_new;
    end
    data_uv_feature_new = data_uv_feature(:, k);

    %% 车子运动轨迹解算
    XY_CAN_pre_n = [x_CAN_n, y_CAN_n]';
    fai_pre_n  = fai_n;
    
          while is_CAN_match_lane == 0
            current_CAN_index = current_CAN_index + 1;        
            if current_CAN_index <= 1 % 第一次进入
                current_CAN_index = current_CAN_index + 1;
                d_alpha = 0;
                c = 0;
                beta = 0;
                d_alpha1 = 0;
            else               
                speed_new = CAN_speed(1, current_CAN_index);  
                steer_new = CAN_steer(1, current_CAN_index);
                dt_CAN = CAN_time(current_CAN_index) - CAN_time(current_CAN_index-1);

                if abs(steer_new)>5 % 方向盘小角度不计算
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
                
                % new 不清零
                fai_n = fai_n + d_fai;
                Vx_n = speed_new*cos(fai_n + beta);
                Vy_n = speed_new*sin(fai_n + beta);
                x_CAN_n = x_CAN_n + Vx_n*dt_CAN;
                y_CAN_n = y_CAN_n + Vy_n*dt_CAN;  
    %             save_fai(:,current_CAN_index) = fai*180/pi;
            end            

            % 判断speed时间戳，取最靠近此时steer的时间戳
            dt_CAN_lane_C = abs(CAN_time(current_CAN_index) - lane_time(k));
            dt_CAN_lane_N = abs(CAN_time(current_CAN_index+1) - lane_time(k));

            if dt_CAN_lane_C < dt_CAN_lane_N && dt_CAN_lane_C<0.5   
                is_CAN_match_lane = 1; % 这一次CAN数据和lane数据 已经匹配
%                 fai_pre  = fai*180/pi;
        
            end           
        end

    %% att 匹配
    while is_att_match_lane == 0
        current_att_index = current_att_index + 1;
        % 判断speed时间戳，取最靠近此时steer的时间戳
        dt_C = abs(att_IMU_time(current_att_index) - lane_time(k));
        dt_N = abs(att_IMU_time(current_att_index+1) - lane_time(k));
        if dt_C < dt_N && dt_C<0.1  
            is_att_match_lane = 1;
            if is_FirstTime_att == 1            
                pre_att = att(:, current_att_index);
                current_att = att(:, current_att_index);
                is_FirstTime_att = 0;
            else             
                pre_att = current_att;
                current_att = att(:, current_att_index);
            end
        end
        save_att(:,current_att_index) = att(:, current_att_index);
        att(3, current_att_index)
    end

    % %% 当前帧的车道线预测
    % % IPM后的手工标注特征点在图像中的坐标
    % dyaw_degree = current_att(3) - pre_att(3);
    % save_dyaw_degree(1, loopIndex) = dyaw_degree;
    % save_yaw_degree(1, loopIndex) =  current_att(3);
    % save_fai_pre(1, loopIndex) = fai_pre;
    % save_fai(1, loopIndex) = fai*180/pi;
    % dyaw = dyaw_degree/180*pi;
    % Rn2c_kT = [ cos(dyaw) sin(dyaw);
    %             -sin(dyaw) cos(dyaw)];           
    % P_kT = [x_CAN, y_CAN]';
    % % x_CAN = 0;
    % % y_CAN = 0;
    % % P_kT = [0 0]';
    % 
    % % 特征点IPM到世界坐标系上
    % for j = 1:12
    %     % 矩阵式逆投影变化
    %     uv_pre = [data_uv_feature_pre([j*2-1:j*2]); 1];
    %     xy_pre = inv(R_IPM)*uv_pre;
    %     s_2 = h/(xy_pre(3)); % 缩放系数（因为相机高度是固定的）
    %     xy_pre = xy_pre*s_2 + Pc; % 地理坐标系下坐标
    %     Xk_pre(1:2, j) = xy_pre(1:2);
    %     XK_new(1:2, j) = Rn2c_kT*(xy_pre(1:2) - P_kT); % 预测车道线在当前帧的坐标
    % end
    % 
    % % 预测当前帧当中车道线的参数
    % for i = 1:3
    %     Y = XK_new(2, (i*4-3):i*4)';
    %     X = [XK_new(1, (i*4-3):i*4)' , [1 1 1 1]'];
    %     line_p_est(:, i) = inv(X'*X)*X'*Y;   
    % end 

%% 当前图像中的直线车道拟合
    % IPM后的手工标注特征点在图像中的坐标
    for j = 1:12
        % 矩阵式逆投影变化
        uv_new = [data_uv_feature_new([j*2-1:j*2]); 1];
        xy_tmp = inv(R_IPM)*uv_new;
        s_2 = h/(xy_tmp(3)); % 缩放系数（因为相机高度是固定的）
        xy_new = xy_tmp*s_2 + Pc; % 地理坐标系下坐标
        xy_feature(1:2, j) = xy_new(1:2, 1);
    end
%%% 角度显示   调试
%     if is_FirstTimeLoop == 0
%         tan_est_angle = atand(line_p_est(1,2)) % 预测到曲线角度
%         angle_middle_pre_camera = atand(line_p(1,2)) % 上一帧中间车道线的图像标注角度
%     end    
    
    % 参数辨识
    for i = 1:3
        Y = xy_feature(2, (i*4-3):i*4)';
        X = [xy_feature(1, (i*4-3):i*4)' , [1 1 1 1]'];
        line_p(:, i) = inv(X'*X)*X'*Y;   
    end 
    
%%% 角度显示 调试
%     angle_middle_camera = atand(line_p(1,2)) % 中间车道线的图像标注角度
    
    if is_FirstTimeLoop == 1
        line_p_pre = line_p;
        d_angle_line_M = 0;
        x_CAN = 0;
        y_CAN = 0;
    
    else
        angle_line_M_current = atan(line_p(1,2));
        angle_line_M_pre = atan(line_p_pre(1,2));
        d_angle_line_M = angle_line_M_current - angle_line_M_pre;
        save_lane_change_M(1, loopIndex) = d_angle_line_M*180/pi;
        save_lane_change_M(2, loopIndex) = cos(angle_line_M_current)*line_p(2,2) - cos(angle_line_M_pre)*line_p_pre(2,2);
      
        line_p_pre = line_p;
    end
    save_angle_line_M(1, loopIndex) = atan(line_p(1,2))*180/pi;

     %% 当前帧的车道线预测
    % IPM后的手工标注特征点在图像中的坐标
    dyaw_degree = current_att(3) - pre_att(3);
    save_dyaw_degree(1, loopIndex) = dyaw_degree;
    save_yaw_degree(1, loopIndex) =  current_att(3);
    
    save_fai(1, loopIndex) = fai*180/pi;
    % dyaw = -d_angle_line_M/180*pi;
    % dyaw = fai_pre/180*pi;
    dyaw = dyaw_degree/180*pi;
    Rn2c_kT = [ cos(dyaw) sin(dyaw);
                -sin(dyaw) cos(dyaw)];
         
  % CAN 计算过程不清零（fai要转换到pre时刻的坐标系下）
    XY_CAN_cur_n = [x_CAN_n, y_CAN_n]';
    P_kT_t = XY_CAN_cur_n - XY_CAN_pre_n;
    dT_fai = fai_pre_n;
    dT_fai_deg = dT_fai*180/pi;
    R_dfai = [ cos(dT_fai) sin(dT_fai);
                -sin(dT_fai) cos(dT_fai)]; 
    P_kT_n = R_dfai*P_kT_t;
    
    P_kT =  P_kT_n;
%     x_CAN = 0;
%     y_CAN = 0;
    % 特征点IPM到世界坐标系上
    for j = 1:12
        % 矩阵式逆投影变化
        uv_pre = [data_uv_feature_pre([j*2-1:j*2]); 1];
        xy_pre = inv(R_IPM)*uv_pre;
        s_2 = h/(xy_pre(3)); % 缩放系数（因为相机高度是固定的）
        xy_pre = xy_pre*s_2 + Pc; % 地理坐标系下坐标
        Xk_pre(1:2, j) = xy_pre(1:2);
        XK_new(1:2, j) = Rn2c_kT*(xy_pre(1:2) - P_kT); % 预测车道线在当前帧的坐标
    end

    % 预测当前帧当中车道线的参数
    for i = 1:3
        Y = XK_new(2, (i*4-3):i*4)';
        X = [XK_new(1, (i*4-3):i*4)' , [1 1 1 1]'];
        line_p_est(:, i) = inv(X'*X)*X'*Y;   
    end 
    
 %% 图片 IPM      
    % 读取图片数据
    str_data = sprintf('data/0801_tracking/jpg/%d.jpg',k);
    I = imread(str_data);
    I_g = rgb2gray(I);
    [m n] = size(I_g);
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

%% 画车道线
    % 上一帧原始车道线
    if is_FirstTimeLoop == 0        
        NUM = length(line_pre);
        for i = 1:NUM
            u = line_pre(1, i);
            v = line_pre(2, i);
             if u>0.5 && v>0.5 && u<W1 && v<H1
                CC1(v, u) = 150; % 
            end  
        end
    end    

    j = 0;
    for i = 2 %1:3 % 3条车道线              
        for x_line = x_min:0.1:x_max
            y_line = [x_line, 1]*line_p(:,i);            
            M = (-x_line + x_max)*H1/x_max; % 减少重复计算
            if x_line<x_max && x_line>x_min && y_line>y_min && y_line<y_max  
                N = (y_line + y_max)*W1/(2*y_max);
                u = round(N);
                v = round(M);
                if u>0.5 && v>0.5 && u<W1 && v<H1
                    CC1(v, u) = 220;
                     j = j+1;
                    line_pre(:, j) = [u, v]';
                end  
            end
        end
    end
    
    % 预测的当前帧车道线
    for i = 2 %1:3 % 3条车道线              
        for x_line = x_min:0.1:x_max
            y_line = [x_line, 1]*line_p_est(:,i);            
            M = (-x_line + x_max)*H1/x_max; % 减少重复计算
            if x_line<x_max && x_line>x_min && y_line>y_min && y_line<y_max    
                N = (y_line + y_max)*W1/(2*y_max);
                u = round(N);
                v = round(M);
                 if u>0.5 && v>0.5 && u<W1 && v<H1
                    CC1(v, u) = 0; % 预测的曲线涂成黑色
                end  
            end
        end
    end 
        
    figure(1);
    imshow(CC1);
    str_name = sprintf('frame%d俯视图',k);
    title(str_name);        
    CC1(:,:) = 0; % 用于刷新图
    
    is_FirstTimeLoop = 0;   
    
    %% save data
    angle_M_est =  atan(line_p_est(1,2));
    angle_M_camera =  atan(line_p(1,2));
    tracking_error(1, loopIndex) = (angle_M_est - angle_M_camera)*180/pi;
    tracking_error(2, loopIndex) = cos(angle_M_est)*line_p_est(2,2) - cos(angle_M_camera)*line_p(2,2);
 
end
% tracking error
figure()
subplot(2,1,1)
plot(tracking_error(1,:));
hold on;
plot(save_lane_change_M(1, :)); % lane本身相对车的运动
grid on;
legend('tracking-angle-error', 'lane-angle-change');

subplot(2,1,2)
plot(tracking_error(2,:));
hold on;
plot(save_lane_change_M(2, :)); % lane本身相对车的运动
grid on;
legend('tracking-angle-offset-error', 'lane-offset-change');

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


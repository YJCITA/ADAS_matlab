%% 2016.12.29 整合姿态解算，重采样，相对姿态求取，转向灯检测

clc
clear 
close all
%% IMU 数据预处理
log_address = ['./data/1229_拨杆/1228-log-2.txt'];
fid = fopen(log_address,'r');

is_first_time = 1; % 用于计算time_start
is_first_imu_camera = 1; % 是否第一次采集到Gsensor数据，用于初始化
is_first_imu_fmu = 1;% 是否第一次采集到FMU数据，用于初始化
imu_data_camera = zeros(7,1);
acc_fliter_camera = zeros(3,1);
gyro_fliter_camera = zeros(3,1);
imu_data_camera_index = 0; % 用于保存数据的index
imu_data_fmu_index = 0;
turnlamp_data_index = 0;
att_camera = zeros(3,1);
while ~feof(fid)
     % 到文件末尾就退出
    if feof(fid)
        break;
    end
    
    lineData = fgetl(fid);
    str_line_raw = regexp(lineData,' ','split'); %以空格为特征分割字符串    
    raw_time_s = str2num(str_line_raw{1,1});
    raw_time_us = str2num(str_line_raw{1,2});
    if is_first_time % 计算time_start
        time_start = (raw_time_s-10) + raw_time_us*1e-6;
        is_first_time = 0;
    end
    time_s = raw_time_s + raw_time_us*1e-6 - time_start;
    
    str_line_data_flag = str_line_raw{1,3};
    if  strcmp(str_line_data_flag, 'Gsensor') % imu
        for i = 1:6
            imu_data_t(i, 1) = str2num(str_line_raw{1, i+3});
        end
        data_imu = fun_imu_data_trans_simple( imu_data_t );        
        acc_raw = data_imu(1:3);
        gyro_raw = data_imu(4:6);
        
        % 第一次进入循环进行初始化
        if is_first_imu_camera
            is_first_imu_camera = 0;
            acc_fliter_camera = acc_raw; % 低通滤波初值初始化
            gyro_fliter_camera = gyro_raw;
            AccAngle(1) = atan2(-acc_fliter_camera(2), -acc_fliter_camera(3)); % roll
            AccAngle(2) = atan2(acc_fliter_camera(1), sqrt(acc_fliter_camera(2)^2 + acc_fliter_camera(3)^2));
            att0 = [AccAngle(1), AccAngle(2), 0]';
            Q_camera = funEuler2Quat( att0);  % 四元数计算初始化
        end
        
        acc_fliter_camera = funLowpassFilterVector3f( acc_fliter_camera, acc_raw, 0.01, 10 );
        gyro_fliter_camera = funLowpassFilterVector3f( gyro_fliter_camera, gyro_raw, 0.01,10 );
         %互补滤波        
        [att_camera, Q_camera ,AccAngle_G] = funComplementaryFilter_q_G(Q_camera, acc_fliter_camera, gyro_fliter_camera, 0.01 );
        
        imu_data_camera_index = imu_data_camera_index + 1;
        imu_data_camera(:, imu_data_camera_index) = [time_s; acc_fliter_camera; gyro_fliter_camera];
        att_camera_save(:, imu_data_camera_index) = [time_s; att_camera];
    elseif  strcmp(str_line_data_flag, 'FMU') % 
        % 获取数据      
        for i = 1:6
            fmu_data_t(i, 1) = str2num(str_line_raw{1, i+7});
        end
        
        % 坐标系变换
        acc_data_fmu_t = fmu_data_t(1:3, 1)/100; %  m/s^2
        acc_raw(1,1) = acc_data_fmu_t(3,1);
        acc_raw(2,1) = acc_data_fmu_t(1,1);
        acc_raw(3,1) = acc_data_fmu_t(2,1); 
        gyro_data_fmu_t = fmu_data_t(4:6, 1)/10/180*pi; % 。/s
        gyro_raw(1,1) = gyro_data_fmu_t(3,1);
        gyro_raw(2,1) = gyro_data_fmu_t(1,1);
        gyro_raw(3,1) = gyro_data_fmu_t(2,1); 
        
         % 第一次进入循环进行初始化
        if is_first_imu_fmu
            is_first_imu_fmu = 0;
            acc_fliter_fmu = acc_raw; % 低通滤波初值初始化
            gyro_fliter_fmu = gyro_raw;
            AccAngle(1) = atan2(-acc_fliter_fmu(2), -acc_fliter_fmu(3)); % roll
            AccAngle(2) = atan2(acc_fliter_fmu(1), acc_fliter_fmu(2)^2 + acc_fliter_fmu(3)^2);
            att0 = [AccAngle(1), AccAngle(2), 0]';
            Q_fmu = funEuler2Quat( att0);  % 四元数计算初始化
        end
        
%         acc_fliter_fmu = funLowpassFilterVector3f( acc_fliter_fmu, acc_raw, 0.01, 10 );
%         gyro_fliter_fmu = funLowpassFilterVector3f( gyro_fliter_fmu, gyro_raw, 0.01,10 );
         %互补滤波        
        [att_fmu, Q_fmu ,AccAngle_G] = funComplementaryFilter_q_G(Q_fmu, acc_raw, gyro_raw, 11/400 );
        
        imu_data_fmu_index = imu_data_fmu_index + 1;
        imu_data_fmu(:, imu_data_fmu_index) = [time_s; acc_raw; gyro_raw];
        att_fmu_save(:, imu_data_fmu_index) = [time_s; att_fmu];
        %%%% 计算相对姿态
        [ R_camera ] = funAtt2Rnb( att_camera );
        [ R_fmu ] = funAtt2Rnb( att_fmu );
        R_relative = R_fmu*R_camera';
        roll = atan2(R_relative(2,3), R_relative(3,3));
        pitch = asin(-R_relative(1,3));
        yaw = atan2(R_relative(1,2), R_relative(1,1));  
        att_relative(:, imu_data_fmu_index) = [time_s; roll; pitch; yaw];
        
        % 计算cross(ga,gb)
        r11 = cross(acc_fliter_fmu, acc_fliter_camera);
        Rgg(:, imu_data_fmu_index) = [time_s; r11];
        
    elseif strcmp(str_line_data_flag, 'turnlamp')% turnlamp
        % turnlamp 转向灯信号：0 No steering light 1 Left steering light；2 Right steering light";
        turnlamp_t = str2num(str_line_raw{1, 4});
        if turnlamp_t == 2
            turnlamp_t = -1;
        end 
        turnlamp_data_index = turnlamp_data_index + 1;
        turnlamp_data(:, turnlamp_data_index) = [time_s; turnlamp_t];
    end
end

% acc
figure()
subplot(3,1,1)
plot(imu_data_camera(1,:), imu_data_camera(2,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(2,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
grid on;
legend('camera-acc-x','fmu-acc-x', 'turnlamp')

subplot(3,1,2)
plot(imu_data_camera(1,:), imu_data_camera(3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
grid on;
legend('camera-acc-y','fmu-acc-y', 'turnlamp')

subplot(3,1,3)
plot(imu_data_camera(1,:), imu_data_camera(4,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(4,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
grid on;
legend('camera-acc-z','fmu-acc-z', 'turnlamp')

% gyro
figure()
subplot(3,1,1)
plot(imu_data_camera(1,:), imu_data_camera(2+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(2+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-x','fmu-gyro-x', 'turnlamp')

subplot(3,1,2)
plot(imu_data_camera(1,:), imu_data_camera(3+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(3+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-y','fmu-gyro-y', 'turnlamp')

subplot(3,1,3)
plot(imu_data_camera(1,:), imu_data_camera(4+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(4+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-z','fmu-gyro-z', 'turnlamp')

% % 拨杆和camera相对姿态
% figure()
% subplot(3,1,1)
% plot(att_relative(1,:), att_relative(2,:));
% hold on;
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('relative-roll', 'turnlamp')
% title('相对姿态')
% 
% subplot(3,1,2)
% plot(att_relative(1,:), att_relative(3,:));
% hold on;
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('relative-pitch', 'turnlamp')
% 
% subplot(3,1,3)
% plot(att_relative(1,:), att_relative(4,:));
% hold on;
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('relative-yaw', 'turnlamp')


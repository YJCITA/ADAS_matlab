clc;
clear 
close all

one_G = 9.80665;
A0 = [ -0.0056, -0.0080, -0.0433]';
A1 = [0.9992   -0.0020    0.0063
     -0.0009    0.9999   -0.0114
      0.0197   -0.0016    1.0090];
A2 = [ 0.0041    0       0;
       0     -0.0012     0;
       0         0    0.0362];  

%% 获取数据
data = importdata('data_log_gsensor_1.txt');
% data = importdata('静态测试数据_4号模组.txt');

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

for i = 1:3
    w_drift(i) = mean(gyro_cal(i,100:250)); 
    gyro_cal(i,:) = gyro_cal(i,:) - w_drift(i);
end


% 初始化变量
acc_filter = acc_cal(: , 1); % 一阶低通后的加速度计值
gyro_fiter = gyro_cal(: , 1);
acc_lowfilt_hz = 1; % 加速度计低通截止频率

AccAngle = zeros(2,1);
AccAngle(1) = atan2(acc_filter(2), acc_filter(3)); % roll
AccAngle(2) = -atan2(acc_filter(1), sqrt(acc_filter(2)^2 + acc_filter(3)^2));
att = [AccAngle(1), AccAngle(2), 0]';
Q = funEuler2Quat( att);
Q1 = Q;
GyroAngle = att;

dt_err_num = 0; % dt = 0时，+1
%%
for i = 2:NUM  
    IMU_time(i) = i;
    
    if i == 3800
        kk=1;
    end
    
    dt = (time_s(i)-time_s(i-1)) + (time_us(i)-time_us(i-1))*1e-6;
    if dt ~= 0
        % IMU数据一阶低通滤波
        acc_new =  acc_cal(: , i);
        [ acc_filter ] = funLowpassFilterVector3f( acc_filter, acc_new, dt, acc_lowfilt_hz );

        %互补滤波
        gyro_fiter = gyro_cal(: , i);
        [att, Q ,AccAngle] = funComplementaryFilter_q_new(Q, acc_filter, gyro_fiter, dt );  
        
        [GyroAngle, Q1 ] = funComplementaryFilter_q(Q1, acc_filter, gyro_fiter,  dt );

    else
        dt_err_num = dt_err_num + 1; % 数据记录会有重复的时候（时间戳重复）
    end  
    
    acc_filter_save(:,i-1) = acc_filter;
    IMU_trim = [0, 0, 0]'*180/pi;    
    att_save(:,i-1) = att*180/pi - IMU_trim;
    AccAngle_save(:,i-1) = AccAngle*180/pi - IMU_trim(1:2);
    GyroAngle_save(:,i-1) = GyroAngle*180/pi - IMU_trim;    

end

figure()
plot(IMU_time(2:end),att_save(1, :));
hold on;
grid on;
plot(IMU_time(2:end), AccAngle_save(1, :));
plot(IMU_time(2:end), GyroAngle_save(1, :));
legend('att', 'AccAngle', 'GyroAngle');

figure()
plot(IMU_time(2:end), att_save(2, :));
hold on;
grid on;
plot(IMU_time(2:end), AccAngle_save(2, :));
plot(IMU_time(2:end), GyroAngle_save(2, :));
legend('att', 'AccAngle', 'GyroAngle');

figure()
plot(IMU_time(2:end), att_save(3, :));
hold on;
grid on;
plot(IMU_time(2:end), GyroAngle_save(3, :));
legend('att','GyroAngle');

%% IMU data plot 
% figure()
% % subplot(2,1,1)
% plot(IMU_time, acc_cal(1, :));
% hold on;
% plot(IMU_time, acc_cal(2, :));
% plot(IMU_time, acc_cal(3, :));
% grid on;
% plot(IMU_time, gyro_cal(1, :));
% plot(IMU_time, gyro_cal(2, :));
% plot(IMU_time, gyro_cal(3, :));
% legend('ax','ay','az','gx','gy','gz');


% subplot(2,1,2)
% plot(IMU_time, gyro_cal(1, :));
% hold on;
% plot(IMU_time, gyro_cal(2, :));
% plot(IMU_time, gyro_cal(3, :));
% grid on;
% legend('x','y','z');
% title('陀螺仪')


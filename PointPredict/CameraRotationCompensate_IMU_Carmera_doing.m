clc;
clear 
close all

%%
% camera_time camera_uv imu_time  acc_cal  gyro_cal
load('imu_camera.mat');

tmp1 = length(camera_time);
j = 0;
for i = 1:2:tmp1
    j = j + 1;
    camera_time_tmp(j) = camera_time(i);
    camera_uv_tmp(:,j) = camera_uv(:, i);
end
camera_time = camera_time_tmp;
camera_uv = camera_uv_tmp;


%相机内参
fx = 1437.72915;
fy = 1435.42215;
cx = 610.46050;
cy = 376.26171;
M1 = [fx   0 cx;
       0  fy cy;
       0   0  1 ];
Cimu2c = [0 1 0;
          0 0 1;
          1 0 0];

% 初始化变量
NUM_imu = length(imu_time);
NUM_camera = length(camera_time);

isFirstTime = 1;
carmera_index = 1;% 表示当前的camera的帧序号
Q_camera_pre = [1 0 0 0]';  % 记录上次camera数据来的时候的姿态


acc_cal = -acc_cal;
acc_filter = acc_cal(: , 1); % 一阶低通后的加速度计值
gyro_fiter = gyro_cal(: , 1);
acc_lowfilt_hz = 1; % 加速度计低通截止频率

% 用加速度的前50：100数据的平均值计算初始角度，给att赋初值
AccAngle = zeros(2,1);
for i = 1:3
    acc_init_data(i) = mean(acc_cal(i, 50:100));
end
AccAngle(1) = atan2(acc_init_data(2), acc_init_data(3)); % roll
AccAngle(2) = -atan2(acc_init_data(1), sqrt(acc_init_data(2)^2 + acc_init_data(3)^2));
att = [AccAngle(1), AccAngle(2), 0]';
Q = funEuler2Quat( att); 
Q1 = Q;
GyroAngle = att;

dt_err_num = 0; % dt = 0时，+1
%%
for i = 2:NUM_imu-1  
    IMU_time(i) = i;   
    dt = imu_time(i) - imu_time(i-1);
    if dt ~= 0
        % IMU数据一阶低通滤波
        acc_new =  acc_cal(: , i);
        acc_filter = funLowpassFilterVector3f( acc_filter, acc_new, dt, acc_lowfilt_hz );
        gyro_fiter = gyro_cal(: , i);
        
        %互补滤波        
        [att, Q ,AccAngle] = funComplementaryFilter_q_new(Q, acc_filter, gyro_fiter, dt );
        [GyroAngle, Q1 ] = funComplementaryFilter_q(Q1, acc_filter, gyro_fiter,  dt );
        
        % test
        if carmera_index == 449
            kk = 1;
        end
        
        % 判断camera时间戳，取最靠近的时间戳
        dt_current = abs(imu_time(i)-camera_time(carmera_index));
        dt_next = abs(imu_time(i+1)-camera_time(carmera_index));
        if dt_current < dt_next && carmera_index < NUM_camera
            if isFirstTime == 1 % 是否是第一次进入
                isFirstTime = 0;
                Q_camera_pre = Q;
                camera_uv_pre = camera_uv(:, carmera_index);
            else
                carmera_index = carmera_index + 1;% 表示当前IMU和camera时间戳是最接近的，进行预测计算
                camera_uv_new = camera_uv(:, carmera_index);

                 % 进行相机运动补偿，预测像素点坐标
                 Cc2n_pre = (Cimu2c*funQ2Cb2n(Q_camera_pre)')'; % c2n
                 Cn2c_current = (Cimu2c*funQ2Cb2n(Q)'); % n2c
                 C_dT = Cn2c_current*Cc2n_pre;                 
                 tmp = M1*C_dT*inv(M1)*[camera_uv_pre(1), camera_uv_pre(2), 1]';
                 camera_uv_estimation = [tmp(1), tmp(2)]';
                 camera_uv_true = camera_uv_new;                   
                 
                 % 预测坐标与实际坐标的误差
                 duv = camera_uv_estimation - camera_uv_true;
                 duv_rotation = camera_uv_pre - camera_uv_new; % 前后两帧里面像素点的运动
                 dR = sqrt(sum(duv.^2)) ; % 预测结果的误差半径
                 dR_rotation = sqrt(sum(duv_rotation.^2)) ; % 由于转动导致的前后两帧的像素偏移
                 
                 camera_uv_estimation_save(:, carmera_index-1) =  camera_uv_estimation;
                 camera_uv_true_save(:, carmera_index-1) = camera_uv_true;
                 duv_save(:, carmera_index-1) = duv; 
                 duv_rotation_save(:, carmera_index-1) = duv_rotation;
                 dR_save(:, carmera_index-1) = dR;
                 dR_rotation_save(:, carmera_index-1) = dR_rotation;
                 att_camera_rotation_save(:, carmera_index-1) = att*180/pi;
%                  if dR_rotation == 0 || dR_rotation<2
%                     dR_rotation_save(:, carmera_index-1) = 0;
%                  else
%                     dR_rotation_save(:, carmera_index-1) = dR/dR_rotation;
%                  end
                 
                 % pre值更新
                 camera_uv_pre = camera_uv_new;
                 Q_camera_pre = Q;
                 
            end            
        end
    else
        dt_err_num = dt_err_num + 1; % 数据记录会有重复的时候（时间戳重复）
    end  
    
    acc_filter_save(:,i-1) = acc_filter;
    att_save(:,i-1) = att*180/pi;
    AccAngle_save(:,i-1) = AccAngle*180/pi;
    GyroAngle_save(:,i-1) = GyroAngle*180/pi;    

end

%% 点坐标预测
figure()
% 实际的点
subplot(2,1,1)
plot(camera_uv_true_save(1, :));
grid on;
hold on;
plot(camera_uv_estimation_save(1, :));
legend('实际','预测点')
title('X轴');


subplot(2,1,2)
plot(camera_uv_true_save(2, :));
grid on;
hold on;
plot(camera_uv_estimation_save(2, :));
legend('实际','预测点')
title('Y轴');

% %预测点
% subplot(2,2,3)
% plot(camera_uv_estimation_save(1, :));
% grid on;
% legend('X');
% title('预测')
% 
% subplot(2,2,4)
% plot(camera_uv_estimation_save(2, :));
% grid on;
% legend('Y');
% title('预测')

%误差
figure()
subplot(2,1,1);
plot(duv_save(1, :));
grid on;
hold on;
plot(duv_save(2, :));
plot(duv_rotation_save(1, :));
plot(duv_rotation_save(2, :));
% plot(camera_uv_true_save(1, :)/50);
% plot(camera_uv_true_save(2, :)/50);
% legend('dX','dY','X','Y');
legend('dX','dY','duv_rotation-X','duv_rotation-Y');
title('XY误差')

subplot(2,1,2);
plot(dR_save);
grid on;
hold on;
plot(dR_rotation_save);
legend('dR','dR-rotation');
title('误差半径')

% 相机数据来的时候的姿态
figure()
plot(att_camera_rotation_save(1,:));
grid on;
hold on;
plot(att_camera_rotation_save(2,:));
plot(att_camera_rotation_save(3,:));
legend('X','Y','Z');
title('图像数据来的时候的姿态角')

%% 姿态角
% figure()
% plot(IMU_time(2:end),att_save(1, :));
% hold on;
% grid on;
% plot(IMU_time(2:end), AccAngle_save(1, :));
% plot(IMU_time(2:end), GyroAngle_save(1, :));
% legend('att', 'AccAngle', 'GyroAngle');
% 
% figure()
% plot(IMU_time(2:end), att_save(2, :));
% hold on;
% grid on;
% plot(IMU_time(2:end), AccAngle_save(2, :));
% plot(IMU_time(2:end), GyroAngle_save(2, :));
% legend('att', 'AccAngle', 'GyroAngle');
% 
% figure()
% plot(IMU_time(2:end), att_save(3, :));
% hold on;
% grid on;
% plot(IMU_time(2:end), GyroAngle_save(3, :));
% legend('att','GyroAngle');

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


% 去掉了时间的输入和输出
function [ data_imu ] = fun_imu_data_trans_simple( data_imu )
% Y1模组
    one_G = 9.80665; 
    % Y1模组的加速度计校正参数
    A0 = [0 0 0]';
    A1 = diag([1, 1, 1]);
    % raw data
    NUM_imu = length(data_imu);
    %% acc 
    accel_range_scale = 8/(2^15);% acc_max_g = 8g
    acc_raw = data_imu(1:3,:)*accel_range_scale;
    acc_cal_tmp(:, 1) = inv(A1)*(acc_raw(:, 1) - A0)*one_G;

    %% gyro
    gyro_range_scale = 2000/180*pi/(2^15); %acc_max_g = 2000 degree/s   rad/s
    gyro_cal_tmp = data_imu(4:6, : )*gyro_range_scale;

    % Y1模组
    acc_cal(1,:) = -acc_cal_tmp(3,:);
    acc_cal(2,:) = -acc_cal_tmp(2,:);
    acc_cal(3,:) = -acc_cal_tmp(1,:);
    gyro_cal(1,:) = -gyro_cal_tmp(3,:);
    gyro_cal(2,:) = -gyro_cal_tmp(2,:);
    gyro_cal(3,:) = -gyro_cal_tmp(1,:);
    
    data_imu = [acc_cal; gyro_cal];
end


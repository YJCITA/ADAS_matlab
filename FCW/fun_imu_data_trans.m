function [ data_imu ] = fun_imu_data_trans( data_imu )
% Y1模组
    one_G = 9.80665; 
    % Y1模组的加速度计校正参数
    A0 = [0.0628    0.0079   -0.0003]';
    A1 = [  1.0011    0.0028   -0.0141;
           -0.0161    1.0005    0.0181;
            0.0163   -0.0065    1.0140 ];
    A2 = [ -0.0047    0       0;
             0      0.0039    0;
             0         0    0.0124];
    % raw data
    [row, NUM_imu] = size(data_imu);
    %% acc 
    accel_range_scale = 8/(2^15);% acc_max_g = 8g
    acc_raw = data_imu(2:4,:)*accel_range_scale;
    for j = 1:NUM_imu
       acc_cal_tmp(:, j) = inv(A1)*(acc_raw(:, j) - A0)*one_G;
    end

    %% gyro
    gyro_range_scale = 2000/180*pi/(2^15); %acc_max_g = 2000 degree/s   rad/s
    gyro_cal_tmp = data_imu(5:7, : )*gyro_range_scale;
    % time
    imu_time = data_imu(1, :);

    % Y1模组
    if acc_cal_tmp(1,1) < -0.5*one_G
        % 新的摄像头
        acc_cal(1,:) = -acc_cal_tmp(3,:);
        acc_cal(2,:) = acc_cal_tmp(2,:);
        acc_cal(3,:) = acc_cal_tmp(1,:);
        gyro_cal(1,:) = -gyro_cal_tmp(3,:);
        gyro_cal(2,:) = gyro_cal_tmp(2,:);
        gyro_cal(3,:) = gyro_cal_tmp(1,:);        
    else
        % 旧的摄像头
        acc_cal(1,:) = -acc_cal_tmp(3,:);
        acc_cal(2,:) = -acc_cal_tmp(2,:);
        acc_cal(3,:) = -acc_cal_tmp(1,:);
        gyro_cal(1,:) = -gyro_cal_tmp(3,:);
        gyro_cal(2,:) = -gyro_cal_tmp(2,:);
        gyro_cal(3,:) = -gyro_cal_tmp(1,:);
        
        
    end
    data_imu = [imu_time; acc_cal; gyro_cal];


end


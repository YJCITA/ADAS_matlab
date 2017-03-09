clc
clear 
close all
%% IMU 数据预处理
% origin_address = ['data/1123_拨杆/'];
% origin_name = 'log-1123-bogan-';
origin_address = ['data/1123_拨杆/'];
origin_name = '0927_G_C-';

address_gensor = [origin_address, origin_name, 'gsensor.ini'];
data_raw_gensor = load(address_gensor)';
time_s = data_raw_gensor(1, :);
time_us = data_raw_gensor(2, :);
time = time_s + time_us *1e-6;
% 统一初始时间
time_start = time(1);
time = time - time_start;
data_gensor_raw = [time; data_raw_gensor(3:8, :)]; 
data_gsensor = fun_imu_data_trans( data_gensor_raw );

% turnlamp 转向灯信号：0 No steering light 1 Left steering light；2 Right steering light";
address_turnlamp = [origin_address, origin_name, 'turnlamp.ini'];
data_raw_turnlamp =  load(address_turnlamp);
data_raw_turnlamp = data_raw_turnlamp';
time_s = data_raw_turnlamp(1, :);
time_us = data_raw_turnlamp(2, :);
time = time_s + time_us *1e-6 - time_start;
turnlamp_NUM = length(time);
turnlamp_t(1, :) = data_raw_turnlamp(3, :);
for i = 1:turnlamp_NUM
    if data_raw_turnlamp(3, i) == 2
        turnlamp_t(1, i) = -1; % 右
    end        
end
data_turnlamp = [time; turnlamp_t];

figure()
plot(data_turnlamp(1,:)/3600, data_turnlamp(2,:));
hold on;
plot(data_gsensor(1,:)/3600, data_gsensor(5,:));
plot(data_gsensor(1,:)/3600, data_gsensor(6,:));
plot(data_gsensor(1,:)/3600, data_gsensor(7,:));
ylim([-1.5,1.5])
legend('trunlamp', 'gyro-x', 'gyro-y', 'gyro-z');
grid on;

figure()
plot(data_turnlamp(1,:)/3600, data_turnlamp(2,:));
hold on;
plot(data_gsensor(1,:)/3600, data_gsensor(2,:)/9.8);
plot(data_gsensor(1,:)/3600, data_gsensor(3,:)/9.8);
plot(data_gsensor(1,:)/3600, data_gsensor(4,:)/9.8);
legend('trunlamp', 'acc-x', 'acc-y', 'acc-z');
grid on;
%% FCW 车距频率分析
% 首先分析了数据，对于正常行驶，车距变化的频率应该是不会高于0.2~0.5hz左右（这个可以再测试分析下其他数据？？？）
% 所以可以对bouding box出来的数据先进行低通滤波，去除高频噪声

% 对于数据进行重采样，然后进行FFT频域分析
% save_z = [time, vision_range, vel_self]
close all
clear all
clc

%% 载入数据
vision_data = load('vision_range.mat');

%% imu波动和vision range波动对比
% 初步结论：bounding box出来的数据会波动，很肯能就是以内车辆的颠簸影响图像
source_addr = 'F:/数据/FCW/case1/log';
% imu数据
log_addr = [source_addr , '/39946.log-gsensor.ini'];
gsensor_data = load(log_addr)';
time_imu = gsensor_data(1,:) + gsensor_data(2,:)*1e-6;
data_imu_raw = [time_imu; gsensor_data(3:8, :)];
[ data_imu ] = fun_imu_data_trans( data_imu_raw );

% vision range数据
log_addr = [source_addr , '/39946.vision_dist'];
vision_range_data = load(log_addr)';

time_start = min(vision_range_data(1,1), data_imu(1,1));

figure()
plot(data_imu(1,:)-time_start, data_imu(6,:)*100+30);
hold on;
grid on;
plot(vision_range_data(1,:)-time_start, vision_range_data(2,:));
legend('gyro-Y', 'vision-range');

figure()
hold on;
grid on;
plot(vision_range_data(1,:)-time_start, vision_range_data(2,:));
legend('vision-range');

%% FFT 频域分析
% vision range
% 重采样
Fs = 25;  % 原始数据差不多是25hz,这个值不会影响频谱分析
vision_range_resample = resample(vision_data.save_z(2,:), vision_data.save_z(1, :), Fs);
fft_vision_range_sample = vision_range_resample;
fft_vision_range_sample = fft_vision_range_sample - mean(fft_vision_range_sample);
L = length(fft_vision_range_sample);
Y = fft(fft_vision_range_sample,L);%进行fft变换
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% 做频谱图
grid on;
title('vision range 原始数据频域特性')

% imu
% 重采样
Fs = 100;  % 原始数据差不多是25hz,这个值不会影响频谱分析
data_resample = resample(data_imu(6,:), data_imu(1, :), Fs);
data_resample = data_resample - mean(data_resample);
L = length(data_resample);
Y = fft(data_resample,L);%进行fft变换
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% 做频谱图
grid on;
title('imu-gyro-Y 原始数据频域特性')

%% 低通滤波处理
% 是否重采样，对低通几乎没影响
y = vision_data.save_z(2,1);
NUM = length(vision_data.save_z(2,:));
time_pre = vision_data.save_z(1,1);
for i = 1:NUM
    x_new =  vision_data.save_z(2,i);
    time_cur = vision_data.save_z(1,i);
    dt = time_cur - time_pre;
    time_pre = time_cur;
    filt_hz = 0.25;
    [ y ] = fun_LowpassFilter( y, x_new, dt, filt_hz );
    vision_new(:, i) = [time_cur; y];
    save_dt(1, i) = dt; 
end

figure();
plot(vision_data.save_z(1,:), vision_data.save_z(2,:)); % 原始vision range
hold on
plot(vision_new(1,:), vision_new(2,:)); % 低通滤波后vision range
grid on;
legend('vision-range-raw', 'vision-range-filter')
str_name = sprintf('低通截止频率： %.2f 滤波前后数据  ', filt_hz);
title(str_name)


%% vision range 减去低频数据后 在分析高频分量的频率
% 重采样
Fs = 25;  % 原始数据差不多是25hz,这个值不会影响频谱分析
vision_data_remove_low_hz = vision_data.save_z(2,:) - vision_new(2,:);
vision_data_remove_low_hz = vision_data_remove_low_hz - mean(vision_data_remove_low_hz);
time_data = vision_new(1, :);
data_resample = resample(vision_data_remove_low_hz, time_data, Fs);
data_resample = data_resample - mean(data_resample);
L = length(data_resample);
Y = fft(data_resample,L);%进行fft变换
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% 做频谱图
grid on;
title('vision range减去低频数据后频域特性')

%%  对比imu波动和vision range波动
figure()
plot(data_imu(1,:)-data_imu(1,1), data_imu(6,:));
grid on;
legend('gyro-Y')












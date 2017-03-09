% 读取bounding box出来的原始距离数据和雷达的数据进行对比

clc
clear all
close all

%% 数据导入
source_addr = 'F:/数据/FCW/case1/log';
% radar
data_addr = [source_addr , '/39946.log-radar.ini'];
data_t = load(data_addr)';
time_t = data_t(1,:) + data_t(2, :)*1e-6;
radar_data = [time_t; data_t(3:end, :)];

% bounding box range
data_addr = [source_addr , '/39946.vision_dist'];
vision_range_data = load(data_addr)';

% 统一一下时间轴  从0开始
time_start = min(vision_range_data(1,1), radar_data(1,1));
radar_data(1,:) = radar_data(1,:) - time_start;
vision_range_data(1,:) = vision_range_data(1,:) - time_start;

%% plot
figure()
plot(radar_data(1,:), radar_data(2,:));
hold on;
plot(vision_range_data(1,:), vision_range_data(2,:));
grid on;
legend('radar', 'vision-raw');
title('车距');

% figure()
% subplot(2,1,1)
% plot(vision_range_data(1,1:2802), vision_range_data(2,1:2802));
% 
% % subplot(2,1,2)
% hold on
% plot(vision_range_data(1,2803:end), vision_range_data(2,2803:end));
% title('车距');
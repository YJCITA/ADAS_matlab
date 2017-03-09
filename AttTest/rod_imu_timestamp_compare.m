% 比较rod和camera的acc数据的同步性

clc
clear all
close all

%% 读入数据
acc_imu = load('./data/0222_rod_imu_timestamp/2/rod_imu2-imu.ini')';
acc_rod = load('./data/0222_rod_imu_timestamp/2/rod_imu2-rod.ini')';

% 统一时间初始点
time0_imu = acc_imu(1, 1);
time0_rod = acc_rod(1, 1);
time0 = min(time0_imu, time0_rod);

acc_imu(1, :) = acc_imu(1, :) - time0;
acc_rod(1, :) = acc_rod(1, :) - time0;


%% 画图
% 坐标
figure(1)
subplot(3, 1, 1)
plot(acc_imu(1, :), acc_imu(2, :));
hold on;
plot(acc_rod(1, :), -acc_rod(4, :)); % z->x
grid on;
legend('acc-imu-x','acc-rod-x');

subplot(3, 1, 2)
plot(acc_imu(1, :), acc_imu(3, :));
hold on;
plot(acc_rod(1, :), acc_rod(2, :));% x->y
grid on;
legend('acc-imu-y','acc-rod-y');

subplot(3, 1, 3)
plot(acc_imu(1, :), acc_imu(4, :));
hold on;
plot(acc_rod(1, :), -acc_rod(3, :));
grid on;
legend('acc-imu-z','acc-rod-z');

% saveas(1,'filename','bmp');







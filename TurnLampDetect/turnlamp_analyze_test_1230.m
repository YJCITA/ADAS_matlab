% 利用imu的acc和gyro相对变化值，分析下是否可以唯一确定当前的状态

% fmu和camera件转换关系
fmu_acc_0 = [-2.42, -1.92, -9.25]';
camera_acc_0 = [-1, -0.36, -10.05]';

fmu_AccAngle(1,1) = atan2(-fmu_acc_0(2), -fmu_acc_0(3)); % roll
fmu_AccAngle(2,1) = atan2(fmu_acc_0(1), sqrt(fmu_acc_0(2)^2 + fmu_acc_0(3)^2));
fmu_AccAngle(3,1) = 0;
R_fmu = funAtt2Rnb( fmu_AccAngle );

camera_AccAngle(1,1) = atan2(-camera_acc_0(2), -camera_acc_0(3)); % roll
camera_AccAngle(2,1) = atan2(camera_acc_0(1), sqrt(camera_acc_0(2)^2 + camera_acc_0(3)^2));
camera_AccAngle(3,1) = 0;

R_camera = funAtt2Rnb( camera_AccAngle );
R_fmu2camera = R_camera*R_fmu';
low_filter_hz = 20;
             
% fmu delay
fmu_delay = 0.1;

% camera
imu_camera_pre = imu_data_camera(2:7, 1);
num_imu_camera = length(imu_data_camera);
acc_fliter_camera = imu_data_camera(2:4, 1);
for i = 2:num_imu_camera
    d_imu_camera(1,i) = imu_data_camera(1, i);
    d_imu_camera(2:7,i) = imu_data_camera(2:7, i) - imu_data_camera(2:7, i-1);
   
    % 保存低通后的数据
    acc_fliter_camera = funLowpassFilterVector3f( acc_fliter_camera, imu_data_camera(2:4, i), 0.01, low_filter_hz );   
    acc_fliter_camera_save(1,i) = imu_data_camera(1, i);
    acc_fliter_camera_save(2:4,i) = acc_fliter_camera;
end

% fmu
acc_fliter_fmu = imu_data_fmu(2:4, 1);
imu_fmu_pre = imu_data_fmu(2:7, 1);
num_imu_fmu = length(imu_data_fmu);
for i = 2:num_imu_fmu
    d_imu_fmu(1,i) = imu_data_fmu(1, i);
    d_imu_fmu(2:7,i) = imu_data_fmu(2:7, i) - imu_data_fmu(2:7, i-1);
    
    % 保存低通后的数据
    acc_fliter_fmu = funLowpassFilterVector3f( acc_fliter_fmu, imu_data_fmu(2:4, i), 11/400, low_filter_hz );   
    acc_fliter_fmu_save(1,i) = imu_data_fmu(1, i);
    acc_fliter_fmu_save(2:4,i) = R_fmu2camera*acc_fliter_fmu;
%     acc_fliter_fmu_save(2:4,i) = acc_fliter_fmu;
end

%% 数据重采样--截取相交时段的数据
time_begin = max(acc_fliter_fmu_save(1,1), acc_fliter_camera_save(1,1));
time_end = min(acc_fliter_fmu_save(1,end), acc_fliter_camera_save(1,end))-1;

length_tmp = length(acc_fliter_camera_save);
save_i_index = 0;
for i = 1:length_tmp
    if acc_fliter_camera_save(1,i) >= time_begin + fmu_delay && acc_fliter_camera_save(1,i) <= time_end + fmu_delay
        save_i_index = save_i_index + 1;
        acc_fliter_camera_for_resample(:, save_i_index) = acc_fliter_camera_save(:, i);
    end
end

length_tmp = length(acc_fliter_fmu_save);
save_i_index = 0;
for i = 1:length_tmp
    if acc_fliter_fmu_save(1,i) >= time_begin && acc_fliter_fmu_save(1,i) <= time_end
        save_i_index = save_i_index + 1;
        acc_fliter_fmu_for_resample(:, save_i_index) = acc_fliter_fmu_save(:, i);
    end
end

fs_HZ = 40;
for i=1:3
    acc_fliter_camera_resample(i,:) = resample(acc_fliter_camera_for_resample(1+i,:), acc_fliter_camera_for_resample(1,:), fs_HZ);
    acc_fliter_fmu_resample(i,:) = resample(acc_fliter_fmu_for_resample(1+i,:), acc_fliter_fmu_for_resample(1,:) - fmu_delay, fs_HZ);
end

% time reasmple
length_time_resample = length(acc_fliter_camera_resample);
for i = 1:length_time_resample
    time_reasmple(1, i) = i*(1/fs_HZ);
end

%% plot
% 加速度计差值
figure()
acc_diff = acc_fliter_fmu_resample(:, 1:length_time_resample) - acc_fliter_camera_resample(:, 1:length_time_resample);
plot(time_reasmple, acc_diff(1,:));
hold on;
plot(time_reasmple, acc_diff(2,:));
% plot(time_reasmple, acc_diff(3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10, 'k');
grid on;
% legend('diff-acc-x', 'diff-acc-y', 'diff-acc-z', 'turnlamp')
legend('diff-acc-x', 'diff-acc-y', 'turnlamp')
title('IMU差值')

% acc
% figure()
% plot(acc_fliter_camera_save(1,:), acc_fliter_camera_save(4,:)+10);
% hold on;
% plot(acc_fliter_fmu_save(1,:), acc_fliter_fmu_save(4,:)+10-0.88);
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('camera-acc-z', 'fmu-acc-z', 'turnlamp')
% title('两个imu的关系')

% acc-x
figure()
plot(acc_fliter_camera_save(1,:), acc_fliter_camera_save(2,:));
hold on;
plot(acc_fliter_fmu_save(1,:), acc_fliter_fmu_save(2,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-acc-x', 'fmu-acc-x', 'turnlamp')
title('两个imu的关系')

% acc-y
figure()
plot(acc_fliter_camera_save(1,:), acc_fliter_camera_save(3,:));
hold on;
plot(acc_fliter_fmu_save(1,:), acc_fliter_fmu_save(3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-acc-y', 'fmu-acc-y', 'turnlamp')
title('两个imu的关系')

% acc-z
figure()
plot(acc_fliter_camera_save(1,:), acc_fliter_camera_save(4,:));
hold on;
plot(acc_fliter_fmu_save(1,:), acc_fliter_fmu_save(4,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-acc-z', 'fmu-acc-z', 'turnlamp')
title('两个imu的关系')

% % gyro
% figure()
% plot(d_imu_fmu(1,:), d_imu_fmu(2+3,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(3+3,:));
% plot(d_imu_fmu(1,:), d_imu_fmu(4+3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('gyro-x', 'gyro-y','gyro-z', 'turnlamp')
% 
% 前后变化值acc
figure()
plot(d_imu_fmu(1,:), d_imu_fmu(2,:));
hold on;
plot(d_imu_fmu(1,:), d_imu_fmu(3,:));
plot(d_imu_fmu(1,:), d_imu_fmu(4,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10, 'k');
grid on;
legend('acc-x', 'acc-y','acc-z', 'turnlamp')
title('acc-fmu前后差值')

% gyro
figure()
plot(d_imu_fmu(1,:), d_imu_fmu(2+3,:));
hold on;
plot(d_imu_fmu(1,:), d_imu_fmu(3+3,:));
plot(d_imu_fmu(1,:), d_imu_fmu(4+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10, 'k');
grid on;
legend('gyro-x', 'gyro-y','gyro-z', 'turnlamp')
title('gyro-fmu前后差值')

% % acc
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(2,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(2,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
% grid on;
% legend('camera-acc-x','fmu-acc-x', 'turnlamp')
% 
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(3,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
% grid on;
% legend('camera-acc-y','fmu-acc-y', 'turnlamp')
% 
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(4,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(4,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
% grid on;
% legend('camera-acc-z','fmu-acc-z', 'turnlamp')
% 
% % gyro
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(2+3,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(2+3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('camera-gyro-x','fmu-gyro-x', 'turnlamp')
% 
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(3+3,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(3+3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('camera-gyro-y','fmu-gyro-y', 'turnlamp')
% 
% figure()
% plot(d_imu_camera(1,:), d_imu_camera(4+3,:));
% hold on;
% plot(d_imu_fmu(1,:), d_imu_fmu(4+3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:));
% grid on;
% legend('camera-gyro-z','fmu-gyro-z', 'turnlamp')

% camera_imu_address = ['./data/1229_²¦¸Ë/1228-log-gsensor.ini'];
% camera_imu_data =  load(camera_imu_address)';

fmu_imu_address = ['./data/1229_²¦¸Ë/1228-log-fmu.ini'];
fmu_imu_data =  load(camera_imu_address)';

% acc
figure()
plot(imu_data_camera(1,:), imu_data_camera(2,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(2,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
grid on;
legend('camera-acc-x','fmu-acc-x', 'turnlamp')
% 
% subplot(3,1,2)
% plot(imu_data_camera(1,:), imu_data_camera(3,:));
% hold on;
% plot(imu_data_fmu(1,:), imu_data_fmu(3,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
% grid on;
% legend('camera-acc-y','fmu-acc-y', 'turnlamp')
% 
% subplot(3,1,3)
% plot(imu_data_camera(1,:), imu_data_camera(4,:));
% hold on;
% plot(imu_data_fmu(1,:), imu_data_fmu(4,:));
% plot(turnlamp_data(1,:), turnlamp_data(2,:)*10);
% grid on;
% legend('camera-acc-z','fmu-acc-z', 'turnlamp')
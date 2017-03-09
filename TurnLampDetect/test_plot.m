% acc
figure()
plot(imu_data_camera(1,:), imu_data_camera(2,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(2,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*5);
grid on;
legend('camera-acc-x','fmu-acc-x', 'turnlamp')

figure()
plot(imu_data_camera(1,:), imu_data_camera(3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*5);
grid on;
legend('camera-acc-y','fmu-acc-y', 'turnlamp')

figure()
plot(imu_data_camera(1,:), imu_data_camera(4,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(4,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:)*5);
grid on;
legend('camera-acc-z','fmu-acc-z', 'turnlamp')

% gyro
figure()
plot(imu_data_camera(1,:), imu_data_camera(2+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(2+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-x','fmu-gyro-x', 'turnlamp')

figure()
plot(imu_data_camera(1,:), imu_data_camera(3+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(3+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-y','fmu-gyro-y', 'turnlamp')

figure()
plot(imu_data_camera(1,:), imu_data_camera(4+3,:));
hold on;
plot(imu_data_fmu(1,:), imu_data_fmu(4+3,:));
plot(turnlamp_data(1,:), turnlamp_data(2,:));
grid on;
legend('camera-gyro-z','fmu-gyro-z', 'turnlamp')
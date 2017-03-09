figure()
plot(IMU_time(2:end),att_save(1, :));
hold on;
grid on;
plot(IMU_time(2:end), AccAngle_save(1, :));
plot(IMU_time(2:end), GyroAngle_save(1, :));
plot(att_q_time, att_q(1, :));
plot(IMU_time(2:end), w_P_save(1, :)*180/pi*10);
plot(IMU_time(2:end),att_save(1, :) - AccAngle_save(1, :));
legend('att', 'AccAngle', 'GyroAngle','q','wP', 'error');
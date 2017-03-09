% close all

figure()
plot(-save_angle_line_M);
hold on;
grid on;
plot(save_yaw_degree);
plot(save_fai);
legend('angle-line-M', 'yaw-degree', 'fai');

figure()
plot(-save_angle_line_M+save_angle_line_M(1));
hold on;
grid on;
plot(save_yaw_degree-save_yaw_degree(1));
plot(save_fai-save_fai(1));
legend('angle-line-M', 'yaw-degree', 'fai');

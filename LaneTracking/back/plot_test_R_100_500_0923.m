NUM = length(R_VNw_speed);
kk = 0;
for i = 1:NUM
    R_abs = abs(R_VNw_speed(i));
    if R_abs > 100 && R_abs < 500
        R_VNw_speed_100_500(1, i) = R_VNw_speed(i);
        kk = kk + 1;
    else
         R_VNw_speed_100_500(1, i) = 0;
    end
    
end
figure()
plot( R_VNw_speed_100_500);
grid on;
ylim([-700,700])
legend('RÔÚ[100, 500]')

kk/NUM*100
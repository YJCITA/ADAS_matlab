clc
clear all

% 转换规则：对N位二进制表示的有符号十进制数，其数值范围为-2^(N-1)~2^(N-1)-1，
% 对正数n，对应十六进制数为dec2hex(n)；
% 对负数n，对应十六进制数为dec2hex(2^(N)+n)。
data = load('3-fmu.txt');
att = data(:, 4:6)/10; % 度
vel = data(:, 7:9)/100; % m/s
vel_x = data(:, 7);

vel_x(1) = 12027;
NUM = length(vel_x);
for i = 1:NUM
    if vel_x(i)<0
        hex_t = dec2hex(vel_x(i)+2^16, 4);
    else
        hex_t = dec2hex(vel_x(i), 4);
    end
    
    hex_new = [hex_t(3), hex_t(4), hex_t(1), hex_t(2)];
    
    if hex_new(1)>='a' || hex_new(1)>='8'  % 为负数
        vel_x_new(1,i) = hex2dec(hex_new) - 2^16;
    else
        vel_x_new(1,i) = hex2dec(hex_new);
    end
end

% tt = dec2hex(-10241+2^16, 4);

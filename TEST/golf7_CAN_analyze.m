% golf7 CAN数据进行拟合
clc
clear all
close all

%%
% Left1(-6.5°) ：0x00D
% 
% Left2(-16.9°) ：0x68A
% 
% Left3(-35.4°) ：0xBC4
% 
% Right1(-12.3°) ：0x48F
% 
% Right2 (-19.5°) ：0x74B
% 
% Right3 (-28.4°) ：0xB05


y = [6.5 16.9 35.4 12.3 19.5 28.4]';
x = [hex2dec('0D') hex2dec('68A') hex2dec('BC4') hex2dec('48F') hex2dec('74B') hex2dec('B05')]';

% plot(0.15*y, x,'*');

% y1 = [6.5 12.3 19.5 28.4]';
y1 = [-180 90 180]';
% y1 = [-16.9 -35.4 12.3 19.5 28.4]';
x1 = [-hex2dec('68A')  hex2dec('48F') hex2dec('74B')]';

NUM_length = length(x1);
X = [ones(NUM_length,1), x1];
Y = y1;
A = inv(X'*X)*X'*Y;
j = 0;
for i = -hex2dec('B05'):1:hex2dec('B05')
    j = j + 1;
    line_plot(1,j) = i;
    line_plot(2,j) = A'*[1, i]';
end

figure()
plot(x1, y1, '*');hold on;
plot(line_plot(1, :), line_plot(2, :));
grid on;


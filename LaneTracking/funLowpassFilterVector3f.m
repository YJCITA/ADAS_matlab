% 1*3矢量低通滤波器
% y_pre：上一刻输出的低通后的值
% x_new: 新的量测值
% dt： 前后两次的时间间隔
% filt_hz：低通截止频率
function [ y ] = funLowpassFilterVector3f( y_pre, x_new, dt, filt_hz )

    if filt_hz == 0
        alpha = 1;
    else
        rc = 1/(2*pi*filt_hz);
        alpha = dt/(dt + rc);
    end    
    y = y_pre + alpha.*(x_new - y_pre);
end


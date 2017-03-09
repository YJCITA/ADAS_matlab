% 获取一阶低通滤波的系数
function [ alpha ] = get_filt_alpha( dt, filt_hz )

    if filt_hz == 0
        alpha = 1;
    else
        rc = 1/(2*pi*filt_hz);
        alpha = dt/(dt + rc);
    end

end


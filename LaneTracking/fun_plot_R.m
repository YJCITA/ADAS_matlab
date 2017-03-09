function [ CC_rgb ] = fun_plot_R( R_current, CC_rgb, rgb_value_t, camera_parameter  )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;
  
    R_current_abs = abs(R_current);
    if R_current == 0
        x_max_t = x_max;
    else
        x_max_t = min([x_max, R_current_abs]);
    end

    for x_line = x_min:0.1:x_max_t
        if R_current>0
            y_line = R_current - sqrt(R_current^2 - x_line^2);
        elseif R_current<0 
            y_line = R_current + sqrt(R_current^2 - x_line^2);
        else
            y_line = 0;
        end          
        M = (-x_line + x_max)*H1/x_max; % 减少重复计算
        if x_line<x_max_t && x_line>x_min && y_line>y_min && y_line<y_max  
            N = (y_line + y_max)*W1/(2*y_max);
            u = round(N);
            v = round(M);
            if u>0.5 && v>0.5 && u<W1 && v<H1
%                     rgb_value_t = 240;% 0:黑色
                CC_rgb(v, u, :) = rgb_value_t; 
                CC_rgb(v+1, u, :) = rgb_value_t;
                CC_rgb(v, u+1, :) = rgb_value_t;
                CC_rgb(v+1, u+1, :) = rgb_value_t;

            end  
        end
    end
   
end


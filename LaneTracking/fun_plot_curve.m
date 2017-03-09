function [ CC_rgb ] = fun_plot_curve( line_p, CC_rgb, rgb_value_t, camera_parameter  )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;
%     zoom = camera_parameter.zoom;
          
    for x_line = x_min:0.1:x_max
        y_line = [x_line^2, x_line, 1]*line_p;            
        M = (-x_line + x_max)*H1/x_max; % ºı…Ÿ÷ÿ∏¥º∆À„
        if x_line<x_max && x_line>x_min && y_line>y_min && y_line<y_max  
            N = (y_line + y_max)*W1/(2*y_max);
            u = round(N);
            v = round(M);
            if u>2 && v>2 && u<W1-1 && v<H1-1
                CC_rgb(v, u, :) = rgb_value_t;
            end  
        end
    end    
end


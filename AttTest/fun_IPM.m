function [ CC_rgb ] = fun_IPM( I_rgb, camera_parameter )
    x_min = camera_parameter.x_min;
    x_max = camera_parameter.x_max;
    y_min = camera_parameter.y_min;
    y_max = camera_parameter.y_max;
    H1 = camera_parameter.H1;
    W1 = camera_parameter.W1;    
    m = camera_parameter.m;
    n = camera_parameter.n;
    
    roll = camera_parameter.roll;
    pitch = camera_parameter.pitch;
    yaw = camera_parameter.yaw;
    
    Pc = camera_parameter.Pc;
    M1 = camera_parameter.M1;    
  
    % 相机姿态矩阵
    Ratt_roll = [1        0        0;
                 0   cos(roll)  sin(roll);
                 0   -sin(roll)  cos(roll);];
             
    Ratt_pitch = [cos(pitch)  0  -sin(pitch);
                      0        1     0;
                  sin(pitch)  0  cos(pitch)];
    Ratt_yaw = [cos(yaw)   sin(yaw)  0;
                -sin(yaw)  cos(yaw)  0;
                     0        0      1;];
    Ratt = Ratt_roll*Ratt_pitch*Ratt_yaw;     
    Rc12c = [0 1 0;% 相机-图像坐标系  
             0 0 1;
             1 0 0];         
    I3 = diag([1,1,1]);
    
    % 相机位置的偏移
    R_w2i =  M1*Rc12c*Ratt*[I3 -Pc]
    for M = 1:H1               %变换之后路面图像H1*W1=400*600个像素，路面宽度为7m，高度为10m
        x = -(M*x_max/H1 - x_max);
        for N=1:W1            
            y = N*2*y_max/W1 - y_max; 
            if x<x_max && x>x_min && y>y_min && y<y_max
                % 投影变化，相机成像模型
                Point_xyz = [x, y, 0, 1]';
                uv_tmp = R_w2i*Point_xyz;
                uv_new = uv_tmp/(uv_tmp(3));% z方向深度归一化
                u = round(uv_new(1));
                v = round(uv_new(2));                
                if u>0.5 && v>0.5 && u<n && v<m
                    %CC1(M, N) = I_g(v,u);
                    CC_rgb(M, N, :) = I_rgb(v, u, :);
                end  
            end
        end
    end 


end


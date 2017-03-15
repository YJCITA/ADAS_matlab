%% 2017.03.13 修改程序增加对数据批处理的能力
% 修改模型 改变为匀加速模型  效果非常好
% 以bounding box作为量测进行KF跟踪
clc
clear all
close all

%% 数据导入
log_id_vector = [14364, 14468, 14156, 10300, 15937, 13796, 1487, 12416, 8429, 9569, 11979, 11191, 11226, 15793, 10553];
NUM_log = length(log_id_vector);
for log_index = 1:NUM_log
    log_ID = num2str(log_id_vector(log_index));
    source_addr = 'F:/数据/FCW/dist_cases';
    log_addr = [source_addr , '/', log_ID, '.log.txt'];
    vison_range_addr = [source_addr , '/', log_ID, '.vision_dist'];
    % source_addr = 'F:/数据/FCW/case1/log';
    % log_addr = [source_addr , '/39946.log.txt'];
    % vison_range_addr = [source_addr , '/39946.vision_dist'];
    % radar_addr = [source_addr , '/39946.log-radar.ini'];

    fid_log = fopen(log_addr,'r');
    fid_vison_range = fopen(vison_range_addr,'r');

    %% 初始化
    is_start_from_beginning = 1; % 是否从log中第一帧开始计算
    ipm_index = 2200; % 从哪一帧图片开始ipm
    ipm_step = 1; % 步长

    % 量测数据
    struct_speed.data = 0;
    struct_speed.counter = 0;
    speed_update = 0; % 新的speed数据更新
    radar_range_cur = 0;
    radar_vel_cur = 0;
    % 滤波
    is_first_read_camera_data = 1;
    is_kf_init_ok = 0; % KF是否已经进行初始化
    is_first_radar_data = 1;
    save_i_index = 0; % 数据存储计数

    dv_filter = 0;
    % 对vision range 和 radar数据查询首先读取一个值
    % 因为radar和camera数据更新频率是接近或者更慢的，所以每次查询时先读取新的数据，可能是不匹配程度加剧
    % 应该是每次进入查询的时候先匹配上一个数据是否可用
    vison_range_data = fgetl(fid_vison_range);
    str_line_raw = regexp(vison_range_data,' ','split'); %
    vison_range_cur = str2num(str_line_raw{1,2});
    vison_range_raw = vison_range_cur;
    is_first_vision_range = 1;
    fcw_state = 0;

    %% 主循环
    while ~feof(fid_log) % 读取log数据    
        lineData = fgetl(fid_log);
        str_line_raw = regexp(lineData,' ','split'); %以空格为特征分割字符串
        time_s = str2num(str_line_raw{1,1});
        time_us = str2num(str_line_raw{1,2});
        time = time_s + time_us *1e-6;
        str_line_data_flag = str_line_raw(3);
        % Gsensor
        if  strcmp(str_line_data_flag, 'Gsensor')
            for i = 1:6
                imu_data_t(i, 1) = str2num(str_line_raw{1, i+3});
            end
            data_gensor_raw = [time; imu_data_t]; 

        % speed
        elseif strcmp(str_line_data_flag, 'brake_signal')
            speed_cur = str2num(str_line_raw{1, 24})/3.6; 
            struct_speed.data = struct_speed.data + speed_cur;
            struct_speed.counter =  struct_speed.counter + 1;
            speed_update = 1;        
        % fcw
        elseif strcmp(str_line_data_flag, 'sound_type')
            fcw_state = str2num(str_line_raw{1, 24}); 
        % radar
        elseif strcmp(str_line_data_flag, 'Id')
            radar_range_cur = str2num(str_line_raw{1,16});
            radar_vel_cur = str2num(str_line_raw{1,29});
            radar_acc_cur = str2num(str_line_raw{1,25});
            if is_first_radar_data
                radar_range_pre = radar_range_cur;
                dv_radar_filter = 0;
                radar_time_pre = time;
                is_first_radar_data = 0;
            end
            dt_radar = time - radar_time_pre;
            radar_time_pre = time;

            % 直接距离微分计算相对速度
            if dt_radar ~= 0
                dv_radar = (radar_range_cur - radar_range_pre)/dt_radar;
                radar_range_pre = radar_range_cur;
                [ dv_radar_filter ] = fun_LowpassFilter( dv_radar_filter, dv_radar, dt_radar, 0.5 );  
            end
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % 获取数据
            t_s = str2num(str_line_raw{1, 1});
            t_us = str2num(str_line_raw{1, 2});
            image_timestamp = t_s + t_us*1e-6;
            mp4_file_name_log = str_line_raw{1, 4}; % mp4文件路径
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log中图像index编号是从0开始

            if(is_first_read_camera_data)
                iamge_timestamp_pre = image_timestamp;
                time_start = image_timestamp;
                if is_start_from_beginning
                    ipm_index = image_index_num+3;
                end
                is_first_read_camera_data = 0;
            end        

             % 比对当前图像index  因为可能控制了step  && speed数据要有更新
            if  ipm_index == image_index_num           
                % 速度进行平均
                if struct_speed.counter > 0
                    speed_average = struct_speed.data/struct_speed.counter;
                end
                struct_speed.data = 0;
                struct_speed.counter = 0;

               %% 匹配时间戳  读取bounding box的range
                is_vision_range_search_ok = 0;
                while (~feof(fid_vison_range) && ~is_vision_range_search_ok)                
                    str_line_raw = regexp(vison_range_data,' ','split'); %以空格为特征分割字符串
                    time_vision = str2num(str_line_raw{1,1});
                    dt1 = time_vision - image_timestamp;

                    % 低通滤波初始化
                    if(is_first_vision_range)
                        time_vision_range_pre = time_vision;
                        is_first_vision_range = 0;
                    end
                    if abs(dt1)<0.5 || dt1>0
                        if dt1 > 2
                            fprintf('vison_range search error!!\n ');
                        end 
                        is_vision_range_search_ok = 1;
                    else
                        vison_range_data = fgetl(fid_vison_range);            
                        vison_range_raw =  str2num(str_line_raw{1,2});
                    end                         
                end

              %%  滤波          
                if(~is_kf_init_ok)
                    acc = 0;
                    speed_relative = 0;
                    Xk = [vison_range_cur, speed_relative, acc]';
                    Pk = diag([2, 1, 1]);
                    Q = diag([5,2,2]);
                    R = diag([4]);      
%                     Pk = diag([10, 5, 4]);
%                     Q = diag([5,2,2]);
%                     R = diag([5]);  

                    vison_range_pre = vison_range_raw;
                    is_kf_init_ok = 1;
                end
                % 低通滤波处理                    
                dt = time_vision - time_vision_range_pre;
                time_vision_range_pre = time_vision;
                filt_hz = 0.4; % 为了控制波动  基本0.5是极限
                [ vison_range_cur ] = fun_LowpassFilter( vison_range_cur, vison_range_raw, dt, filt_hz ); 

                % X = [d, vs, vt]' 车距，本车速度，目标车速
                % z = [vision_range car_speed]
                dt_image = image_timestamp - iamge_timestamp_pre;
                iamge_timestamp_pre = image_timestamp; 
                z = [vison_range_cur]';
    %             z = [radar_range_cur, speed_average]';  % 以雷达作为量测
                F = [1  dt_image  1/2*dt_image^2;
                     0   1  dt_image;
                     0   0  1];
                 H = [1 0 0;];
                [Xk_new, Pk_new] = fun_KF(Xk, Pk, z, Q, R, F, H);
                Xk = Xk_new;
                Pk = Pk_new;

                % 保存数据
                time_cur = image_timestamp - time_start;
                time_cur_file = image_timestamp;
                save_i_index = save_i_index+1;
                save_Xk_file(:, save_i_index) = [time_cur_file; Xk_new]; % 用于保存文件
                save_Xk(:, save_i_index) = [time_cur; Xk_new];
                save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
                dv =  Xk_new(2);% vt-vs 
                % 计算ttc
                if abs(dv) >0.5
                    ttc = Xk_new(1)/dv;
                else
                    ttc = 0;
                end
                save_ttc(:, save_i_index) = [time_cur; ttc];
                save_relative_v(:, save_i_index) = [time_cur; dv];
                save_z(:, save_i_index) = [time_cur; z];
                save_radar(:, save_i_index) = [time_cur; radar_range_cur; radar_vel_cur; radar_acc_cur];
    %             save_dv_radar(:, save_i_index) = [time_cur; dv_radar];
                save_dv_radar_filter(:, save_i_index) = [time_cur; dv_radar_filter];
                save_dt_image(:, save_i_index) = [time_cur; dt_image];
                save_fcw_state(:, save_i_index) = [time_cur; fcw_state];
                % 直接距离微分计算相对速度
                if dt_image ~= 0
                    dv_t = (vison_range_raw - vison_range_pre)/dt_image;
                    vison_range_pre = vison_range_raw;
                    [ dv_filter ] = fun_LowpassFilter( dv_filter, dv_t, dt_image, 3 ); 
                    save_dv_t(:, save_i_index) = [time_cur; dv_t];
                    save_dv_filter(:, save_i_index) = [time_cur; dv_filter];
                end
                ipm_index = ipm_index + ipm_step;
            end
        end
    end
    
%     figure('visible','off')    
    h = figure();
    % 车距
    subplot(3,1,1)
    plot(save_vision_raw(1,:), save_vision_raw(2,:)); % vision range 量测
    hold on;
    plot(save_Xk(1,:), save_Xk(2,:)); % range-estimate
    plot(save_radar(1,:), save_radar(2,:)); % 雷达
    grid on;
    legend({'vision-range-measure-raw','range-estimate','radar-range'},'Location','northeast','FontSize',7);
    legend('boxoff')
    log_addr_t = [ log_ID, '.log.txt'];
    str_name = sprintf('log文件: %s \n 车距 ', log_addr_t);
    title(str_name);
    
    % 速度
    subplot(3,1,2)
    plot(save_relative_v(1,:), save_relative_v(2,:)); % vel estimation
    hold on;
    plot(save_radar(1,:), save_radar(3,:)); % 雷达测量相对速度
    grid on;
    legend({'vel-estimation', 'radar-vel'},'Location','northeast','FontSize',7);
    legend('boxoff')
    title('速度');

    % ttc fcw
    subplot(3,1,3)
    plot(save_ttc(1,:), save_ttc(2,:)); % ttc
    hold on;
    plot(save_fcw_state(1,:), save_fcw_state(2,:)*10); % fcw_mobileye
    NUM = length(save_ttc(1,:));
    plot(save_ttc(1,:), ones(1,NUM)*4)
    plot(save_ttc(1,:), ones(1,NUM)*-4)
    grid on;
    legend({'ttc', 'fcw-mobileye'},'Location','northeast','FontSize',7);
    legend('boxoff')
    str_name = sprintf('ttc&fcw');
    ylim([-30, 30]);
    title(str_name);
    
    % 保存figure
    filename=[log_addr_t, '--plot.png'];
    saveas(gcf,filename)
%     close(gcf)    
    
    % 保存计算结果txt
    NUM = length(save_Xk_file);
    est_log_name = ['./data/est_result/', log_ID, '.log_KF_estimation.txt'];
    fp = fopen(est_log_name, 'wt');
    for i = 1:NUM
        fprintf(fp, '%f %f %f %f\n', save_Xk_file(1, i), save_Xk_file(2, i), save_Xk_file(3, i), save_Xk_file(4, i));
    end
    fclose(fp);

end

%% plot
% figure()
% plot(save_vision_raw(1,:), save_vision_raw(2,:)); % vision range 量测
% hold on;
% plot(save_Xk(1,:), save_Xk(2,:)); % range-estimate
% plot(save_radar(1,:), save_radar(2,:)); % 雷达
% grid on;
% legend('vision-range-measure-raw','range-estimate','radar-range');
% str_name = sprintf('车距 ');
% title(str_name);
% 
% % 速度
% figure()
% plot(save_relative_v(1,:), save_relative_v(2,:)); % vel estimation
% hold on;
% plot(save_radar(1,:), save_radar(3,:)); % 雷达测量相对速度
% % plot(save_dv_radar_filter(1,:), save_dv_radar_filter(2,:)); % 雷达数据直接微分的速度
% % plot(save_dv_filter(1,:), save_dv_filter(2,:)); % 直接微分后低通滤波的速度
% grid on;
% legend('vel-estimation', 'radar-vel', 'dv-雷达微分后低通', 'dv-微分后低通滤波');
% title('速度');
% 
% % 加速度
% figure()
% plot(save_Xk(1,:), save_Xk(4,:)); % acc estimation
% hold on;
% plot(save_radar(1,:), save_radar(4,:)); % 雷达测量acc
% grid on;
% legend('acc-estimation', 'radar-acc');
% title('加速度');
% 
% % ttc fcw
% figure()
% plot(save_ttc(1,:), save_ttc(2,:)); % ttc
% hold on;
% plot(save_fcw_state(1,:), save_fcw_state(2,:)*20); % fcw_mobileye
% NUM = length(save_ttc(1,:));
% plot(save_ttc(1,:), ones(1,NUM)*4)
% plot(save_ttc(1,:), ones(1,NUM)*-4)
% grid on;
% legend('ttc', 'fcw-mobileye');
% title('ttc&fcw');


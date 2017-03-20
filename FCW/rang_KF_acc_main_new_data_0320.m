%% 2017.03.13 修改程序增加对数据批处理的能力
% 修改模型 改变为匀加速模型  效果非常好
%% 2017.03.18 适应新的数据格式，增加批处理能力，统计正报和误报
% 以bounding box作为量测进行KF跟踪
%% 2017.03.20  这个版本效果还OK，考虑增加连续的横向速度估计和IMU判断颠簸情况，去掉误报
clc
clear all
close all

%% 数据导入
% 遍历文件夹
maindir = 'F:\数据\FCW\result_0316';
sub_l1_dir  = dir( maindir );
% 一级目录
for l1_i = 1 : length( sub_l1_dir )
    if( isequal( sub_l1_dir(l1_i).name, '.' )|| isequal( sub_l1_dir(l1_i).name, '..')||  ~sub_l1_dir(l1_i).isdir)               % 如果不是目录则跳过
        continue;
    end
    % 二级子目录
    sub_l2_dir = dir(fullfile( maindir, sub_l1_dir(l1_i).name));
    for l2_i = 1 : length( sub_l2_dir )
        if( isequal( sub_l2_dir(l2_i).name, '.' )|| isequal( sub_l2_dir(l2_i).name, '..')||  ~sub_l2_dir(l2_i).isdir)               % 如果不是目录则跳过
            continue;
        end
        subdirpath = fullfile( maindir, sub_l1_dir(l1_i).name, sub_l2_dir(l2_i).name, '*.txt' );
        txt_dir = dir( subdirpath );               % 子文件夹下找后缀为txt的文件
        for txt_i = 1:length(txt_dir)
            log_addr = fullfile( maindir, sub_l1_dir(l1_i).name, sub_l2_dir(l2_i).name, txt_dir(txt_i).name);
            fid_log = fopen(log_addr, 'r');

           %% 初始化
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
            is_first_vision_range = 1;
            fcw_state = 0;

            %% 主循环相关变量
            is_first_read_legal_data = 1;
            image_frame_id_pre = 0; % 上一次计算时候image frame index 用于计算dt
            image_frame_T = 5*60/7744; % 两帧图片之间的dt  一个视频5min，差不多是7744帧
            mobileye_fcw_state = 0; % mobileye fcw state  1：warning
            index_of_target_pre = 0; % 上一帧目标车辆的ID
            is_new_target = 0; % 是否是新目标车辆            
            fcw_counter = 0; % 满足触发条件计数
          %% 主循环
            while ~feof(fid_log) % 读取log数据    
                lineData = fgetl(fid_log);
        %         str = 'Here is a date: 01-Apr-2020';
        %         expression = '(?<day>\d+)-(?<month>\w+)-(?<year>\d+)';
        %         mydate = regexp(str,expression,'names')
                exp1 = '	';
                str_line_raw = regexp(lineData, exp1, 'split'); %以空格为特征分割字符串
                image_frame_id = str2num(str_line_raw{1,1});
                image_timestamp_cur = image_frame_id*image_frame_T; % 通过图像生成频率计算timestamp
                str_frame_index = str2num(str_line_raw{1,2}); % 每一幅图对应3帧

                % 读取mobileye 数据
                if(str_frame_index == 1)
                    data_mobileye_tmp(1,1) = str2num(str_line_raw{1,21});
                    data_mobileye_tmp(2,1) = str2num(str_line_raw{1,23});
                    if(data_mobileye_tmp(1) == 0 && data_mobileye_tmp(2) == 1)
                        mobileye_fcw_state = 1;
                    else
                        mobileye_fcw_state = 0;
                    end
                end

                % 新数据 index: 0: minieye vision raw result   2: filter data        
                if(str_frame_index == 0)
                    % 采用 index:0 读取数据
                    speed_cur = str2num(str_line_raw{1,12});
                    vison_range_raw = str2num(str_line_raw{1,13});
                    vision_horizon = str2num(str_line_raw{1,14});
                    index_of_target = str2num(str_line_raw{1,17}); % >=0 is legal
                    ttc_raw = str2num(str_line_raw{1,19}); 

                    if(is_first_read_legal_data)
                        image_timestamp_pre = image_timestamp_cur;
                        is_first_read_legal_data = 0;
                    end 

                  %%  滤波          
                    if(~is_kf_init_ok)
                        acc = 0;
                        speed_relative = 0;
                        vison_range_cur = vison_range_raw;
                        acc_h = 0;
                        speed_relative_h = 0;
                        vision_horizon_cur = vision_horizon;
                        Xk = [vison_range_cur, speed_relative, acc, vision_horizon_cur, speed_relative_h, acc_h ]';
                        Pk = diag([2, 4, 5, 1, 0.5, 0.5]);
                        Q = diag([5, 2, 5, 0.2, 0.5, 0.5]);
                        R = diag([4, 0.09]);      
                        is_kf_init_ok = 1;
                    end
                    
                    % 判断 target ID，来确认是否是一辆新的车
                    if (index_of_target_pre ~= index_of_target)
                        is_new_target = 1;
                    else
                        is_new_target = 0;                        
                    end
                    index_of_target_pre = index_of_target;                    
                    dt_image = image_timestamp_cur - image_timestamp_pre;
                    image_timestamp_pre = image_timestamp_cur; 
                    % 为了处理长时间没有有效前车的问题 dt太大则重置滤波
                    % 对于新目标，也要重置滤波
                    if(dt_image > 0.5 || is_new_target)
                        % dt >1 则重置
                        acc = 0;
                        speed_relative = 0;
                        vison_range_cur = vison_range_raw;
                        acc_h = 0;
                        speed_relative_h = 0;
                        vision_horizon_cur = vision_horizon;
                        Xk = [vison_range_cur, speed_relative, acc, vision_horizon_cur, speed_relative_h, acc_h ]';
                        Pk = diag([5, 4, 2, 1, 0.5, 0.5]);
                        Q = diag([5,2,5, 0.2, 0.5, 0.5]);
                        R = diag([4, 0.09]);  
                    else
                        % 低通滤波处理 
                        filt_hz = 0.5; % 为了控制波动  基本0.7是极限（>1之后就会引入车辆波动导致的车速变化）
                        [ vison_range_cur ] = fun_LowpassFilter( vison_range_cur, vison_range_raw, dt_image, filt_hz ); 

                        % KF 滤波
                        % X = [d, vs, vt]' 车距，本车速度，目标车速
                        % z = [vision_range car_speed]                   
                        z = [vison_range_cur vision_horizon]';
                        F_sub = [1  dt_image  1/2*dt_image^2;
                                 0   1  dt_image;
                                 0   0  1];
                        O33 = zeros(3, 3);
                        F = [F_sub, O33;
                             O33, F_sub];
                         H = [1 0 0 0 0 0;
                              0 0 0 1 0 0];
                         [Xk, Pk] = fun_KF(Xk, Pk, z, Q, R, F, H);
                    end
                    
                  %% FCW触发逻辑判断
                    % 计算ttc 
                    dv =  Xk(2);% vt-vs 
                    if abs(dv) >0.2
                        ttc = abs(Xk(1)/dv);
                    else
                        ttc = -1;
                    end
                    % fcw 触发逻辑 ttc<4, 相对车距离<20, 车速>60/3.6, 水平距离<1.2, 累计满足5帧
                    % 初步判断mobileye告警的场景都是比较高速的
                    % vel < -1
                    % 这个限制应该来说是不合理 可以考虑限制一个大点的距离 防止误报
                    range_cur = Xk(1);
                    vel_cur = Xk(2);
                    acc_cur = Xk(3);
                    horizon_range = Xk(4);
                    horizon_vel = Xk(5); % 横向速度大于0.8不报警（目前可能弯道就有问题）  % 还要考虑是否已经在减速
                    if(ttc<3.3 && ttc>0 && speed_cur>40/3.6 && abs(vision_horizon)<1.2 && abs(horizon_vel)<0.6 && vel_cur<-1)  %  && range_cur<35 
                        fcw_counter = fcw_counter + 1;
                        if(fcw_counter >= 5)
                            fcw_state = 1;
                        else
                            fcw_state = 0;
                        end
                    else
                        fcw_counter = 0;
                        fcw_state = 0;
                    end
                    
                  %% 保存数据
                    time_cur = image_timestamp_cur;
                    save_i_index = save_i_index + 1;
                    save_Xk(:, save_i_index) = [time_cur; Xk];
                    save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
                    save_ttc(:, save_i_index) = [time_cur; ttc ];
                    save_ttc_raw(:, save_i_index) = [time_cur; ttc_raw ];
                    save_dt_image(:, save_i_index) = [time_cur; dt_image];
                    save_fcw_state_mobileye(:, save_i_index) = [time_cur; mobileye_fcw_state];
                    save_fcw_state(:, save_i_index) = [time_cur;fcw_state ];
                    save_data_mobileye_tmp(:, save_i_index) = [time_cur; data_mobileye_tmp];    
                    save_index_of_target(:, save_i_index) = [time_cur; index_of_target];       
                    save_vision_horizon(:, save_i_index) = [time_cur; vision_horizon];   
                    save_speed_car(:, save_i_index) = [time_cur; speed_cur];                       
                    
                end        
            end

        %     figure('visible','off')    
            h = figure();
            % 车距
            ax1 = subplot(3,1,1);
            plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range 量测
            hold on;
            plot(save_Xk(1,:), save_Xk(2,:), '.'); % range-estimate
            grid on;
            legend({'vision-range-measure-raw','range-estimate'},'Location','northeast','FontSize',10);
            legend('boxoff')
            log_addr_t = log_addr;
            str_name = sprintf('log文件: %s \n 车距 ', log_addr_t);
            title(str_name);

            % 速度 & 加速度 & 横向距离 & 本车速度
            ax2 = subplot(3,1,2);
            plot(save_Xk(1,:), save_Xk(3, :), '.'); % vel estimation
            hold on;
            plot(save_Xk(1,:), save_Xk(4,:), '.'); % acc estimation
            plot(save_Xk(1,:), save_Xk(5,:)*10, '.'); % vision horizon
            plot(save_Xk(1,:), save_Xk(6,:)*10, '.'); % vision horizon vel
            plot(save_speed_car(1,:), save_speed_car(2,:), '.'); % speed-car
            grid on;
            legend({'vel-estimation','acc-estimation', 'vision-horizon*10', 'vision-horizon-vel*10', 'speed-car'},'Location','northeast','FontSize',10);
%             ylim([-30, 10]);
            legend('boxoff')
            title('速度&加速度&横向距离&本车速度');
            
            % ttc fcw
            ax3 = subplot(3,1,3);
            NUM1 = length(save_ttc);
            plot(save_ttc(1,:), save_ttc(2,:), '.'); % ttc
            hold on;
            plot(save_fcw_state(1,:), save_fcw_state_mobileye(2,:)*5); % fcw_mobileye
            plot(save_fcw_state(1,:), save_fcw_state(2,:)*5); % fcw_minieye
            NUM = length(save_ttc(1,:));
            plot(save_ttc(1,:), ones(1,NUM)*3.3)
%             plot(save_ttc(1,:), ones(1,NUM)*-4)
            grid on;
            legend({'ttc', 'fcw-mobileye','fcw-minieye', 'ttc'},'Location','northeast','FontSize',10);
            legend('boxoff')
            str_name = sprintf('ttc&fcw');
            ylim([-1, 10]);
            title(str_name);

            linkaxes([ax1,ax2,ax3], 'x'); % 同步子图的坐标轴
            
            % 保存figure
        %     filename=[log_addr_t, '--plot.png'];
        %     saveas(gcf,filename)
        %     close(gcf)    

            % 保存计算结果txt
        %     NUM = length(save_Xk);
        %     est_log_name = ['./data/est_result_new/', log_ID, '.log_KF_estimation.txt'];
        %     fp = fopen(est_log_name, 'wt');
        %     for i = 1:NUM
        %         fprintf(fp, '%d %f %f %f\n', save_Xk(1, i)/image_frame_T, save_Xk(2, i), save_Xk(3, i), save_Xk(4, i));
        %     end
        %     fclose(fp);
        % 
            clear save_Xk  save_relative_v save_ttc save_vision_raw save_fcw_state save_data_mobileye_tmp...
                save_index_of_target save_vision_horizon save_dt_image save_speed_car save_ttc_raw save_fcw_state_mobileye

        end
    end
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


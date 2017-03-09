% 显示人工标注的点
clc;
clear all; 
close all

SAVE_R = 0; % 保存R
SHOW_IPM = 1; % 是否用于IPM显示转弯半径 
%% 数据导入
% 转弯半径
source_addr = 'data/1013_快速变道/';

% 车道线特征点 左右两条，每条8个特征点
address_lane = [source_addr, 'lane_feature_122230_700_900.txt'];
lane_feature_raw_data = load(address_lane)'; % data_lane: lane index*1, 左右车道分别8个点
NUM_lane = length(lane_feature_raw_data(1, :)); % 样本数量
lane_feature_data.frame_index = lane_feature_raw_data(1, :);
for i = 1:8
    lane_feature_data.left_uv_feature(i, :, :) = lane_feature_raw_data(2*i:2*i+1, :);
    lane_feature_data.right_uv_feature(i, :, :) = lane_feature_raw_data(2*i+16:2*i+17, :);
end

for k_lane = 1:1:NUM_lane
    k = lane_feature_data.frame_index(k_lane);

    % 读取图片数据
    image_name = sprintf('/%08d.jpg',k);
    str_data = [source_addr, '122230_部分/',image_name];
    I_rgb = imread(str_data);
    I_g = rgb2gray(I_rgb);
    [m, n] = size(I_g);

    for i = 1:8
         u_L = lane_feature_data.left_uv_feature(i, 1, k_lane);
         v_L = lane_feature_data.left_uv_feature(i, 2, k_lane);
         u_R = lane_feature_data.right_uv_feature(i, 1, k_lane);
         v_R = lane_feature_data.right_uv_feature(i, 2, k_lane);

         value_t = 230;
         I_rgb(v_L+1:v_L+3, u_L+1:u_L+3) = value_t;

         value_t = 0;
         I_rgb(v_R+1:v_R+3, u_R+1:u_R+3) = value_t;

    end

    figure(1);
    str_name = sprintf('frame%d',k);
    title(str_name); 
    imshow(I_rgb);    
end
        


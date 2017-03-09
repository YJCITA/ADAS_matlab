clc 
clear all
% close all

addr_origin = 'data/1214_ADC/cap6/';
data_index = 0;
file_index = 0;
for i = 1:4    
    index = num2str(file_index);
    address = [addr_origin, 'adc_data_', index, '.txt'];
    fid = fopen(address,'r');
    % 读取数据    
    while ~feof(fid)
        data_index = data_index + 1;
        lineData = fgetl(fid);
        str_line_raw = regexp(lineData,'\t|\s','split'); %以空格为特征分割字符串
        for kk = 1:1
            data_raw_log(kk, data_index) = str2num(str_line_raw{1,kk});            
        end 
        
        % 分割时间
        str_time = regexp(str_line_raw{1,5}, ':', 'split');
        for kk = 1:3
            time_t(kk) = str2num(str_time{1,kk});
        end
        time_s(data_index) = time_t(1)*3600 + time_t(2)*60 + time_t(3);
    end
    
    figure()
    plot( time_s, data_raw_log*2);
    title(index)
    grid on;
    ylim([3.5,4.5])

    data_index = 0;  
    clear data_raw_log;
    clear time_s;

    file_index = file_index+1 
end







clc 
clear all
close all

addr_origin = 'data/AEB/';

index = [10, 20, 30, 40, 50, 60, 70, 80];
for j = 1:8
    index_str = num2str(index(j));
    data_name_addr = [addr_origin, 'WBS', index_str, '.mat'];
    load(data_name_addr);
    data_length = length(control_valid);
    is_first_time = 1;
    for i = 1:data_length
        if control_valid(i, 2) == 1
            if(is_first_time)
                time_start = control_valid(i, 1);
                is_first_time = 0;
            else
                time_end = control_valid(i, 1);
            end            
        end 
    end
    dt(1, j) = index(j);
    dt(2, j) = time_end - time_start;   
    
    length_vel = length(Velocity_Kmh);
    d = 0;
    for k = 1:length_vel
        if Velocity_Kmh(k, 1) >= time_start &&  Velocity_Kmh(k, 1) <= time_end
            dt_vel = Velocity_Kmh(k, 1) - Velocity_Kmh(k-1, 1);
            d = d + Velocity_Kmh(k, 2)/3.6*dt_vel;
        end
    end
    
    distance_car(1, j) = index(j);
    distance_car(2, j) = d;
end


figure()
plot( dt(1, :),  dt(2, :));
grid on;

figure()
plot( distance_car(1, :),  distance_car(2, :));
grid on;

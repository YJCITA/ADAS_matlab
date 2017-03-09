addr_origin = 'data/AEB/';
data_name_addr = [addr_origin, 'WBS', '30', '.mat'];
load(data_name_addr);

Velocity_Kmh


figure()
plot( control_valid(:, 1),  control_valid(:, 2));
grid on;
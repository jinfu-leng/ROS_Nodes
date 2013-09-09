# configure file for independent sensors in indoor environment
param_number_nodes = 6
param_ground_width = 10.0
param_ground_height = 10.0
param_network_type = 'homogeneous2'

param_node_power_capacity = 60
param_node_power_consumption_rate = 1
param_node_initial_power = param_node_power_capacity

param_UAV_power_capacity = 25.0 * 3600
param_UAV_flight_power_consumption_rate = 75.0
param_UAV_initial_power = param_UAV_power_capacity
param_UAV_charging_power_consumption_rate = 45.0
param_UAV_charging_power_transfer_rate = 0.3333
param_UAV_moving_speed = 0.5
param_UAV_charged_power_accumulation_rate = 0.0

param_UAV_initial_x = 0.0
param_UAV_initial_y = 0.0
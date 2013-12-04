# configure file for test environment
param_number_nodes = 20
param_ground_width = 2000.0
param_ground_height = 2000.0
param_network_type = 'homogeneous2'

param_node_power_capacity = 2.34 * 3600
param_node_power_consumption_rate = 0.1855
param_node_initial_power = param_node_power_capacity

param_UAV_power_capacity = 25.0 * 3600
param_UAV_flight_power_consumption_rate = 120
param_UAV_hovering_power_consumption_rate = 60
param_UAV_initial_power = param_UAV_power_capacity
param_UAV_charging_power_consumption_rate = 20
param_UAV_charging_power_transfer_rate = 0.2
param_UAV_localization_time = 6.0
param_UAV_moving_speed = 5.0
param_UAV_initial_x = param_ground_width / 2
param_UAV_initial_y = param_ground_height / 2
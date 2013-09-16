# configure file for independent sensors in outdoor environment
param_number_nodes = 20
param_ground_width = 100.0
param_ground_height = 100.0
param_network_type = 'homogeneous2'

param_node_power_capacity = 2.34 * 3600
param_node_power_consumption_rate = 0.1855
param_node_initial_power = param_node_power_capacity

param_UAV_power_capacity = 25.0 * 3600
param_UAV_flight_power_consumption_rate = 75.0
param_UAV_initial_power = param_UAV_power_capacity
param_UAV_charging_power_consumption_rate = 45.0
param_UAV_charging_power_transfer_rate = 0.3333
param_UAV_moving_speed = 5.0
param_UAV_charged_power_accumulation_rate = 2000.0 # if we supposed that the battery can be replaced, the recharging can be very fast

param_UAV_initial_x = -1.0 * param_UAV_moving_speed * 2.5 * 60 # make the distance between UAV base and wsn field a 2.5 minutes' flying
param_UAV_initial_y = 0.0
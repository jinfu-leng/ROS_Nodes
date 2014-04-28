# configure file for test environment
# Variables with None value are initialized in either start_experiments.py or object_manager.py
param_number_nodes = None
param_ground_width = None
param_ground_height = None
param_network_type = None

param_node_power_capacity = 2.34 * 3600
param_node_power_consumption_rate = param_node_power_capacity / (60 * 24 * 3600)
param_node_initial_power = param_node_power_capacity

param_UAV_power_capacity = None
param_UAV_flight_power_consumption_rate = 121.91
param_UAV_hovering_power_consumption_rate = None

param_UAV_charging_power_consumption_rate = None
param_UAV_charging_power_transfer_rate = None
param_UAV_localization_time = None
param_UAV_moving_speed = 7.33
param_UAV_base_distance = None
param_UAV_initial_x = None
param_UAV_initial_y = None

# Base station recharging only (not implemented yet)
param_UAV_accumulating_power_rate = 0
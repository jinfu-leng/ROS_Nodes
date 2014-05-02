import os, copy
from object_manager import ObjectManager
import UAV_AI


import test_config as config
param_number_nodes = [8] # nodes are randomly distributed, see object_manager.py
param_ground_size = [200]
param_UAV_localization_time = [36]
param_transfer_rate = [0.2]
param_UAV_hovering_power_consumption_rate = [92.28]
param_UAV_charging_power_consumption_rate = [20]
param_UAV_base_distance = [0]
param_UAV_power_capacity = [25.0 * 3600]
param_time_limit = 604800 * 10000 # the maximal round of simulation
# homogeneous: nodes have same initial energy
# homogeneous2: nodes have random initial energy (20%-60% of capacity)
param_network_type = ['homogeneous2']
param_charge_mode = ['random', 'to_full', 'to_initial_average', 'with_constant', 'to_optimized_one_flight']
param_path_mode = ['least_power', 'closest', 'shortest']
# If the energy percents of all the sensor nodes are above this threshold, the UAV would stay in the base (good for multi-flight experiment)
param_task_threshold = [0.8]
param_experiment_time = 100
param_res_file_name = 'center_test.csv'

is_single_flight = True

if is_single_flight == True:
	param_res_file_name = 'data_one_' + param_res_file_name
else:
	param_res_file_name = 'data_multi_' + param_res_file_name


def is_valid_combination(charge_mode, path_mode):
	if charge_mode == 'to_optimized_one_flight' and path_mode != 'least_power':
		return False
	if path_mode == 'least_power' and charge_mode != 'to_optimized_one_flight':
		return False
	return True

def is_valid_node_network(nodes):
	for node in nodes:
		if node['power'] <= 0:
			return False
	return True

def is_valid_UAV(UAV):
	if UAV['power'] <= 0:
		return False
	else:
		return True

def nodes_next_second(nodes):
	for node in nodes:
		node['power'] -= node['rate']

# create object manager to create system
object_manager_ = ObjectManager()

# write the first line of the result
res_file = open(param_res_file_name, 'w')

if is_single_flight:
	res_file.write('network_type,UAV_mode,round_number\n')
else:
	res_file.write('network_type,UAV_mode,flight_number\n')

# create the file
for experiment_time in range(param_experiment_time):
	print experiment_time
	for num in param_number_nodes:
		for size in param_ground_size:
			for localization_time in param_UAV_localization_time:
				for transfer_rate in param_transfer_rate: #transfer efficiency rate
					for net_type in param_network_type:
						for hovering_rate in param_UAV_hovering_power_consumption_rate:
							for charging_rate in param_UAV_charging_power_consumption_rate: #charging energy consumption rate
								for task_threshold in param_task_threshold:
									for base_distance in param_UAV_base_distance:
										for UAV_power_capacity in param_UAV_power_capacity:
											config.param_number_nodes = num
											config.param_ground_width = size
											config.param_ground_height = size
											config.param_UAV_localization_time = localization_time
											config.param_UAV_charging_power_transfer_rate = transfer_rate
											config.param_network_type = net_type
											config.param_UAV_hovering_power_consumption_rate = hovering_rate
											config.param_UAV_charging_power_consumption_rate = charging_rate
											config.param_UAV_base_distance = base_distance
											config.param_UAV_power_capacity = UAV_power_capacity
											network_type_str = str(num) + '_' + str(size) + '_' + str(size) \
												+ '_' + str(net_type) + '_' + str(localization_time) \
												+ '_' + str(transfer_rate) + '_' + str(task_threshold) \
												+ '_' + str(hovering_rate) + '_' + str(charging_rate) \
												+ '_' + str(base_distance) + '_' + str(UAV_power_capacity)
											print 'Network Type: ' + network_type_str							
											UAV_new, nodes_new = object_manager_.create_objects(config)

											# single flight
											if is_single_flight == True:
												# lower bound
												nodes = copy.deepcopy(nodes_new)
												lower_bound = UAV_AI.compute_lifetime_lower_bound(nodes)
												print 'Lifetime lower Bound', lower_bound
												res_file.write(network_type_str + ',' + 'lower_bound' + ',' + str(lower_bound) + '\n')	

												# start the experiment
												for charge_mode in param_charge_mode:
													for path_mode in param_path_mode:
														# filter invalid combination
														if is_valid_combination(charge_mode, path_mode) == False:
															continue
														UAV_mode = path_mode + '_' + charge_mode
														print 'UAV Mode: ' + UAV_mode
														UAV = copy.deepcopy(UAV_new)
														nodes = copy.deepcopy(nodes_new)
														# a hack for combination experiment
														# for node in nodes:
														# 	node['capacity'] = 100000000000 
														params = {}
														round_num = 0
														while is_valid_node_network(nodes) and is_valid_UAV(UAV) and round_num < param_time_limit:
															nodes_next_second(nodes)
															UAV_AI.next_second(UAV, nodes, charge_mode, path_mode, task_threshold, params)
															round_num += 1
														round_num += int(min(node['power'] for node in nodes) / nodes[0]['rate'])
														print 'The system is dead at round: ' + str(round_num)
														res_file.write(network_type_str + ',' + UAV_mode + ',' + str(round_num) + '\n')
											# multi flight
											else:
												for charge_mode in param_charge_mode:
													for path_mode in param_path_mode:
														# filter invalid combination
														if is_valid_combination(charge_mode, path_mode) == False:
															continue
														UAV_mode = path_mode + '_' + charge_mode
														print 'UAV Mode: ' + UAV_mode
														UAV = copy.deepcopy(UAV_new)
														nodes = copy.deepcopy(nodes_new)
														params = {}
														round_num = 0
														UAV['flight_number'] = 0						
														while is_valid_node_network(nodes) and is_valid_UAV(UAV) and round_num < param_time_limit:						
															nodes_next_second(nodes)
															UAV_AI.next_second(UAV, nodes, charge_mode, path_mode, task_threshold, params)
															round_num += 1
														
														if round_num == param_time_limit:
															print 'Number of flight: ' + str(UAV['flight_number'])
															res_file.write(network_type_str + ',' + UAV_mode + ',' + str(UAV['flight_number']) + '\n')
														else:
															print 'The system is dead at round: ' + str(round_num)
															res_file.write(network_type_str + ',' + UAV_mode + ',' + str(-1 * round_num) + '\n')	
															if is_valid_UAV(UAV) == False:
																print 'Error: The UAV is invalid!'
															
											print
res_file.close()

import os, copy
from object_manager import ObjectManager
import UAV_AI


import test_config as config
param_number_nodes = [5]
param_ground_size = [400]
param_localization_time = [40]
param_transfer_rate = [0.2]
param_time_limit = 604800
param_network_type = ['homogeneous2']
param_UAV_modes = []
param_UAV_modes += ['closest_to_full', 'hamiltonian_to_full', 'least_power_to_full']
#param_UAV_modes += ['closest_to_full', 'closest_with_precomputed_amount', 'closest_to_initial_average',]
#param_UAV_modes += ['hamiltonian_to_full','closest_to_full']
##param_UAV_modes += ['hamiltonian_to_average','closest_to_average']
#param_UAV_modes += ['hamiltonian_with_precomputed_amount','closest_with_precomputed_amount']
#param_UAV_modes += ['hamiltonian_to_initial_average','closest_to_initial_average']
params_task_threshold = [0.9]
param_experiment_time = 5
param_res_file_name = 'repeat_center_test.csv'

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
res_file.write('network_type,UAV_mode,flight_number\n')

# create the file
for num in param_number_nodes:
	for size in param_ground_size:
		for localization_time in param_localization_time:
			for transfer_rate in param_transfer_rate:
				for net_type in param_network_type:
					for task_threshold in params_task_threshold:
						config.param_number_nodes = num
						config.param_ground_width = size
						config.param_ground_height = size
						config.param_UAV_localization_time = localization_time
						config.param_transfer_rate = transfer_rate
						config.param_network_type = net_type
						network_type_str = str(num) + '_' + str(size) + '_' + str(size) \
							+ '_' + str(net_type) + '_' + str(localization_time) \
							+ '_' + str(transfer_rate) + '_' + str(task_threshold)
						print 'Network Type: ' + network_type_str
						for experiment_time in range(param_experiment_time):
							print experiment_time
							UAV_new, nodes_new = object_manager_.create_objects(config)
							
							# start the experiment
							for UAV_mode in param_UAV_modes:
								UAV = copy.deepcopy(UAV_new)
								nodes = copy.deepcopy(nodes_new)
								params = {}
								round_num = 0							
								while is_valid_node_network(nodes) and is_valid_UAV(UAV) and round_num < param_time_limit:						
									nodes_next_second(nodes)
									UAV_AI.next_second(UAV, nodes, UAV_mode, task_threshold, params)
									round_num += 1
								print 'UAV Mode: ' + UAV_mode
								if round_num == param_time_limit:
									print 'Number of flight: ' + str(UAV['flight_number'])
									res_file.write(network_type_str + ',' + UAV_mode + ',' + str(UAV['flight_number']) + '\n')
								else:
									print 'The system is dead at round: ' + str(round_num)
									res_file.write(network_type_str + ',' + UAV_mode + ',' + str(-1) + '\n')	
							print
res_file.close()

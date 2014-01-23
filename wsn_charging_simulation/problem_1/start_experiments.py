import os, copy
from object_manager import ObjectManager
import UAV_AI


import test_config as config
param_number_nodes = [5, 7, 9, 11, 13]
param_ground_size = [400]
param_localization_time = [40]
param_transfer_rate = [0.2]
param_time_limit = 604800
param_network_type = ['homogeneous2']
#param_charge_mode = ['to_full', 'with_constant_amount', 'with_individual_amount']
#param_path_mode = ['least_power', 'closest', 'hamiltonian']
param_charge_mode = ['with_individual_amount', 'to_full', 'with_constant_amount']
param_path_mode = ['least_power']
param_task_threshold = [0.8]
param_experiment_time = 2
param_res_file_name = 'repeat_center_nodenum_test.csv'

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
					for task_threshold in param_task_threshold:
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
							UAV_new, nodes_new = object_manager_.create_objects(config)
							
							# start the experiment
							for charge_mode in param_charge_mode:
								for path_mode in param_path_mode:
									UAV_mode = path_mode + '_' + charge_mode 
									UAV = copy.deepcopy(UAV_new)
									nodes = copy.deepcopy(nodes_new)
									params = {}
									round_num = 0
									UAV['flight_number'] = 0						
									while is_valid_node_network(nodes) and is_valid_UAV(UAV) and round_num < param_time_limit:						
										nodes_next_second(nodes)
										UAV_AI.next_second(UAV, nodes, charge_mode, path_mode, task_threshold, params)
										round_num += 1
									print 'UAV Mode: ' + UAV_mode
									if round_num == param_time_limit:
										print 'Number of flight: ' + str(UAV['flight_number'])
										if is_valid_UAV(UAV) == False:
											print '!!!!!!!!!!!!!!!!It is beacause of the UAV!!!!!!!!!!!!!!!!!!'
										res_file.write(network_type_str + ',' + UAV_mode + ',' + str(UAV['flight_number']) + '\n')
									else:
										print 'The system is dead at round: ' + str(round_num)
										res_file.write(network_type_str + ',' + UAV_mode + ',' + str(-1 * round_num) + '\n')	
							print
res_file.close()

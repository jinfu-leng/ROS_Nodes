import os, copy
from object_manager import ObjectManager
import UAV_AI


import test_config as config
param_number_nodes = [8]
param_ground_size = [100, 400]
param_localization_time = [10]
param_transfer_rate = [0.2, 0.3]
param_network_type = ['homogeneous2']
param_UAV_modes = ['closest_to_initial_average', 'hamiltonian_to_initial_average', 'closest_to_average', 'hamiltonian_to_average', 'closest_to_full', 'hamiltonian_to_full']
param_experiment_time = 20
param_res_file_name = 'center_real_data_test.csv'

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
res_file.write('network_type,UAV_mode,work_time\n')

# create the file
for num in param_number_nodes:
	for size in param_ground_size:
		for localization_time in param_localization_time:
			for transfer_rate in param_transfer_rate:
				for net_type in param_network_type:
					config.param_number_nodes = num
					config.param_ground_width = size
					config.param_ground_height = size
					config.param_UAV_localization_time = localization_time
					config.param_transfer_rate = transfer_rate
					config.param_network_type = net_type
					network_type_str = str(num) + '_' + str(size) + '_' + str(size) + '_' + str(net_type) + '_' + str(localization_time) + '_' + str(transfer_rate)
					print 'Network Type: ' + network_type_str
					for experiment_time in range(param_experiment_time):
						UAV_new, nodes_new = object_manager_.create_objects(config)
						# compute lifetime lower bound
						UAV = copy.deepcopy(UAV_new)
						nodes = copy.deepcopy(nodes_new)
						lower_bound = UAV_AI.compute_lifetime_lower_bound(UAV, nodes)
						print "Lifttime lower bound:", lower_bound
						res_file.write(network_type_str + ',' + 'lower_bound' + ',' + str(lower_bound) + '\n')
						# compute lifetime upper bound
						UAV = copy.deepcopy(UAV_new)
						nodes = copy.deepcopy(nodes_new)
						upper_bound = UAV_AI.compute_lifetime_upper_bound(UAV, nodes)
						print "Lifttime upper bound:", upper_bound
						res_file.write(network_type_str + ',' + 'upper_bound' + ',' + str(upper_bound) + '\n')
						# start the experiment
						for UAV_mode in param_UAV_modes:
							UAV = copy.deepcopy(UAV_new)
							nodes = copy.deepcopy(nodes_new)
							params = {}
							round_num = 0							
							while is_valid_node_network(nodes):						
								nodes_next_second(nodes)
								if is_valid_UAV(UAV):
									UAV_AI.next_second(UAV, nodes, UAV_mode, params)
								round_num += 1
							print 'UAV Mode: ' + UAV_mode
							print 'The system is dead at round: ' + str(round_num)
							res_file.write(network_type_str + ',' + UAV_mode + ',' + str(round_num) + '\n')
						print

res_file.close()

import os, copy
from object_manager import ObjectManager
import UAV_AI


import outdoor_faraway_config as config
param_number_nodes = [8]
param_ground_size = [100]
param_network_type = ['homogeneous2']
param_UAV_modes = ['all_to_full', 'below_average_to_average', 'below_average_to_full', 'cloeset_to_half']
param_experiment_time = 1
param_res_file_name = 'res_prob2_outdoor_faraway_11_15.csv'

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
res_file.write('UAV_method,number_nodes,ground_width,ground_height,network_type,work_time\n')

# create the file
for num in param_number_nodes:
	for size in param_ground_size:
		for net_type in param_network_type:
			config.param_number_nodes = num
			config.param_ground_width = size
			config.param_ground_height = size
			config.param_network_type = net_type
			for experiment_time in range(param_experiment_time):
				UAV_new, nodes_new = object_manager_.create_objects(config)
				# compute lifetime lower bound
				UAV = copy.deepcopy(UAV_new)
				nodes = copy.deepcopy(nodes_new)
				print "lifttime lower bound:", UAV_AI.compute_lifetime_lower_bound(UAV, nodes)
				# compute lifetime upper bound
				UAV = copy.deepcopy(UAV_new)
				nodes = copy.deepcopy(nodes_new)
				print "lifttime upper bound:", UAV_AI.compute_lifetime_upper_bound(UAV, nodes)		
				# start the experiment
				for UAV_mode in param_UAV_modes:
					UAV = copy.deepcopy(UAV_new)
					nodes = copy.deepcopy(nodes_new)
					output_line = str(UAV_mode) + ',' \
						+ str(num) + ',' + str(size) + ',' + str(size) + ',' \
						+ str(net_type)
					round_num = 0					
					print 'System config: ' + output_line
					while is_valid_node_network(nodes):
						
						nodes_next_second(nodes)
						if is_valid_UAV(UAV):
							UAV_AI.next_second(UAV, nodes, UAV_mode)
						round_num += 1
					output_line += ',' + str(round_num)
					print 'The system is died at round ' + str(round_num)
					print
					# write one record
					res_file.write(output_line + '\n')

res_file.close()
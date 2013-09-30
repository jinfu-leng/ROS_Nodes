import os, copy
from object_manager import ObjectManager
import UAV_AI


import outdoor_faraway_config as config
param_number_nodes = [10, 20]
param_ground_size = [100, 500]
param_network_type = ['homogeneous']
param_UAV_modes = ['least_power', 'least_power_partition']
param_time_limits = [604800]
param_res_file_name = 'res_outdoor_faraway.csv'


def get_all_file(dir_path, file_extension):
	file_list=os.listdir(dir_path)
	res_file_list=[]
	for file_name in file_list:
		if file_name.endswith(file_extension):
			res_file_list.append(file_name)
	return res_file_list

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
res_file.write('UAV_method,number_nodes,ground_width,ground_height,network_type,time_limit,number_task\n')

# create the file
for num in param_number_nodes:
	for size in param_ground_size:
		for net_type in param_network_type:
			config.param_number_nodes = num
			config.param_ground_width = size
			config.param_ground_height = size
			config.param_network_type = net_type
			UAV_new, nodes_new = object_manager_.create_objects(config)
			
			# start the experiment
			for UAV_mode in param_UAV_modes:
				for time_limit in param_time_limits:
					UAV = copy.deepcopy(UAV_new)
					nodes = copy.deepcopy(nodes_new)
					output_line = str(UAV_mode) + ',' + str(num) + ',' + str(size) + ',' + str(size) + ',' \
						+ str(net_type) + ',' + str(time_limit)
					round_num = 1
					
					print 'System config: ' + output_line
					while is_valid_node_network(nodes) and is_valid_UAV(UAV):
						if round_num == time_limit:
							print 'The system was valid during the past ' + str(time_limit) + ' seconds'
							print 'The charging task was conducted ' + str(UAV['task_num']) + ' times'
							output_line += ',' + str(UAV['task_num'])
							break
						round_num += 1
						nodes_next_second(nodes)
						UAV_AI.next_second(UAV, nodes, UAV_mode)
					else:
						print 'The system is not valid at round ' + str(round_num)
						output_line += ',' + str(-1)
					print
					# write one record
					res_file.write(output_line + '\n')

res_file.close()
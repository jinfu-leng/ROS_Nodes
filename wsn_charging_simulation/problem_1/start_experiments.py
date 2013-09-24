import os
from object_manager import ObjectManager
import UAV_AI


UAV_modes = ['least_power', 'least_power_k']
time_limits = [604800]

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

object_manager_ = ObjectManager()
object_files = get_all_file('.', '.csv')
for object_file in object_files:
	for UAV_mode in UAV_modes:
		for time_limit in time_limits:
			round_num = 1
			UAV, nodes = object_manager_.read_objects(object_file)
			print 'UAV mode: ' + UAV_mode
			print 'System data: ' + object_file
			while is_valid_node_network(nodes) and is_valid_UAV(UAV):
				if round_num == time_limit:
					print 'The system was valid during the past ' + str(time_limit) + ' seconds'
					print 'The charging task was conducted ' + str(UAV['task_num']) + ' times'
					break
				round_num += 1
				nodes_next_second(nodes)
				UAV_AI.next_second(UAV, nodes, UAV_mode)
			else:
				print 'The system is not valid at round ' + str(round_num)
			print
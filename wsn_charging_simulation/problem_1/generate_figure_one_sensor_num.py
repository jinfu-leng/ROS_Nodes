import numpy as np
import matplotlib.pyplot as plt

input_file_name = 'data_one_center_sensor_num_2.csv'

def UAV_mode_label_matcher(UAV_mode):
	matcher = {}
	matcher['lower_bound'] = 'NO'
	matcher['closest_to_full'] = 'FULL'
	matcher['hamiltonian_to_full'] = 'FULL*'
	matcher['closest_with_constant'] = 'FIX'
	matcher['hamiltonian_with_constant'] = 'FIX*'
	matcher['closest_to_initial_average'] = 'AVG'
	matcher['hamiltonian_to_initial_average'] = 'AVG*'
	matcher['closest_random'] = 'RND'
	matcher['hamiltonian_random'] = 'RND*'
	matcher['least_power_to_optimized_one_flight'] = 'LEAST'
	return matcher[UAV_mode]


def draw_bar_err_group_figure(nodes):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)
	bar_width = 1.0 / (label_cnt + 1)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:int(node))
	group_cnt = len(group_list)
	left_coordinates = np.arange(group_cnt)

	colors = 'bgrcmyk'
	ax = plt.subplot(111)
	for i in range(len(label_list)):
		label = label_list[i]
		candidate_nodes = [node for node in nodes if node['label'] == label]
		candidate_nodes = sorted(candidate_nodes, key=lambda node:node['group'])
		value = [node['value'] / (24 * 3600) for node in candidate_nodes]
		error = [node['error'] / (24 * 3600) for node in candidate_nodes]
		ax.bar(left_coordinates + i * bar_width, value, bar_width,
		label=UAV_mode_label_matcher(label), color=colors[i])
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel('Sensor Node Number')
	plt.ylabel('Lifetime (day)')
	plt.title('Lifetime by Sensor Node Numbers and Algorithms')
	plt.xticks(left_coordinates + 0.5, group_list)

def draw_normalized_bar_err_group_figure(nodes):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)
	bar_width = 1.0 / (label_cnt + 1)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:int(node))
	group_cnt = len(group_list)
	left_coordinates = np.arange(group_cnt)

	colors = 'bgrcmyk'
	ax = plt.subplot(111)
	base_nodes = [node for node in nodes if node['label'] == label_list[0]]
	base_nodes = sorted(base_nodes, key=lambda node:node['group'])
	for i in range(len(label_list)):
		label = label_list[i]
		candidate_nodes = [node for node in nodes if node['label'] == label]
		candidate_nodes = sorted(candidate_nodes, key=lambda node:node['group'])
		value = [node['value'] / base['value'] for node, base in zip(candidate_nodes, base_nodes)]
		error = [node['error'] / base['error'] for node, base in zip(candidate_nodes, base_nodes)]
		ax.bar(left_coordinates + i * bar_width, value, bar_width,
		label=UAV_mode_label_matcher(label), color=colors[i])
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel('Sensor Node Number')
	plt.ylabel('Normalized Lifetime')
	plt.title('Lifetime by Sensor Node Numbers and Algorithms')
	plt.xticks(left_coordinates + 0.5, group_list)


input_file = open(input_file_name, 'r')
first_line = input_file.readline()

res = {}
UAV_modes = {}
network_types = {}
for line in input_file.readlines():
	line_split = line[:-1].split(',')
	network_type = line_split[0]
	UAV_mode = line_split[1]
	lifetime = int(line_split[2])

	if network_type not in res:
		res[network_type] = {}
	if UAV_mode not in res[network_type]:
		res[network_type][UAV_mode] = []

	if UAV_mode not in UAV_modes:
		UAV_modes[UAV_mode] = True
	if network_type not in network_types:
		network_types[network_type] = True
		
	res[network_type][UAV_mode].append(lifetime)
input_file.close()

# generate average lifetime figures
nodes = []
for network_type in network_types:
	group = int(network_type.split('_')[0]) # sensor number
	for UAV_mode in res[network_type].keys():
		node = {}
		node['label'] = UAV_mode
		node['value'] = np.mean(res[network_type][UAV_mode])
		node['error'] = np.std(res[network_type][UAV_mode])
		node['group'] = group
		nodes.append(node)

draw_normalized_bar_err_group_figure(nodes)
plt.show()




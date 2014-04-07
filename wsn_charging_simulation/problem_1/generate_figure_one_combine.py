import numpy as np
import matplotlib.pyplot as plt
import generate_figure_common as fm

input_file_name = 'data_one_center_combine.csv'

input_file = open(input_file_name, 'r')
first_line = input_file.readline()

res = {}
UAV_modes = {}
valid_network_types = []
# basic
valid_network_types.append('8_200_200_homogeneous2_36_0.2_0.8_92.28_20_0_90000.0')
# localization time
valid_network_types.append('8_200_200_homogeneous2_0_0.2_0.8_92.28_20_0_90000.0')
# UAV energy capacity
valid_network_types.append('8_200_200_homogeneous2_36_0.2_0.8_92.28_20_0_720000.0')
# hovering consumption
valid_network_types.append('8_200_200_homogeneous2_36_0.2_0.8_0_20_0_90000.0')
# transfer consumption
valid_network_types.append('8_200_200_homogeneous2_36_0.2_0.8_92.28_80_0_90000.0')
# charging efficiency
valid_network_types.append('8_200_200_homogeneous2_36_0.8_0.8_92.28_20_0_90000.0')
# combine
valid_network_types.append('8_200_200_homogeneous2_0_0.8_0.8_0_80_0_720000.0')

network_type_name_matcher = {}
network_type_name_matcher['8_200_200_homogeneous2_36_0.2_0.8_92.28_20_0_90000.0'] = 'Basic'
network_type_name_matcher['8_200_200_homogeneous2_0_0.2_0.8_92.28_20_0_90000.0'] = 'Localization'
network_type_name_matcher['8_200_200_homogeneous2_36_0.2_0.8_92.28_20_0_720000.0'] = 'Capacity'
network_type_name_matcher['8_200_200_homogeneous2_36_0.2_0.8_0_20_0_90000.0'] = 'Hovering'
network_type_name_matcher['8_200_200_homogeneous2_36_0.2_0.8_92.28_80_0_90000.0'] = 'Transfer'
network_type_name_matcher['8_200_200_homogeneous2_36_0.8_0.8_92.28_20_0_90000.0'] = 'Efficiency'
network_type_name_matcher['8_200_200_homogeneous2_0_0.8_0.8_0_80_0_720000.0'] = 'Combined'

existing_network_types = []
for line in input_file.readlines():

	line_split = line[:-1].split(',')
	network_type = line_split[0]
	UAV_mode = line_split[1]
	lifetime = int(line_split[2])

	if UAV_mode != 'least_power_to_optimized_one_flight':
		continue
	if network_type not in valid_network_types:
		continue
	if network_type not in res:
		res[network_type] = []
		existing_network_types.append(network_type)
		
	res[network_type].append(lifetime)
input_file.close()

nodes = []
print valid_network_types
for network_type in valid_network_types:
	if network_type not in existing_network_types:
		continue
	label = network_type_name_matcher[network_type]
	node = {}
	node['value'] = np.mean(res[network_type])
	node['error'] = np.std(res[network_type])
	node['label'] = label
	nodes.append(node)



bar_width = 0.66
nodes_cnt = len(nodes)
for index in range(nodes_cnt):
	value = nodes[index]['value'] / (24 * 3600)
	error = nodes[index]['error'] / (24 * 3600)	
	plt.bar(index + 0.25 * bar_width, value, bar_width, color = 'c')	
	plt.xlabel('System Parameter')
	plt.ylabel('Lifetime (day)')
	#plt.title('Lifetime by Individual System Parameter Changes')
	plt.title('Lifetime by Individual and Combined System Parameter Changes')
	plt.xticks(np.arange(nodes_cnt) + 0.75 * bar_width, [node['label'] for node in nodes])
	plt.legend()
plt.show()
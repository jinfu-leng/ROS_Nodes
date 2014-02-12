import numpy as np
import matplotlib.pyplot as plt
import generate_figure_common as fm

input_file_name = 'data_one_center_hovering_rate.csv'

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
	group = float(network_type.split('_')[7]) # hovering rate
	for UAV_mode in res[network_type].keys():
		node = {}
		node['label'] = UAV_mode
		node['group'] = group
		node['value'] = np.mean(res[network_type][UAV_mode])
		node['error'] = np.std(res[network_type][UAV_mode])
		nodes.append(node)

fm.draw_bar_err_label_figure(nodes, 'Algorithms', 'Lifetime (day)',
	'Lifetime by Hovering Energy Consumption Rate and Algorithms', True)
plt.show()

fm.draw_bar_err_label_figure_normalized(nodes, 'Algorithms', 'Normalized Lifetime',
	'Normalized Lifetime by Hovering Energy Consumption Rate and Algorithms', True)
plt.show()
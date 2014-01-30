import numpy as np
import matplotlib.pyplot as plt


def UAV_mode_label_matcher(UAV_mode):
	matcher = {}
	matcher['lower_bound'] = 'No Charge'
	matcher['closest_to_full'] = 'FULL'
	matcher['hamiltonian_to_full'] = 'FULL*'
	matcher['closest_with_constant'] = 'FIX'
	matcher['hamiltonian_with_constant'] = 'FIX*'
	matcher['closest_to_initial_average'] = 'AVG'
	matcher['hamiltonian_to_initial_average'] = 'AVG*'
	matcher['least_power_to_optimized_one_flight'] = 'LEAST**'
	return matcher[UAV_mode]


def draw_bar_err_figure(nodes):
	bar_width = 0.66
	colors = 'bgrcmyk'

	nodes = sorted(nodes, key=lambda node:int(node['value']))
	nodes_cnt = len(nodes)

	for index in range(nodes_cnt):
		value = nodes[index]['value'] / (24 * 3600)
		error = nodes[index]['error'] / (24 * 3600)
		plt.bar(index + 0.25 * bar_width, value, bar_width, color = 'c',
			yerr = error, ecolor = 'r')
	
	plt.xlabel('Algorithms')
	plt.ylabel('Lifetime (day)')
	plt.title('Lifetime by Algorithms')
	plt.xticks(np.arange(nodes_cnt) + 0.75 * bar_width, [UAV_mode_label_matcher(node['label']) for node in nodes])
	plt.legend()



input_file_name = 'one_center_ground_size.csv'
input_file = open(input_file_name, 'r')
first_line = input_file.readline()

res = {}
UAV_modes = {}
network_types = {}
for line in input_file.readlines():
	line_split = line[:-1].split(',')
	network_type = line_split[0]
	UAV_mode = line_split[1]
	lifetime = abs(int(line_split[2]))

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
for network_type in network_types:
	print network_type
	res_avg_lifetime = {}
	nodes = []
	for UAV_mode in res[network_type].keys():
		node = {}
		node['label'] = UAV_mode
		node['value'] = np.mean(res[network_type][UAV_mode])
		node['error'] = np.std(res[network_type][UAV_mode])
		nodes.append(node)
	draw_bar_err_figure(nodes)
	plt.show()




from object_manager import ObjectManager
import UAV_AI as UAVAI
import test_config as config
import matplotlib.pyplot as plt
import numpy as np

output_path = '/Users/jinfu/Desktop/Defense/'
save = True

def compute_to_optimized_one_flight_demo(UAV, nodes, path_mode):
	res = []
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	node_num = len(sorted_nodes)
	best_node_target_power = 0
	initial_total_power = UAV['power']
	for charge_node_num in range(1, node_num + 1):
		total_power = initial_total_power
		dist = UAVAI.compute_total_cycle_distance(UAV, sorted_nodes, range(0, charge_node_num)) #least power path
		flight_power = (dist / UAV['speed']) * UAV['flight_power_rate']
		total_power -=  flight_power
		localization_power = charge_node_num * UAV['localization_time'] * UAV['hovering_power_rate']
		total_power -= localization_power
		hover_power = total_power * UAV['hovering_power_rate'] / (UAV['hovering_power_rate'] + UAV['charging_power_rate'])
		total_power -= hover_power
		transfer_overhead_power = total_power * (1.0 - UAV['transfer_rate'])
		total_power -= transfer_overhead_power
		node_target_power = UAVAI.compute_max_min(total_power, [node['power'] for node in sorted_nodes[0: charge_node_num]])
		lifetime = node_target_power
		if charge_node_num < node_num:
			lifetime = min(node_target_power, sorted_nodes[charge_node_num]['power'])
		res.append([charge_node_num, node_target_power, flight_power, localization_power, hover_power, transfer_overhead_power, total_power, lifetime])
	return res

def generate_figure_demo_least(UAV, nodes, information_node):
	charge_node_num = information_node[0]
	node_target_power = information_node[1]
	flight_power = information_node[2]
	localization_power = information_node[3]
	hover_power = information_node[4]
	transfer_overhead_power = information_node[5]
	total_power = information_node[6]
	lifetime = information_node[7]
	initial_total_power = flight_power + localization_power + hover_power + transfer_overhead_power + total_power

	bar_width = 0.66
	colors = 'bgrcmyk'
	nodes = sorted(nodes, key=lambda node:float(node['power']))
	nodes_cnt = len(nodes)

	for index in range(nodes_cnt):
		node_energy = nodes[index]['power']
		if index < charge_node_num:
			plt.bar(index + 0.25 * bar_width, node_target_power, bar_width, color = 'b')
		plt.bar(index + 0.25 * bar_width, node_energy, bar_width, color = 'c')

	plt.bar(nodes_cnt + 0.25 * bar_width, initial_total_power, bar_width, color = 'b', label = 'Efficient Energy')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power + hover_power + transfer_overhead_power, bar_width, color = 'g', label = 'Charging Overhead')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power + hover_power, bar_width, color = 'y', label = 'Hovering Consumption')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power, bar_width, color = 'k', label = 'Localization Consumption')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power, bar_width, color = 'm', label = 'Flight Consumption')

	plt.plot([0, nodes_cnt], [lifetime, lifetime], 'r--', lw = 2)
	plt.ylim([0,90000])

	plt.xlabel('Nodes and UAV')
	plt.ylabel('Energy (J)')
	title = 'Schedule to Charge ' + str(charge_node_num) + ' Node(s)'
	plt.title(title)
	labels = range(nodes_cnt)
	labels.append('UAV')
	plt.xticks(np.arange(nodes_cnt + 1) + 0.75 * bar_width, labels)
	plt.legend(loc = 2)
	if save == True:
		file_path = output_path + 'least_' + str(charge_node_num)
		plt.savefig(file_path)
	plt.show()

def generate_figure_demo_least_nodes(UAV, nodes, information_node):
	charge_node_num = information_node[0]
	node_target_power = information_node[1]
	flight_power = information_node[2]
	localization_power = information_node[3]
	hover_power = information_node[4]
	transfer_overhead_power = information_node[5]
	total_power = information_node[6]
	lifetime = information_node[7]
	initial_total_power = flight_power + localization_power + hover_power + transfer_overhead_power + total_power

	bar_width = 0.66
	colors = 'bgrcmyk'
	nodes = sorted(nodes, key=lambda node:float(node['power']))
	nodes_cnt = len(nodes)

	for index in range(nodes_cnt):
		node_energy = nodes[index]['power']
		if index < charge_node_num:
			plt.bar(index + 0.25 * bar_width, node_target_power, bar_width, color = 'b')
		plt.bar(index + 0.25 * bar_width, node_energy, bar_width, color = 'c')

	plt.plot([0, nodes_cnt], [lifetime, lifetime], 'r--', lw = 2)
	plt.ylim([0,8000])

	plt.xlabel('Nodes')
	plt.ylabel('Energy (J)')
	title = 'Schedule to Charge ' + str(charge_node_num) + ' Node(s)'
	plt.title(title)
	labels = range(nodes_cnt)
	plt.xticks(np.arange(nodes_cnt + 1) + 0.75 * bar_width, labels)
	plt.legend(loc = 2)
	if save == True:
		file_path = output_path + 'least_nodes_' + str(charge_node_num)
		plt.savefig(file_path)
	plt.show()

def generate_figure_demo_least_UAV(UAV, nodes, information_node):
	charge_node_num = information_node[0]
	node_target_power = information_node[1]
	flight_power = information_node[2]
	localization_power = information_node[3]
	hover_power = information_node[4]
	transfer_overhead_power = information_node[5]
	total_power = information_node[6]
	lifetime = information_node[7]
	initial_total_power = flight_power + localization_power + hover_power + transfer_overhead_power + total_power

	ax = plt.subplot(111)

	bar_width = 0.5

	ax.bar(0.5 * bar_width, initial_total_power, bar_width, color = 'b', label = 'Efficient Energy')
	ax.bar(0.5 * bar_width, flight_power + localization_power + hover_power + transfer_overhead_power, bar_width, color = 'g', label = 'Charging Overhead')
	ax.bar(0.5 * bar_width, flight_power + localization_power + hover_power, bar_width, color = 'y', label = 'Hovering Consumption')
	ax.bar(0.5 * bar_width, flight_power + localization_power, bar_width, color = 'k', label = 'Localization Consumption')
	ax.bar(0.5 * bar_width, flight_power, bar_width, color = 'm', label = 'Flight Consumption')

	# Shink current axis
	box = ax.get_position()
	ax.set_position([box.x0 + 0.1, box.y0, box.width * 0.10, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.ylim([0,90000])
	plt.xlabel('UAV')
	plt.ylabel('Energy (J)')
	title = 'Schedule to Charge ' + str(charge_node_num) + ' Node(s)'
	#plt.title(title)
	plt.text(0.5, 92000, title, horizontalalignment = 'center')
	plt.xticks(np.arange(1) + 0.75 * bar_width, '')
	if save == True:
		file_path = output_path + 'least_UAV' + str(charge_node_num)
		plt.savefig(file_path)
	plt.show()



object_manager_ = ObjectManager()
UAV, nodes = object_manager_.create_objects(config)
res = compute_to_optimized_one_flight_demo(UAV, nodes, 'least')
for information_node in res:
	print information_node
	generate_figure_demo_least(UAV, nodes, information_node)
	generate_figure_demo_least_nodes(UAV, nodes, information_node)
	generate_figure_demo_least_UAV(UAV, nodes, information_node)
	#break



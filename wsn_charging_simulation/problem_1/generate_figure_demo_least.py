from object_manager import ObjectManager
import UAV_AI as UAVAI
import test_config as config
import matplotlib.pyplot as plt
import numpy as np

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

	plt.bar(nodes_cnt + 0.25 * bar_width, initial_total_power, bar_width, color = 'b')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power + hover_power + transfer_overhead_power, bar_width, color = 'g')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power + hover_power, bar_width, color = 'r')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power + localization_power, bar_width, color = 'k')
	plt.bar(nodes_cnt + 0.25 * bar_width, flight_power, bar_width, color = 'm')

	plt.xlabel('Node ID')
	plt.ylabel('Energy')
	title = 'Schedule to Charge ' + str(charge_node_num) + ' Node(s)'
	plt.title(title)
	plt.xticks(np.arange(nodes_cnt) + 0.75 * bar_width, range(nodes_cnt))
	plt.legend()
	plt.show()


	#plot UAV energy distribution





object_manager_ = ObjectManager()
UAV, nodes = object_manager_.create_objects(config)
res = compute_to_optimized_one_flight_demo(UAV, nodes, 'least')
for information_node in res:
	print information_node
	generate_figure_demo_least(UAV, nodes, information_node)
	



import numpy as np
import matplotlib.pyplot as plt

def UAV_mode_label_matcher(UAV_mode):
	matcher = {}
	matcher['lower_bound'] = 'NO'
	matcher['closest_to_full'] = 'FULL'
	matcher['shortest_to_full'] = 'FULL*'
	matcher['closest_with_constant'] = 'FIX'
	matcher['shortest_with_constant'] = 'FIX*'
	matcher['closest_to_initial_average'] = 'AVG'
	matcher['shortest_to_initial_average'] = 'AVG*'
	matcher['closest_random'] = 'RND'
	matcher['shortest_random'] = 'RND*'
	matcher['least_power_to_optimized_one_flight'] = 'LEAST'
	return matcher[UAV_mode]


def draw_bar_err_figure(nodes, xlabel, ylabel, title):
	bar_width = 0.66
	colors = 'bgrcmyk'

	nodes = sorted(nodes, key=lambda node:int(node['value']))
	nodes_cnt = len(nodes)

	for index in range(nodes_cnt):
		value = nodes[index]['value'] / (24 * 3600)
		error = nodes[index]['error'] / (24 * 3600)
		plt.bar(index + 0.25 * bar_width, value, bar_width, color = 'c',
			yerr = error, ecolor = 'r')
	
	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.xticks(np.arange(nodes_cnt) + 0.75 * bar_width, [UAV_mode_label_matcher(node['label']) for node in nodes])
	plt.legend()


def draw_bar_err_group_figure(nodes, xlabel, ylabel, title):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)
	bar_width = 1.0 / (label_cnt + 1)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:float(node))
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
		label = UAV_mode_label_matcher(label), color = colors[i],
		yerr = error, ecolor = 'k')
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.xticks(left_coordinates + 0.5, group_list)


def draw_normalized_bar_err_group_figure(nodes, xlabel, ylabel, title):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)
	bar_width = 1.0 / (label_cnt + 1)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:float(node))
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
		error = [node['error'] / base['value'] for node, base in zip(candidate_nodes, base_nodes)]
		ax.bar(left_coordinates + i * bar_width, value, bar_width,
		label=UAV_mode_label_matcher(label), color=colors[i],
		yerr = error, ecolor = 'k')
		print 'label', label
		print 'value', value
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.xticks(left_coordinates + 0.5, group_list)

def label_index(label):
	label_list = ['lower_bound', 'closest_to_full', 'shortest_to_full', 'closest_random', 'shortest_random',
		'closest_with_constant', 'shortest_with_constant', 'closest_to_initial_average', 'shortest_to_initial_average',
		'least_power_to_optimized_one_flight']
	for i in range(len(label_list)):
		if label == label_list[i]:
			return i
	return -1

def draw_bar_err_label_figure(nodes, xlabel, ylabel, title, group_list_reverse = False):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:float(node), reverse = group_list_reverse)
	group_cnt = len(group_list)

	bar_width = 1.0 / (group_cnt + 1)
	left_coordinates = np.arange(label_cnt)

	colors = 'bgrcmyk'
	ax = plt.subplot(111)
	for i in range(group_cnt):
		group = group_list[i]
		candidate_nodes = [node for node in nodes if node['group'] == group]
		candidate_nodes = sorted(candidate_nodes, key=lambda node:label_index(node['label']))
		value = [node['value'] / (24 * 3600) for node in candidate_nodes]
		error = [node['error'] / (24 * 3600) for node in candidate_nodes]
		ax.bar(left_coordinates + i * bar_width, value, bar_width,
		label = str(group) + 'W', color = colors[i],
		yerr = error, ecolor = 'k')
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.xticks(left_coordinates + bar_width * group_cnt * 0.5, [UAV_mode_label_matcher(label) for label in label_list])

def draw_bar_err_label_figure_normalized(nodes, xlabel, ylabel, title, group_list_reverse = False):
	label_list = ['lower_bound', 'closest_to_full', 'closest_random',
		'closest_with_constant', 'closest_to_initial_average', 'least_power_to_optimized_one_flight']

	label_cnt = len(label_list)

	group_set = set([node['group'] for node in nodes])
	group_list = list(group_set)
	group_list = sorted(group_list, key=lambda node:float(node), reverse = group_list_reverse)
	group_cnt = len(group_list)

	bar_width = 1.0 / (group_cnt + 1)
	left_coordinates = np.arange(label_cnt)

	colors = 'bgrcmyk'
	ax = plt.subplot(111)
	base_nodes = [node for node in nodes if node['group'] == group_list[0]]
	base_nodes = sorted(base_nodes, key=lambda node:label_index(node['label']))
	for i in range(group_cnt):
		group = group_list[i]
		candidate_nodes = []
		candidate_nodes = [node for node in nodes if node['group'] == group]
		candidate_nodes = sorted(candidate_nodes, key=lambda node:label_index(node['label']))
		value = [node['value'] / base['value'] for node, base in zip(candidate_nodes, base_nodes)]
		error = [node['error'] / base['value'] for node, base in zip(candidate_nodes, base_nodes)]
		ax.bar(left_coordinates + i * bar_width, value, bar_width,
		label = str(group) + 'W', color = colors[i],
		yerr = error, ecolor = 'k')
	
	# Shink current axis by 20%
	box = ax.get_position()
	ax.set_position([box.x0, box.y0, box.width * 0.85, box.height])

	# Put a legend to the right of the current axis
	ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))

	plt.xlabel(xlabel)
	plt.ylabel(ylabel)
	plt.title(title)
	plt.xticks(left_coordinates + bar_width * group_cnt * 0.5, [UAV_mode_label_matcher(label) for label in label_list])
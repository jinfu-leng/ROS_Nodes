import math, copy
import itertools

def euclidean_distance(x, y, x2, y2):
	return math.sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2))

def next_position(current_x, current_y, dest_x, dest_y, speed):
	x_diff = dest_x - current_x
	y_diff = dest_y - current_y
	dist_diff = math.sqrt(x_diff * x_diff + y_diff * y_diff)
	moving_ratio = 0
	if dist_diff > 0:
		moving_ratio = min(1, speed / dist_diff)
	return current_x + moving_ratio * x_diff, current_y + moving_ratio * y_diff

def is_UAV_able_back_home_after_next_second(UAV):
	next_power = UAV['power'] - UAV['flight_power_rate']
	if UAV['status'] == 'charging':
		next_power -= UAV['charging_power_rate']
	consume_rate = UAV['flight_power_rate']	
	distance_home = euclidean_distance(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'])
	speed = UAV['speed']
	return int(next_power/consume_rate) > (int(distance_home/speed) + 1)

def is_UAV_at_home(UAV):
	if UAV['current_x'] == UAV['home_x'] and UAV['current_y'] == UAV['home_y']:
		return True
	else:
		return False

def compute_total_cycle_distance(UAV, nodes, path):
	nodes_cnt = len(path)
	total_distance = 0.0
	for index in range(nodes_cnt-1):
		total_distance += euclidean_distance(nodes[path[index]]['x'], nodes[path[index]]['y'], nodes[path[index + 1]]['x'], nodes[path[index + 1]]['y'])
	total_distance += euclidean_distance(UAV['current_x'], UAV['current_y'], nodes[path[0]]['x'], nodes[path[0]]['y'])
	total_distance += euclidean_distance(UAV['current_x'], UAV['current_y'], nodes[path[nodes_cnt - 1]]['x'], nodes[path[nodes_cnt - 1]]['y'])
	return total_distance

def hamiltonian_node_path(UAV, nodes):
	nodes_cnt = len(nodes)
	hamiltonian_path = []
	hamiltonian_dist = 1000000000.0
	for path in itertools.permutations(range(nodes_cnt)):
		current_dist = compute_total_cycle_distance(UAV, nodes, path)
		if current_dist < hamiltonian_dist:
			hamiltonian_dist = current_dist
			hamiltonian_path = path
	return list(hamiltonian_path)

def closest_node_path(UAV, nodes):
	nodes_cnt = len(nodes)	
	usage_nodes = [False for i in range(nodes_cnt)]
	current_x = UAV['current_x']
	current_y = UAV['current_y']
	closest_path = []
	for i in range(nodes_cnt):
		best_node = -1
		best_dist = 0.0
		for j in range(nodes_cnt):
			if usage_nodes[j] == True:
				continue
			dist = euclidean_distance(current_x, current_y, nodes[j]['x'], nodes[j]['y'])
			if best_node < 0 or dist < best_dist:				
				best_dist = dist
				best_node = j
		current_x = nodes[best_node]['x']
		current_y = nodes[best_node]['y']
		closest_path.append(best_node)
		usage_nodes[best_node] = True
	return closest_path

def least_power_node_path(UAV, nodes):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	least_power_path = [node['id'] for node in sorted_nodes]
	return least_power_path


def get_average_power_nodes(nodes):
	return sum([node['power'] for node in nodes]) / len(nodes)

def is_threshold_triggered(nodes, threshold):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	if sorted_nodes[0]['power']/sorted_nodes[0]['capacity'] <= threshold:
		return True
	else:
		return False

def next_second_dest_list(UAV, nodes, charge_mode = 'to_goal', path_mode = 'least_power', threshold = 0.5, params = {}):
	if UAV['status'] != 'back' and UAV['status'] != 'accumulating' and is_UAV_able_back_home_after_next_second(UAV) == False:
		UAV['status'] = 'back'

	if UAV['status'] == 'accumulating':
		UAV['power'] = min(UAV['accumulating_power_rate'] + UAV['power'], UAV['capacity'])
		if UAV['power'] == UAV['capacity'] and is_threshold_triggered(nodes, threshold):
			UAV['flight_number'] += 1
			UAV['status'] = 'idle'

	if UAV['status'] == 'back':
		next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'], UAV['speed'])
		UAV['current_x'] = next_x
		UAV['current_y'] = next_y
		if UAV['current_x'] == UAV['home_x'] and UAV['current_y'] == UAV['home_x']:
			UAV['status'] = 'accumulating'

	if UAV['status'] == 'idle':
		if path_mode == 'closest':
			UAV['dest_list'] = closest_node_path(UAV, nodes)
		elif path_mode == 'hamiltonian':
			UAV['dest_list'] = hamiltonian_node_path(UAV, nodes)
		elif path_mode == 'least_power':
			UAV['dest_list'] = least_power_node_path(UAV, nodes)
		else:
			print 'Error path planning algorithm'
		UAV['status'] = 'looking'

	if UAV['status'] == 'looking':
		if len(UAV['dest_list']) > 0:
			UAV['dest_node_id'] = UAV['dest_list'][0]
			UAV['dest_list'].pop(0)
			UAV['status'] = 'moving'
		else:
			UAV['status'] = 'idle'

	if UAV['status'] == 'moving':
		if nodes[UAV['dest_node_id']]['x'] == UAV['current_x'] and nodes[UAV['dest_node_id']]['y'] == UAV['current_y']:
			UAV['status'] = 'localization'
			UAV['localization_time_left'] = UAV['localization_time']
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y

	if UAV['status'] == 'localization':
		#if mode == 'to_average':
		#	if nodes[UAV['dest_node_id']]['power'] >= get_average_power_nodes(nodes):
		#		UAV['status'] = 'looking'				
		#elif mode == 'below_average_to_full':
		#	if nodes[UAV['dest_node_id']]['power'] >= nodes[UAV['dest_node_id']]['capacity']:
		#		UAV['status'] = 'looking'
		#elif mode == 'to_goal':
		#	if nodes[UAV['dest_node_id']]['power'] >= params['goal']:
		#		UAV['status'] = 'looking'

		if UAV['localization_time_left'] <= 0:
			UAV['status'] ='charging'
			if charge_mode == 'with_fixed_amount':
				UAV['with_fixed_amount_left'] = params['fixed_amount']
			elif charge_mode =='with_individual_amount':
				if len(params['individual_amount']) == 0:
					UAV['status'] == 'back'
				else:
					UAV['with_individual_amount_left'] = params['individual_amount'][0]
					params['individual_amount'].pop(0)
		else:
			UAV['localization_time_left'] -= 1

	if UAV['status'] == 'charging':
		UAV['power'] -= UAV['charging_power_rate']
		nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
		nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
		if charge_mode == 'to_goal':
			if nodes[UAV['dest_node_id']]['power'] >= params['goal']:
				UAV['status'] = 'looking'
		elif charge_mode == 'with_fixed_amount':
			if UAV['with_fixed_amount_left'] <= 0 or nodes[UAV['dest_node_id']]['power'] == nodes[UAV['dest_node_id']]['capacity']:
				UAV['status'] = 'looking'
			else:
				UAV['with_fixed_amount_left'] -= UAV['charging_power_rate'] * UAV['transfer_rate']
		elif charge_mode == 'with_individual_amount':
			if UAV['with_individual_amount_left'] <= 0 or nodes[UAV['dest_node_id']]['power'] == nodes[UAV['dest_node_id']]['capacity']:
				UAV['status'] = 'looking'
			else:
				UAV['with_individual_amount_left'] -= UAV['charging_power_rate'] * UAV['transfer_rate']
		else:
			print 'Error charging algorithm'

		if UAV['status'] == 'charging':
			UAV['power'] -= UAV['hovering_power_rate']
		else:
			UAV['power'] -= UAV['flight_power_rate']


def compute_optimized_amount(UAV, nodes):
	total_power = UAV['power']

	least_power_path = least_power_node_path(UAV, nodes)
	least_power_dist = compute_total_cycle_distance(UAV, nodes, least_power_path)
	flight_power = (least_power_dist / UAV['speed']) * UAV['flight_power_rate']	
	total_power -=  flight_power

	localization_power = len(nodes) * UAV['localization_time'] * UAV['hovering_power_rate']
	total_power -= localization_power

	total_power *= UAV['charging_power_rate'] / (UAV['hovering_power_rate'] + UAV['charging_power_rate'])
	total_power *= UAV['transfer_rate']
	
	return max(0, total_power / len(nodes))



def next_second(UAV, nodes, charge_mode, path_mode, threshold, params = {}):
	node_num = len(nodes)

	if 'node_initial_average' not in params:
		params['initial_average_power'] = get_average_power_nodes(nodes)
						
	if 'node_capacity'	not in params:
		params['node_capacity'] = nodes[0]['capacity']
	
	# to full
	if charge_mode == 'to_full':
		if 'goal' not in params:
			params['goal'] = params['node_capacity']
		next_second_dest_list(UAV, nodes, 'to_goal', path_mode, threshold, params)
	# with precomputed amount	
	elif charge_mode == 'with_precomputed_amount':
		if 'fixed_amout' not in params:
			params['fixed_amount'] = compute_optimized_amount(UAV, nodes)
		next_second_dest_list(UAV, nodes, 'with_fixed_amount', path_mode, threshold, params)
	# with constant amount
	elif charge_mode == 'with_constant_amount':
		if 'fixed_amount' not in params:
			params['fixed_amount'] = 350
		next_second_dest_list(UAV, nodes, 'with_fixed_amount', path_mode, threshold, params)
	# to initial average
	elif charge_mode == 'to_initial_average':
		if 'goal' not in params:
			params['goal'] = params['initial_average_power']
		next_second_dest_list(UAV, nodes, 'to_goal', path_mode, threshold, params)
	# with individual amount
	elif charge_mode == 'with_individual_amount':
		if 'individual_amount' not in params or UAV['status'] == 'idle':
			params['individual_amount'] = [350 for i in range(node_num)]
		next_second_dest_list(UAV, nodes, 'with_individual_amount', path_mode, threshold, params)
	# error
	else:
		print 'Error charge mode: ' + str(charge_mode)
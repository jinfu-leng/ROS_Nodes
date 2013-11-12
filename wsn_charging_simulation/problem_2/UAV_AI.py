import math
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
	next_power = UAV['power'] - UAV['flght_power_rate']
	if UAV['status'] == 'charging':
		next_power -= UAV['charging_power_rate']
	consume_rate = UAV['flght_power_rate']	
	distance_home = euclidean_distance(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'])
	speed = UAV['speed']
	return int(next_power/consume_rate) > (int(distance_home/speed) + 3)

def is_UAV_at_home(UAV):
	if UAV['current_x'] == UAV['home_x'] and UAV['current_y'] == UAV['home_y']:
		return True
	else:
		return False

def shortest_distance_node_path(start_x, start_y, node_id_list, nodes):
	min_distance = None
	best_path = None
	last_x = start_x
	last_y = start_y
	for path in itertools.permutations(node_id_list):
		distance = 0.0
		for node_id in path:
			distance += euclidean_distance(last_x, last_y, nodes[node_id]['x'], nodes[node_id]['y'])
			last_x = nodes[node_id]['x']
			last_y = nodes[node_id]['y']
		if min_distance == None or distance < min_distance:
			min_distance = distance
			best_path = path
	return list(best_path)

def hamiltonian_node_path(UAV, nodes):
	nodes_cnt = len(nodes)
	shortest_path = []
	shortest_dist = 1000000000.0
	for path in itertools.permutations(range(nodes_cnt)):
		current_dist = 0.0
		for index in range(nodes_cnt-1):
			current_dist += euclidean_distance(nodes[path[index]]['x'], nodes[path[index]]['y'], nodes[path[index + 1]]['x'], nodes[path[index + 1]]['y'])
		current_dist += euclidean_distance(UAV['current_x'], UAV['current_y'], nodes[path[0]]['x'], nodes[path[0]]['y'])
		current_dist += euclidean_distance(UAV['current_x'], UAV['current_y'], nodes[path[nodes_cnt - 1]]['x'], nodes[path[nodes_cnt - 1]]['y'])
		if current_dist < shortest_dist:
			shortest_dist = current_dist
			shortest_path = path
	return list(shortest_path)

def get_average_power_nodes(nodes):
	return sum([node['power'] for node in nodes]) / len(nodes)

def closest_node(x, y, dest_list, nodes):
	if len(dest_list) == 0:
		print 'error'
	closest_node_id = dest_list[0]
	closest_node_dist = euclidean_distance(nodes[dest_list[0]]['x'], nodes[dest_list[0]]['y'], x, y)
	for i in range(1, len(dest_list)):
		new_dist = euclidean_distance(nodes[dest_list[i]]['x'], nodes[dest_list[i]]['y'], x, y)
		if new_dist < closest_node_dist:
			closest_node_dist = new_dist
			closest_node_id = dest_list[i]
	return closest_node_id

def next_second_closest_to_ratio(UAV, nodes, ratio):
	#print UAV['status']
	#print UAV['current_x'], UAV['current_y']

	UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'idle':
		UAV['dest_list'] = [node['id'] for node in nodes]
		UAV['status'] = 'looking'

	if UAV['status'] == 'looking':
		UAV['dest_node_id'] = closest_node(UAV['current_x'], UAV['current_y'], UAV['dest_list'], nodes)
		UAV['dest_list'].remove(UAV['dest_node_id'])
		UAV['status'] = 'moving'

	if UAV['status'] == 'moving':
		if nodes[UAV['dest_node_id']]['x'] == UAV['current_x'] and nodes[UAV['dest_node_id']]['y'] == UAV['current_y']:
			UAV['status'] = 'charging'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y

	if UAV['status'] == 'charging':
		if nodes[UAV['dest_node_id']]['power'] >= nodes[UAV['dest_node_id']]['capacity'] * ratio:
			nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['capacity'], nodes[UAV['dest_node_id']]['power'])
			if len(UAV['dest_list']) == 0:
				UAV['status'] = 'idle'
			else:
				UAV['status'] = 'looking'
        else:
        	UAV['power'] -= UAV['charging_power_rate']       
        	nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']

def next_second_closest_to_average(UAV, nodes):
	#print UAV['status']

	UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'idle':
		if 'node_average' in UAV:
			del UAV['node_average']

		UAV['dest_list'] = [node['id'] for node in nodes]
		UAV['status'] = 'looking'

	if UAV['status'] == 'looking':
		UAV['dest_node_id'] = closest_node(UAV['current_x'], UAV['current_y'], UAV['dest_list'], nodes)
		UAV['dest_list'].remove(UAV['dest_node_id'])
		UAV['status'] = 'moving'

	if UAV['status'] == 'moving':
		if nodes[UAV['dest_node_id']]['x'] == UAV['current_x'] and nodes[UAV['dest_node_id']]['y'] == UAV['current_y']:
			UAV['status'] = 'charging'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y

	if UAV['status'] == 'charging':
		if 'node_average' not in UAV:
			UAV['node_average'] = nodes[UAV['dest_node_id']]['power']
			UAV['node_average_cnt'] = 1
		else:
			UAV['node_average'] = (UAV['node_average'] * UAV['node_average_cnt'] + nodes[UAV['dest_node_id']]['power']) / (UAV['node_average_cnt'] + 1)
			UAV['node_average_cnt'] += 1

		if nodes[UAV['dest_node_id']]['power'] >= UAV['node_average']:
			nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['capacity'], nodes[UAV['dest_node_id']]['power'])
			if len(UAV['dest_list']) == 0:
				UAV['status'] = 'idle'
			else:
				UAV['status'] = 'looking'
        else:
        	UAV['power'] -= UAV['charging_power_rate']       
        	nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']

def next_second_dest_list(UAV, nodes, mode):
	UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'idle':
		UAV['dest_list'] = hamiltonian_node_path(UAV, nodes)
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
			UAV['status'] = 'charging'
			if mode == 'below_average_to_full' and nodes[UAV['dest_node_id']]['power'] >= get_average_power_nodes(nodes):
				UAV['status'] = 'looking'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y

	if UAV['status'] == 'charging':
		if mode == 'all_to_full':
			if nodes[UAV['dest_node_id']]['power'] < nodes[UAV['dest_node_id']]['capacity']:
				UAV['power'] -= UAV['charging_power_rate']
				nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
				nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
			else:
				UAV['status'] = 'looking'
		elif mode == 'below_average_to_average':
			if nodes[UAV['dest_node_id']]['power'] < get_average_power_nodes(nodes):
				UAV['power'] -= UAV['charging_power_rate']
				nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
				nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
			else:
				UAV['status'] = 'looking'
		elif mode == 'below_average_to_full':
			if nodes[UAV['dest_node_id']]['power'] < nodes[UAV['dest_node_id']]['capacity']:
				UAV['power'] -= UAV['charging_power_rate']
				nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
				nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
			else:
				UAV['status'] = 'looking'



def next_second(UAV, nodes, mode = 'cloeset_to_half'):
	if mode == 'cloeset_to_half':
		next_second_closest_to_ratio(UAV, nodes, 0.5)
	elif mode == 'cloeset_to_average':
		next_second_closest_to_average(UAV, nodes)
	elif mode == 'below_average_to_average':
		next_second_dest_list(UAV, nodes, mode)
	elif mode == 'below_average_to_full':
		next_second_dest_list(UAV, nodes, mode)
	elif mode == 'all_to_full':
		next_second_dest_list(UAV, nodes, mode)
	else:
		print 'error'
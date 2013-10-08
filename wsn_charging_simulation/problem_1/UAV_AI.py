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

# return the node of least power
def next_node_least_power(nodes, threshold): 
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	node_least_power = sorted_nodes[0]
	if node_least_power['power']/node_least_power['capacity'] <= threshold:
		return [node_least_power['id']]
	else:
		return None

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


# return the list of k nodes of least power; optimize the order of the list
def next_node_least_power_k(UAV, nodes, threshold, k):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	if sorted_nodes[0]['power']/sorted_nodes[0]['capacity'] <= threshold:
		node_least_power_list = []
		for i in range(k):
			node_least_power_list.append(sorted_nodes[i]['id'])
		return shortest_distance_node_path(UAV['current_x'], UAV['current_y'], node_least_power_list, nodes)
	else:
		return None

# partition the nodes into four districts based on the position of the nodes
def preprocess_partition(nodes):
	node_num = len(nodes)
	sorted_nodes = sorted(nodes, key=lambda node: node['x'])
	mid_x = sorted_nodes[node_num/2]['x']
	sorted_nodes = sorted(nodes, key=lambda node: node['y'])
	mid_y = sorted_nodes[node_num/2]['y']
	for node in nodes:
		if node['x'] <= mid_x and node['y'] <= mid_y:
			node['district'] = 0
		elif node['x'] <= mid_x and node['y'] > mid_y:
			node['district'] = 1
		elif node['x'] > mid_x and node['y'] <= mid_y:
			node['district'] = 2
		else:
			node['district'] = 3

def next_node_least_power_partition(nodes, threshold, k):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	if sorted_nodes[0]['power']/sorted_nodes[0]['capacity'] <= threshold:
		district = sorted_nodes[0]['district']
		district_nodes = [node['id'] for node in sorted_nodes if node['district'] == district]
		while len(district_nodes) > k:
			district_nodes.pop()
		return district_nodes
	else:
		return None

# charge the list of nodes
# when start to charge a node, charge it until it is full
# the threshold decides what time to go outside of base
def next_second_charge_until_full(algorithm_name, UAV, nodes, threshold = 1.0, k = 5):
	if UAV['status'] != 'back' and UAV['status'] != 'chargingself' and is_UAV_able_back_home_after_next_second(UAV) == False:
		UAV['status'] = 'back'
		UAV['task_num'] += 1
		
	if UAV['status'] == 'back':
		if UAV['current_x'] == UAV['home_x'] and UAV['current_y'] == UAV['home_y']:
			UAV['status'] = 'chargingself'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y
			UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'chargingself':
		if UAV['power'] >= UAV['capacity']:
			UAV['power'] = UAV['capacity']
			UAV['status'] = 'idle'
		else:
			UAV['power'] += UAV['charged_power_rate']
	
	if UAV['status'] == 'idle':
		if is_UAV_at_home(UAV) == True:
			if algorithm_name == 'least_power':
				UAV['dest_list'] = next_node_least_power(nodes, threshold)
			elif algorithm_name == 'least_power_k':
				UAV['dest_list'] = next_node_least_power_k(UAV, nodes, threshold, k)
			elif algorithm_name == 'least_power_partition':
				UAV['dest_list'] = next_node_least_power_partition(nodes, threshold, k)
			else:
				UAV['dest_list'] = None

		if UAV['dest_list'] == None:
			next_node_id = None
		elif len(UAV['dest_list']) == 0:
			next_node_id = next_node_least_power(nodes, 1.0)[0]
		else:
			next_node_id = UAV['dest_list'].pop(0)

		if next_node_id != None:
			UAV['dest_node_id'] = next_node_id
			UAV['status'] = 'flying'

	if UAV['status'] == 'flying':
		if nodes[UAV['dest_node_id']]['x'] == UAV['current_x'] and nodes[UAV['dest_node_id']]['y'] == UAV['current_y']:			
			UAV['status'] = 'charging'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y
			UAV['power'] -= UAV['flght_power_rate']
		
	if UAV['status'] == 'charging':
		UAV['power'] -= UAV['charging_power_rate']
		UAV['power'] -= UAV['flght_power_rate']
		nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
		nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
		if nodes[UAV['dest_node_id']]['power'] == nodes[UAV['dest_node_id']]['capacity']:
			UAV['status'] = 'idle'
			UAV['dest_node_id'] = None

def next_second(UAV, nodes, mode = 'least_power', task_threshold = 0.6):
	if mode == 'least_power':
		next_second_charge_until_full(mode, UAV, nodes, task_threshold) 
	elif mode == 'least_power_k':
		k = min(5, len(nodes))
		next_second_charge_until_full(mode, UAV, nodes, task_threshold, k) 
	elif mode == 'least_power_partition':
		preprocess_partition(nodes)
		next_second_charge_until_full(mode, UAV, nodes, task_threshold, len(nodes))
	else:
		print 'error'

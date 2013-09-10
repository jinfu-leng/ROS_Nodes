import math

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

# return the list of k nodes of least power; optimize the order of the list
def next_node_least_power_k(nodes, threshold, k):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	if sorted_nodes[0]['power']/sorted_nodes[0]['capacity'] <= threshold:
		node_least_power_list = []
		for i in range(k):
			node_least_power_list.append(sorted_nodes[i]['id'])
		return node_least_power_list
	else:
		return None

# charge the list of nodes with the least power
# optimize the path for the nodes in the list
# the threshold decides what time to go outside of base
def next_second_charge_until_full(algorithmName, UAV, nodes, threshold = 1.0, k = 5):
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
			if algorithmName == 'least_power':
				UAV['dest_list'] = next_node_least_power(nodes, threshold)
			elif algorithmName == 'least_power_k':
				UAV['dest_list'] = next_node_least_power_k(nodes, threshold, k)
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

def next_second(UAV, nodes, mode = 'least_power'):
	if mode == 'least_power':
		next_second_charge_until_full('least_power', UAV, nodes, 0.3) 
	elif mode == 'least_power_k':
		k = 5
		k = min(k, len(nodes))
		next_second_charge_until_full('least_power_k', UAV, nodes, 0.3, k) 
	else:
		print 'error'

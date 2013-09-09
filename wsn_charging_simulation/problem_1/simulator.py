import random
import math
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# read into the config file
import independent_outdoor_config as config

# system parameters
param_number_nodes = config.param_number_nodes
param_ground_width = config.param_ground_width
param_ground_height = config.param_ground_height
param_network_type = config.param_network_type
param_node_power_capacity = config.param_node_power_capacity
param_node_power_consumption_rate = config.param_node_power_consumption_rate
param_node_initial_power = config.param_node_initial_power
param_UAV_power_capacity = config.param_UAV_power_capacity
param_UAV_flight_power_consumption_rate = config.param_UAV_flight_power_consumption_rate
param_UAV_initial_power = config.param_UAV_initial_power
param_UAV_charging_power_consumption_rate = config.param_UAV_charging_power_consumption_rate
param_UAV_charging_power_transfer_rate = config.param_UAV_charging_power_transfer_rate
param_UAV_moving_speed = config.param_UAV_moving_speed
param_UAV_charged_power_accumulation_rate = config.param_UAV_charged_power_accumulation_rate
param_UAV_initial_x = config.param_UAV_initial_x
param_UAV_initial_y = config.param_UAV_initial_y

param_start_visualization = False
param_animation_frame_interval = 1 # wait how long between each frame
param_animation_frame_skip_num = 100 # skip how many frame between each animation

constant_second_of_7days = 604800
constant_second_of_30days = 2592000

def euclidean_distance(x, y, x2, y2):
	return math.sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2))

# nodes are homogeneous
def create_homogeneous_node_network(number_nodes, ground_width, ground_height):
	nodes = []
	for i in range(0, number_nodes):
		node = {}
		node['id'] = i
		node['x'] = random.random() * ground_width
		node['y'] = random.random() * ground_height
		node['capacity'] = param_node_power_capacity
		node['rate'] = param_node_power_consumption_rate
		node['power'] = param_node_initial_power
		nodes.append(node)
	return nodes

# nodes are homogeneous except initial power
def create_homogeneous2_node_network(number_nodes, ground_width, ground_height):
	nodes = []
	for i in range(0, number_nodes):
		node = {}
		node['id'] = i
		node['x'] = random.random() * ground_width
		node['y'] = random.random() * ground_height
		node['capacity'] = param_node_power_capacity
		node['rate'] = param_node_power_consumption_rate
		node['power'] = max(random.random(), 0.5) * param_node_initial_power
		nodes.append(node)
	return nodes

def create_node_network(number_nodes, ground_width, ground_height, network_type = 'homogeneous'):
	if network_type == 'homogeneous':
		return create_homogeneous_node_network(number_nodes, ground_width, ground_height)
	elif network_type == 'homogeneous2':
		return create_homogeneous2_node_network(number_nodes, ground_width, ground_height)
	else:
		return None


def nodes_next_second(nodes):
	for node in nodes:
		node['power'] -= node['rate']

def is_valid_node_network(nodes):
	for node in nodes:
		if node['power'] <= 0:
			return False
	return True

def create_UAV(ground_width, ground_height):
	UAV = {}
	UAV['home_x'] = param_UAV_initial_x
	UAV['home_y'] = param_UAV_initial_y
	UAV['current_x'] = param_UAV_initial_x
	UAV['current_y'] = param_UAV_initial_y
	UAV['capacity'] = param_UAV_power_capacity
	UAV['power'] = param_UAV_initial_power
	UAV['speed'] = param_UAV_moving_speed
	UAV['transfer_rate'] = param_UAV_charging_power_transfer_rate
	UAV['flght_power_rate'] = param_UAV_flight_power_consumption_rate
	UAV['charging_power_rate'] = param_UAV_charging_power_consumption_rate
	UAV['status'] = 'idle'
	UAV['dest_node_id'] = 0
	return UAV

def next_position(current_x, current_y, dest_x, dest_y, speed):
	x_diff = dest_x - current_x
	y_diff = dest_y - current_y
	dist_diff = math.sqrt(x_diff * x_diff + y_diff * y_diff)
	moving_ratio = 0
	if dist_diff > 0:
		moving_ratio = min(1, UAV['speed'] / dist_diff)
	return current_x + moving_ratio * x_diff, current_y + moving_ratio * y_diff

def is_UAV_able_back_home(UAV):
	current_power = UAV['power']
	consume_rate = UAV['flght_power_rate']
	if UAV['status'] == 'charging':
		consume_rate += UAV['charging_power_rate']
	distance_home = euclidean_distance(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'])
	speed = UAV['speed']
	return (current_power/consume_rate - 1) >  distance_home/speed

def UAV_next_second(UAV, nodes):
	global task_num
	if is_UAV_able_back_home(UAV) == False:
		UAV['status'] = 'back'
		
	if UAV['status'] == 'back':
		if UAV['current_x'] == UAV['home_x'] and UAV['current_y'] == UAV['home_y']:
			task_num += 1
			UAV['status'] = 'chargingself'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], UAV['home_x'], UAV['home_y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y
			UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'chargingself':
		UAV['power'] += param_UAV_charged_power_accumulation_rate
		if UAV['power'] >= UAV['capacity']:
			UAV['power'] = UAV['capacity']
			UAV['status'] = 'idle'
	
	if UAV['status'] == 'idle':
		sorted_nodes = sorted(nodes, key=lambda node: node['power'])
		UAV['dest_node_id'] = sorted_nodes[0]['id']
		UAV['status'] = 'flying'

	if UAV['status'] == 'flying':
		if euclidean_distance(nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['current_x'], UAV['current_y']) < 0.1:
			UAV['power'] -= UAV['flght_power_rate']
			UAV['status'] = 'charging'
		else:
			next_x, next_y = next_position(UAV['current_x'], UAV['current_y'], nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['speed'])
			UAV['current_x'] = next_x
			UAV['current_y'] = next_y
			UAV['power'] -= UAV['flght_power_rate']
		
	if UAV['status'] == 'charging':
		UAV['power'] -= UAV['charging_power_rate']
		nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
		nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
		if nodes[UAV['dest_node_id']]['power'] == nodes[UAV['dest_node_id']]['capacity']:
			UAV['status'] = 'idle'
			UAV['dest_node_id'] = None

def visualize_animate(i):
	global visualization_UAV, visualization_nodes_text
	global UAV, nodes, round_num

	if is_valid_node_network(nodes):
		left = param_animation_frame_skip_num + 1
		while left > 0 and is_valid_node_network(nodes):
			nodes_next_second(nodes)
			UAV_next_second(UAV, nodes)
			print 'Round: ' + str(round_num)
			round_num += 1
			left -= 1

	visualization_UAV.set_data(UAV['current_x'], UAV['current_y'])
	for node_index in range(len(visualization_nodes_text)):
		visualization_nodes_text[node_index].set_text('%.2lf' % nodes[node_index]['power'])

def start_visualization():
	global UAV, nodes
	UAV = create_UAV(param_ground_width, param_ground_height)
	nodes = create_node_network(param_number_nodes, param_ground_width, param_ground_height, param_network_type)

	global visualization_nodes_text
	visualization_nodes_text = []
	for node in nodes:
		visualization_node_text = ax.text(node['x'], node['y'], '')
		visualization_nodes_text.append(visualization_node_text)

	ani = animation.FuncAnimation(fig, visualize_animate, interval=param_animation_frame_interval, blit=False)
	plt.show()

def start_simulation():
	global UAV, nodes, round_num, task_num
	UAV = create_UAV(param_ground_width, param_ground_height)
	nodes = create_node_network(param_number_nodes, param_ground_width, param_ground_height, param_network_type)
	while is_valid_node_network(nodes):
		print 'Round: ' + str(round_num)
		if round_num == constant_second_of_7days:
			print 'The system was valid during the past 7 days'
			print 'The charging task was conducted ' + str(task_num) + ' times'
			break
		round_num += 1
		nodes_next_second(nodes)
		UAV_next_second(UAV, nodes)
	

# visualization
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-3, param_ground_width+3), ylim=(-3, param_ground_height+3))
ax.grid()
visualization_UAV, = ax.plot([], [], 'bo', ms=10)
visualization_ground = plt.Rectangle((0, 0), param_ground_width, param_ground_height, lw=2, fc='none')
visualization_nodes_text = None
ax.add_patch(visualization_ground)

UAV = None
nodes = None
round_num = 1
task_num = 1

if param_start_visualization == True:
	start_visualization()
else:
	start_simulation()
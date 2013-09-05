import random
import math
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# system parameters
param_number_nodes = 6
param_ground_width = 100.0
param_ground_height = 100.0

param_node_power_capacity = 2.34 * 3600 * 1.0
param_node_power_consumption_rate = 0.1855
param_node_initial_power = param_node_power_capacity * 0.75

param_UAV_initial_x = 0.0
param_UAV_initial_y = 0.0
param_UAV_power_capacity = 25 * 3600 * 1.0 
param_UAV_flight_power_consumption_rate = 75.0
param_UAV_initial_power = 25 * 3600 * 1.0
param_UAV_charging_power_consumption_rate = 15.0
param_UAV_charging_power_transfer_rate = 1.0 
param_UAV_moving_speed = 5.0

param_animation_frame_interval = 10
param_show_visualization = True

# This variable is used to record the sequence of the states
UAV_nodes_state_log = []

def euclidean_distance(x, y, x2, y2):
	return math.sqrt((x - x2)*(x - x2) + (y - y2)*(y - y2))

def create_node_network(number_nodes, ground_width, ground_height):
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

def is_UAV_able_back_home(UAV):
	dist_to_home = euclidean_distance(UAV['home_x'], UAV['home_y'], UAV['current_x'], UAV['current_y'])
	time_required = math.ceil(dist_to_home / UAV['flght_power_rate'])
	power_required = time_required * UAV['flght_power_rate']
	return UAV['power'] > power_required

def which_node_is_next(UAV, nodes):
	sorted_nodes = sorted(nodes, key=lambda node: node['power'])
	return sorted_nodes[0]['id']

def UAV_next_second(UAV, nodes):
	if UAV['power'] <=0:
		UAV['status'] = 'idle'
		return

	UAV['power'] -= UAV['flght_power_rate']

	if UAV['status'] == 'idle':
		sorted_nodes = sorted(nodes, key=lambda node: node['power'])
		UAV['dest_node_id'] = sorted_nodes[0]['id']
		UAV['status'] = 'flying'

	if UAV['status'] == 'flying':
		if euclidean_distance(nodes[UAV['dest_node_id']]['x'], nodes[UAV['dest_node_id']]['y'], UAV['current_x'], UAV['current_y']) < 0.1:
			UAV['status'] = 'charging'
		else:
			x_diff = nodes[UAV['dest_node_id']]['x'] - UAV['current_x']
			y_diff = nodes[UAV['dest_node_id']]['y'] - UAV['current_y']
			dist_diff = math.sqrt(x_diff * x_diff + y_diff * y_diff)
			moving_ratio = 0
			if dist_diff > 0:
				moving_ratio = min(1, UAV['speed'] / dist_diff)
				UAV['current_x'] += moving_ratio * x_diff
				UAV['current_y'] += moving_ratio * y_diff
		
	if UAV['status'] == 'charging':
		UAV['power'] -= UAV['charging_power_rate']
		nodes[UAV['dest_node_id']]['power'] += UAV['charging_power_rate'] * UAV['transfer_rate']
		nodes[UAV['dest_node_id']]['power'] = min(nodes[UAV['dest_node_id']]['power'], nodes[UAV['dest_node_id']]['capacity'])
		if nodes[UAV['dest_node_id']]['power'] == nodes[UAV['dest_node_id']]['capacity']:
			UAV['status'] = 'idle'
			UAV['dest_node_id'] = None	

	if UAV['status'] == 'back':
		print 'Error!!!!!!!!!!!!!!!!'


# visualization
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-3, param_ground_width+3), ylim=(-3, param_ground_height+3))
ax.grid()
visualization_UAV, = ax.plot([], [], 'bo', ms=10)
visualization_ground = plt.Rectangle((0, 0), param_ground_width, param_ground_height, lw=2, fc='none')
visualization_nodes_text = None
ax.add_patch(visualization_ground)

def visualize_init():
	global visualization_UAV, visualization_ground, visualization_nodes_text
	return visualization_UAV, visualization_ground 
def visualize_animate(i):
	global visualization_UAV, visualization_ground, visualization_nodes_text
	print 'round_num: ' + str((i+1))
	visualization_UAV.set_data([UAV_nodes_state_log[i]['UAV']['current_x']], [UAV_nodes_state_log[i]['UAV']['current_y']])
	for node_index in range(len(visualization_nodes_text)):
		visualization_nodes_text[node_index].set_text('%.2lf' % UAV_nodes_state_log[i]['nodes'][node_index]['power'])
	return visualization_UAV, visualization_ground
def visualize():
	global visualization_nodes_text
	visualization_nodes_text = []
	for node in UAV_nodes_state_log[0]['nodes']:
		visualization_node_text = ax.text(node['x'], node['y'], '')
		visualization_nodes_text.append(visualization_node_text)
	ani = animation.FuncAnimation(fig, visualize_animate, frames=len(UAV_nodes_state_log), interval=param_animation_frame_interval, blit=False, init_func=visualize_init)
	plt.show()


nodes = create_node_network(param_number_nodes, param_ground_width, param_ground_height)
round_num = 1
UAV = create_UAV(param_ground_width, param_ground_height)

# record initial states
state = {}
state['UAV'] = copy.deepcopy(UAV)
state['nodes'] = copy.deepcopy(nodes)
UAV_nodes_state_log.append(state)

while is_valid_node_network(nodes):
	print 'round_num: ' + str(round_num)
	round_num += 1
	nodes_next_second(nodes)
	UAV_next_second(UAV, nodes)
	# record next states
	state = {}
	state['UAV'] = copy.deepcopy(UAV)
	state['nodes'] = copy.deepcopy(nodes)
	UAV_nodes_state_log.append(state)

if param_show_visualization == True:
	visualize()
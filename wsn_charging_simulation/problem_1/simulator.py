import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# read into the config file
import outdoor_faraway_config as config
import UAV_AI

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
param_animation_frame_skip_num = 1000 # skip how many frame between each animation
param_print_round_number = False

param_UAV_mode = 'least_power_partition'

constant_second_of_7days = 604800
constant_second_of_30days = 2592000

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
		print 'error'
		return None

def save_node_network(nodes, outputPath):
	try:
		outputFile = open(outputPath, 'w')
		for node in nodes:
			outputLine = ''
			for key in node.keys():
				outputLine += str(key) + ',' + str(node[key]) + ','
			outputLine = outputLine[0:-1] + '\n'
			outputFile.write(outputLine)
		outputFile.close()
		return True
	except Exception, e:
		print e
		return False

def read_node_network(inputPath):
	try:
		inputFile = open(inputPath, 'r')
		nodes = []
		for line in inputFile.readlines():
			lineSplit = line.split(',')
			fieldNum = len(lineSplit)/2
			node = {}
			for i in range(fieldNum):
				node[lineSplit[2*i]] = float(lineSplit[2*i+1])
			node['id'] = int(node['id'])
			nodes.append(node)
		inputFile.close()
		return nodes
	except:
		print 'error'
		return None

def nodes_next_second(nodes):
	for node in nodes:
		node['power'] -= node['rate']

def is_valid_node_network(nodes):
	for node in nodes:
		if node['power'] <= 0:
			return False
	return True

def is_valid_UAV(UAV):
	if UAV['power'] <= 0:
		return False
	else:
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
	UAV['charging_power_rate'] = param_UAV_charging_power_consumption_rate # charging nodes
	UAV['charged_power_rate'] = param_UAV_charged_power_accumulation_rate # being charged
	UAV['status'] = 'idle'
	UAV['dest_node_id'] = None
	UAV['task_num'] = 0
	return UAV

def visualize_animate(i):
	global visualization_UAV, visualization_nodes_text
	global UAV, nodes, round_num

	left = param_animation_frame_skip_num + 1
	while left > 0 and is_valid_node_network(nodes) and is_valid_UAV(UAV):
		nodes_next_second(nodes)
		UAV_AI.next_second(UAV, nodes, param_UAV_mode)
		if param_print_round_number:
			print 'Round: ' + str(round_num)
		round_num += 1
		left -= 1

	visualization_UAV.set_data(UAV['current_x'], UAV['current_y'])
	for node_index in range(len(visualization_nodes_text)):
		visualization_nodes_text[node_index].set_text('%.2lf' % nodes[node_index]['power'])

def start_visualization():
	global nodes, visualization_nodes_text
	visualization_nodes_text = []
	for node in nodes:
		visualization_node_text = ax.text(node['x'], node['y'], '')
		visualization_nodes_text.append(visualization_node_text)

	ani = animation.FuncAnimation(fig, visualize_animate, interval=param_animation_frame_interval, blit=False)
	plt.show()

def start_simulation():
	global UAV, nodes, round_num
	while is_valid_node_network(nodes) and is_valid_UAV(UAV):
		if param_print_round_number:
			print 'Round: ' + str(round_num)
		if round_num == constant_second_of_7days:
			print 'The system was valid during the past 7 days'
			print 'The charging task was conducted ' + str(UAV['task_num']) + ' times'
			break
		round_num += 1
		nodes_next_second(nodes)
		UAV_AI.next_second(UAV, nodes, param_UAV_mode)
	else:
		print 'The system is not valid at round ' + str(round_num)
	

# visualization
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-3, param_ground_width+3), ylim=(-3, param_ground_height+3))
ax.grid()
visualization_UAV, = ax.plot([], [], 'bo', ms=10)
visualization_ground = plt.Rectangle((0, 0), param_ground_width, param_ground_height, lw=2, fc='none')
visualization_nodes_text = None
ax.add_patch(visualization_ground)

round_num = 1


UAV = create_UAV(param_ground_width, param_ground_height)
#random.seed()
#nodes = create_node_network(param_number_nodes, param_ground_width, param_ground_height, param_network_type)
#save_node_network(nodes, 'outdoor_faraway_20_1000_1000.csv')
nodes = read_node_network('outdoor_center_20_1000_1000.csv')
if param_start_visualization == True:
	start_visualization()
else:
	start_simulation()

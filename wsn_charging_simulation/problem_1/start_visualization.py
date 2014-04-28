import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import time
import numpy as np

import vis_config as config
import UAV_AI
from object_manager import ObjectManager

# system parameters
param_animation_frame_interval = 100 # wait how long between each frame
param_animation_frame_skip_num = 0 # skip how many frame between each animation
path_mode = 'closest'
charge_mode = 'with_constant'
params = {}

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

def visualize_animate(i):
	global visualization_UAV, visualization_nodes_text, visualization_path, path_x, path_y
	global UAV_energy_efficient_bar
	global UAV, nodes, round_num

	# visualize the field
	visualization_UAV.set_data(UAV['current_x'], UAV['current_y'])
	if path_x[-1] != UAV['current_x'] or path_y[-1] !=  UAV['current_y']:
		path_x.append(UAV['current_x'])
		path_y.append(UAV['current_y'])
	visualization_path.set_data(path_x, path_y)
	for node_index in range(len(visualization_nodes_text)):
		visualization_nodes_text[node_index].set_text('%d:' % node_index + '(%.2lf)' % nodes[node_index]['power'])

	# visualize the UAV
	for rect in UAV_energy_bar:
		rect.set_height(UAV['power'])

	# visualize the sensor nodes
	for bars, node in zip(nodes_energy_bar_list, nodes):
		for rect in bars:
			rect.set_height(node['power'])

	left = param_animation_frame_skip_num + 1
	while left > 0 and is_valid_node_network(nodes) and is_valid_UAV(UAV):
		nodes_next_second(nodes)
		UAV_AI.next_second(UAV, nodes, charge_mode, path_mode, 1.0, params)
		round_num += 1
		left -= 1

def start_visualization():
	global nodes, visualization_nodes_text
	visualization_nodes_text = []
	for node in nodes:
		visualization_node_text = ax.text(node['x'], node['y'], '')
		visualization_nodes_text.append(visualization_node_text)
	ani = animation.FuncAnimation(fig, visualize_animate, interval=param_animation_frame_interval, blit=False)
	plt.show()


# main
object_manager_ = ObjectManager()
round_num = 1
UAV, nodes = object_manager_.create_objects(config)
path_x = [UAV['current_x']]
path_y = [UAV['current_y']]		
# visualization
# set up figure and animation
fig = plt.figure(figsize=(12, 6))
gs = gridspec.GridSpec(1, 3, width_ratios=[9, 4, 1], wspace=0.5)
ax = fig.add_subplot(gs[0], xlim=(-3, config.param_ground_width+3), ylim=(-3, config.param_ground_height+3))
ax_nodes = fig.add_subplot(gs[1])
ax_UAV = fig.add_subplot(gs[2])

# field
ax.grid()
visualization_path, = ax.plot([], [], '-', lw=2)
visualization_UAV, = ax.plot([], [], 'bo', ms=10)
visualization_ground = plt.Rectangle((0, 0), config.param_ground_width, config.param_ground_height, lw=2, fc='none')
visualization_nodes_text = None

bar_width = 0.8
# UAV
UAV_energy_bar = ax_UAV.bar(0.2, UAV['capacity'], bar_width, color = 'b', label = 'Total Energy')
ax_UAV.set_xticks([])
ax_UAV.set_xlabel('UAV')
ax_UAV.set_ylabel('Energy (J)')

# sensor nodes
nodes_energy_bar_list = []
index = 0
for node in nodes:
	node_energy_bar = ax_nodes.bar(index, node['capacity'], bar_width, color = 'b')
	nodes_energy_bar_list.append(node_energy_bar)
	index += 1
nodes_cnt = len(nodes)
nodes_labels = range(2, 5)
ax_nodes.set_xticks(np.arange(nodes_cnt) + 0.5)
ax_nodes.set_xticklabels(range(nodes_cnt))
ax_nodes.set_xlabel('Sensor nodes')
ax_nodes.set_ylabel('Energy (J)')

start_visualization()
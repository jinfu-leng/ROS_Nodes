
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# read into the config file
import outdoor_center_config as config
import UAV_AI
from object_manager import ObjectManager

# system parameters

param_animation_frame_interval = 1 # wait how long between each frame
param_animation_frame_skip_num = 10 # skip how many frame between each animation
param_print_round_number = False
param_UAV_mode = 'least_power_partition'

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


# main
object_manager_ = ObjectManager()
round_num = 1
UAV, nodes = object_manager_.create_objects(config)
			
# visualization
# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-3, config.param_ground_width+3), ylim=(-3, config.param_ground_height+3))
ax.grid()
visualization_UAV, = ax.plot([], [], 'bo', ms=10)
visualization_ground = plt.Rectangle((0, 0), config.param_ground_width, config.param_ground_height, lw=2, fc='none')
visualization_nodes_text = None
ax.add_patch(visualization_ground)
start_visualization()
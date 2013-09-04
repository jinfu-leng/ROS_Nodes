from create import create_network
from consume import update_network_energy
from sink import next_sink
import copy
from plot import plot_network_energy



num_col = 11 
num_row = 11
grid_width = 1.0
grid_height = 1.0
initial_energy = 1.0
full_energy = 1.0
iteration_cnt = 36
interesting_iteration = [0, 5, 10, 15, 20, 25, 30, 35]

nodes_list = []
sink_list = []
# create network
nodes = create_network(num_col, num_row, grid_width, grid_height, initial_energy, full_energy)

# iterate
for iteration in range(iteration_cnt):
	# record network status
	nodes_list.append(copy.deepcopy(nodes))

	# compute sink index
	# method = 'static', 'random', 'circle', 'block'
	sink_index = next_sink(nodes_list, sink_list, num_col, num_row, method = 'static')
	sink_list.append(sink_index)	
	
	# update network energy
	update_network_energy(nodes, sink_index)

# plot network's energy
plot_network_energy([nodes_list[i] for i in interesting_iteration], num_col, num_row, grid_width, grid_height)


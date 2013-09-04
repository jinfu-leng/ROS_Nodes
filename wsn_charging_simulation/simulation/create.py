import random

def create_network(num_col, num_row, grid_width, grid_height, initial_energy, full_energy):
	nodes = {} 
	for row in range(num_row):
		for col in range(num_col):
			node = {}
			node_id = col + row * num_col
			node['x'] = col * grid_width + random.random() * grid_width
			node['y'] = row * grid_height + random.random() * grid_height
			#node['x'] = col * grid_width + 0.5 * grid_width 
			#node['y'] = row * grid_height + 0.5 * grid_width
			node['current_energy'] = initial_energy
			node['full_energy'] = full_energy
			node['connection'] = []
			for i in [-1, 0, 1]:
				for j in [-1, 0, 1]:
					#if (i * j != 0) or (i == 0 and j ==0):
					if i == 0 and j ==0:
						continue
					adjacent_col = col + i
					adjacent_row = row + j
					if adjacent_col >=0 and adjacent_col < num_col and adjacent_row >=0 and adjacent_row < num_row:
						node['connection'].append(adjacent_col + adjacent_row * num_col)
			nodes[node_id] = node
	return nodes

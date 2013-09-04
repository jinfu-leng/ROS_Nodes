import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

def plot_network_energy(nodes_list, num_col, num_row, grid_width, grid_height):

	for nodes in nodes_list:

		fig = plt.figure()
		ax = fig.add_subplot(111)
		X = np.ndarray(shape=(num_row, num_col))
	
		for row in range(num_row):
			for col in range(num_col):
				node_id = col + row * num_col
				current_energy = nodes[node_id]['current_energy']
				X[row, col] = current_energy
		ax.imshow(X, cmap=cm.jet, interpolation='nearest', vmin = 0.0, vmax = 1.0)
		

	plt.show()

import math
from Queue import PriorityQueue

def distance_nodes(nodes, index, index2):
	dist2 = (nodes[index]['x'] - nodes[index2]['x']) * (nodes[index]['x'] - nodes[index2]['x']) + (nodes[index]['y'] - nodes[index2]['y']) * (nodes[index]['y'] - nodes[index2]['y'])   
	dist = math.sqrt(dist2)
	return dist

def energy_consumption_nodes(nodes, send_index, receive_index):
	return 0.001, 0.001

def energy_consumption_nodes2(nodes, send_index, receive_index):
	dist = distance_nodes(nodes, send_index, receive_index) 
	return 0.001 + 0.001 * math.pow(dist, 2), 0.001
	#return 0, 0.001

def shortest_path(nodes, from_index, to_index):
	candidates = PriorityQueue()
	candidates.put((0, from_index))
	previous_node = {}
	previous_node[from_index] = from_index
	dist_nodes = {}
	dist_nodes[from_index] = 0
	while not candidates.empty():
		priority, current_node = candidates.get()
		if current_node == to_index:
			break

		for next_node in nodes[current_node]['connection']:
			if (next_node not in dist_nodes) or dist_nodes[next_node] > (dist_nodes[current_node] + distance_nodes(nodes, current_node, next_node)):
				dist_nodes[next_node] = dist_nodes[current_node] + distance_nodes(nodes, current_node, next_node)
				candidates.put((dist_nodes[next_node], next_node))
				previous_node[next_node] = current_node


	if to_index not in previous_node:
		return []
	
	path = []
	last = to_index			
	while previous_node[last] != last:
		path.append(last)
		last = previous_node[last]
	path.append(last)
	path.reverse()
	return path


def update_network_energy(nodes, sink_index):
	for node_index in nodes.keys():
		if node_index == sink_index:
			continue
		node_path = shortest_path(nodes, node_index, sink_index)
		for i in range(len(node_path)-1):
			send_consumption, receive_consumption = energy_consumption_nodes2(nodes, node_path[i], node_path[i+1])
			nodes[node_path[i]]['current_energy'] -= send_consumption
			nodes[node_path[i+1]]['current_energy'] -= receive_consumption



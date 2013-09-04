import random

def next_sink(nodes_list, sink_list, num_col, num_row, method):
	if method == 'static':
		return next_sink_static(nodes_list, sink_list, num_col, num_row)
	if method == 'random':
		return next_sink_random(nodes_list, sink_list, num_col, num_row)
	if method == 'circle':
		return next_sink_circle(nodes_list, sink_list, num_col, num_row)
	if method == 'block':
		return next_sink_block(nodes_list, sink_list, num_col, num_row)

def next_sink_static(nodes_list, sink_list, num_col, num_row):
	return num_col * num_row // 2

def next_sink_random(nodes_list, sink_list, num_col, num_row):
	return random.randint(0, num_col * num_row -1)

def next_sink_circle(nodes_list, sink_list, num_col, num_row):
	index = len(sink_list)
	index %= 2 * num_col + 2 * num_row - 4
	if index == 0:
		return 0

	last_sink = sink_list[index - 1]
	col = last_sink % num_col
	row = last_sink // num_col
	if row == 0:
		if col == num_col - 1:
			row += 1
		else:
			col += 1
	elif col == num_col - 1:
		if row == num_row - 1:
			col -= 1
		else:
			row += 1
	elif row == num_row - 1:
		if col == 0:
			row -= 1
		else:
			col -= 1
	else:
		row -= 1

	return col + row * num_col

def next_sink_block(nodes_list, sink_list, num_col, num_row):
	nodes = nodes_list[len(nodes_list) - 1]
	max_energy = 0
	max_energy_index = -1
	for row in range(num_row):
		for col in range(num_col):
			energy = 0
			for i in [-1, 0 , 1]:
				for j in [-1, 0, 1]:
					if col + i < num_col and col + i >= 0 and row + j < num_row and row + j >= 0:
						energy += nodes[col + i + (row + j) * num_col]['current_energy']
			if energy > max_energy:
				max_energy = energy
				max_energy_index = col + row * num_col
	return max_energy_index

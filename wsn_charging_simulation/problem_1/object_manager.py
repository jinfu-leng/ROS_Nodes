import random

class ObjectManager:
	# create UAV
	def create_UAV(self, config):
		UAV = {}
		UAV['home_x'] = config.param_UAV_initial_x
		UAV['home_y'] = config.param_UAV_initial_y
		UAV['current_x'] = config.param_UAV_initial_x
		UAV['current_y'] = config.param_UAV_initial_y
		UAV['capacity'] = config.param_UAV_power_capacity
		UAV['power'] = config.param_UAV_initial_power
		UAV['speed'] = config.param_UAV_moving_speed
		UAV['transfer_rate'] = config.param_UAV_charging_power_transfer_rate
		UAV['flght_power_rate'] = config.param_UAV_flight_power_consumption_rate
		UAV['charging_power_rate'] = config.param_UAV_charging_power_consumption_rate # charging nodes
		UAV['charged_power_rate'] = config.param_UAV_charged_power_accumulation_rate # being charged
		return UAV

	# create nodes
	def create_nodes(self, config):
		random.seed()
		nodes = []
		for i in range(0, config.param_number_nodes):
			node = {}
			node['id'] = i
			node['x'] = random.random() * config.param_ground_width
			node['y'] = random.random() * config.param_ground_height
			node['capacity'] = config.param_node_power_capacity
			node['rate'] = config.param_node_power_consumption_rate
			if config.param_network_type == 'homogeneous':
				node['power'] = config.param_node_initial_power
			elif config.param_network_type == 'homogeneous2':
				node['power'] = max(random.random(), 0.5) * config.param_node_initial_power
			else:
				return None
			nodes.append(node)
		return nodes

	# save UAV and nodes to file
	def save_objects(self, UAV, nodes, output_path):
		try:
			output_file = open(output_path, 'w')
			# write UAV
			output_line = ''
			for key in UAV.keys():
				output_line += str(key) + ',' + str(UAV[key]) + ','
			output_line = output_line[0:-1] + '\n'
			output_file.write(output_line)
			# write nodes
			for node in nodes:
				output_line = ''
				for key in node.keys():
					output_line += str(key) + ',' + str(node[key]) + ','
				output_line = output_line[0:-1] + '\n'
				output_file.write(output_line)

			output_file.close()
			return True
		except Exception, e:
			print e
			return False

	# create objects
	# please call this method when initializing objects
	def create_objects(self, config, save_path = None):
		UAV = self.create_UAV(config)
		nodes = self.create_nodes(config)
		if save_path != None:
			self.save_objects(UAV, nodes, save_path)
		UAV['status'] = 'idle'
		UAV['dest_node_id'] = None
		UAV['task_num'] = 0
		return UAV, nodes
		
	# read UAV and nodes from file
	def read_objects(self, input_path):
		try:
			input_file = open(input_path, 'r')
			# read UAV
			UAV = {}
			line = input_file.readline()
			line_split = line.split(',')
			field_num = len(line_split)/2
			for i in range(field_num):
				UAV[line_split[2*i]] = float(line_split[2*i+1])
			UAV['status'] = 'idle'
			UAV['dest_node_id'] = None
			UAV['task_num'] = 0
			# read nodes
			nodes = []
			for line in input_file.readlines():
				line_split = line.split(',')
				field_num = len(line_split)/2
				node = {}
				for i in range(field_num):
					node[line_split[2*i]] = float(line_split[2*i+1])
				node['id'] = int(node['id'])
				nodes.append(node)
			input_file.close()
			return UAV, nodes
		except Exception, e:
			print e
			return None

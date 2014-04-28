import random

class ObjectManager:
	# create UAV
	def create_UAV(self, config):
		UAV = {}
		UAV['home_x'] = config.param_ground_width / 2 + config.param_UAV_base_distance
		UAV['home_y'] = config.param_ground_height / 2
		UAV['current_x'] = UAV['home_x']
		UAV['current_y'] = UAV['home_y']
		UAV['capacity'] = config.param_UAV_power_capacity
		UAV['power'] = config.param_UAV_power_capacity
		UAV['speed'] = config.param_UAV_moving_speed
		UAV['transfer_rate'] = config.param_UAV_charging_power_transfer_rate
		UAV['flight_power_rate'] = config.param_UAV_flight_power_consumption_rate
		UAV['hovering_power_rate'] = config.param_UAV_hovering_power_consumption_rate
		UAV['charging_power_rate'] = config.param_UAV_charging_power_consumption_rate # charging nodes
		UAV['localization_time'] = config.param_UAV_localization_time
		UAV['accumulating_power_rate'] = config.param_UAV_accumulating_power_rate
		#print UAV['current_x'], UAV['current_y']
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
				node['power'] = random.uniform(0.2, 0.6) * config.param_node_power_capacity
			else:
				print 'Error: create_nodes()'
				return None
			nodes.append(node)
		#print [node['x'] for node in nodes]
		#print [node['y'] for node in nodes]
		return nodes

	# create objects
	# please call this method when initializing objects
	def create_objects(self, config):
		UAV = self.create_UAV(config)
		UAV['status'] = 'idle'
		UAV['dest_node_id'] = None
		nodes = self.create_nodes(config)		
		return UAV, nodes

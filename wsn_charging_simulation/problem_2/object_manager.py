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
				node['power'] = min(0.5, max(random.random(), 0.2)) * config.param_node_initial_power
			else:
				return None
			nodes.append(node)
		return nodes

	# create objects
	# please call this method when initializing objects
	def create_objects(self, config):
		UAV = self.create_UAV(config)
		UAV['status'] = 'idle'
		UAV['dest_node_id'] = None
		nodes = self.create_nodes(config)		
		return UAV, nodes
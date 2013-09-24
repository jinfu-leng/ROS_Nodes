from object_manager import ObjectManager
import outdoor_faraway_config as config


object_manager_ = ObjectManager()

param_number_nodes = [5, 10, 20, 50]
param_ground_size = [10, 100, 1000]
#param_network_type = ['homogeneous', 'homogeneous2']
param_network_type = ['homogeneous']

for num in param_number_nodes:
	for size in param_ground_size:
		for net_type in param_network_type:
			config.param_number_nodes = num
			config.param_ground_width = size
			config.param_ground_height = size
			config.param_network_type = net_type
			output_file_name = str(num) + '_' + str(size) + '_' + str(size) + '_' + 'random' + '_' + str(net_type) + '_' + 'faraway' + '.csv'
			object_manager_.create_objects(config, output_file_name)


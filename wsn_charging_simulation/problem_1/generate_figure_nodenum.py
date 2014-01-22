import numpy as np
import matplotlib.pyplot as plt

def ComputeStd(nums):
	return np.std(nums)

input_file_name = 'repeat_center_nodenum.csv'
input_file = open(input_file_name, 'r')
first_line = input_file.readline()

res = {}
UAV_modes = {}
network_types = {}
for line in input_file.readlines():
	line_split = line[:-1].split(',')
	network_type = line_split[0]
	UAV_mode = line_split[1]
	task_num = int(line_split[2])

	if network_type not in res:
		res[network_type] = {}
	if UAV_mode not in res[network_type]:
		res[network_type][UAV_mode] = []

	if UAV_mode not in UAV_modes:
		UAV_modes[UAV_mode] = True
	if network_type not in network_types:
		network_types[network_type] = True
		
	res[network_type][UAV_mode].append(task_num)
input_file.close()

# generate success rate figure
UAV_mode_keys = UAV_modes.keys()
network_type_keys = sorted(network_types.keys(), key=lambda node:int(node.split('_')[0]))
network_type_size = [int(value.split('_')[0]) for value in network_type_keys]
print network_type_size
UAV_mode_num = len(UAV_mode_keys)
network_type_num = len(network_type_keys)

left_coordinates = np.arange(network_type_num)
bar_width = 0.25
opacity = 0.4
colors = 'bgrcmyk'

for i in range(UAV_mode_num):
	UAV_mode_success_rate = []
	for j in range(network_type_num):
		UAV_mode = UAV_mode_keys[i]
		network_type = network_type_keys[j]	
		total_cnt = len(res[network_type][UAV_mode])
		success_cnt = len([value for value in res[network_type][UAV_mode] if value >=0])
		success_rate = 1.0 * success_cnt / total_cnt
		UAV_mode_success_rate.append(success_rate)
	plt.bar(left_coordinates + i * bar_width, UAV_mode_success_rate, bar_width,
		label=UAV_mode, color=colors[i], alpha=opacity)
	print UAV_mode, UAV_mode_success_rate

plt.xlabel('Number of sensor nodes')
plt.ylabel('Success Rate')
plt.title('Success rate by number of nodes and algorithmss')
plt.xticks(left_coordinates + bar_width * 1.5, network_type_size)
plt.legend()

#plt.tight_layout()
plt.show()


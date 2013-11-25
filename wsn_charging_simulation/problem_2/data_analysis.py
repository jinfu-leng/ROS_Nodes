import numpy as np

def ComputeStd(nums):
	return np.std(nums)

input_file_name = 'res_prob2_outdoor_center__11_25_4.csv'
output_file_name = input_file_name[:-4] + '_aggregated.csv'

input_file = open(input_file_name, 'r')
output_file = open(output_file_name, 'w')

first_line = input_file.readline()
output_file.write('network_type,UAV_mode,cnt,average,std' + '\n')

res = {}
for line in input_file.readlines():
	line_split = line[:-1].split(',')
	network_type = line_split[0]
	UAV_mode = line_split[1]
	life = int(line_split[2])

	if network_type not in res:
		res[network_type] = {}

	if UAV_mode not in res[network_type]:
		res[network_type][UAV_mode] = []

	res[network_type][UAV_mode].append(life)

for network_type in res.keys():
	for UAV_mode in res[network_type].keys():
		output_line = network_type + ',' + UAV_mode

		cnt = len(res[network_type][UAV_mode])
		average = 1.0 * sum(res[network_type][UAV_mode])/cnt
		std = ComputeStd(res[network_type][UAV_mode])

		output_line += ',' + str(cnt)
		output_line += ',' + str(average)
		output_line += ',' + str(std)

		output_file.write(output_line)
		output_file.write('\n')


input_file.close()
output_file.close()
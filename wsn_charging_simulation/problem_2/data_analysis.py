input_file_name = "res_prob2_outdoor_faraway_11_20.csv"
output_file_name = input_file_name[:-4] + '_aggregated.csv'

input_file = open(input_file_name, "r")
output_file = open(output_file_name, "w")

res = []
for line in input_file.readlines():
	line_split = line[:-1].split(",")
	network_type = line_split[0]
	UAV_mode = line_split[1]
	life = int(line_split[2])

	if network_type not in res:
		res[network_type] = {}

	if UAV_mode not in res[network_type]:
		res[network_type][UAV_mode] = []

	res[network_type][UAV_mode].append(life)
import sys;

inputFileName = sys.argv[1];
inputFile = open(inputFileName,"r")

count = 10
radius = 0.0
for i in range(0,count):
	line = inputFile.readline()
	strs = line.split(" ")
	#print strs[5][]
	radius += float(strs[5])
	line = inputFile.readline()
	line = inputFile.readline()
print radius/count

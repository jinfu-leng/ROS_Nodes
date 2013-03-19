import sys
import matplotlib.pyplot as plt
import numpy
sys.path.append('../trajectoryPredictor/bin')
import ballPredictor


# read line with ignoring lines which begin with '#'
def ReadLine(inputFile):
	while(True):
		line = inputFile.readline()
		if(line!= "" and line[0]!="#" and line[0]!="\n"):
			return line


# plot based on term a, b, c
def PlotABC(a, b, c, xmin, xmax, label):
	xs = []
	ys = []
	i = xmin
	while(i <= xmax):
		xs.append(i)
		ys.append(a*i*i+b*i+c)
		i += 0.1
	plt.plot(xs,ys,label = label)	


# get data file name from the arguments
if(len(sys.argv)!=2):
	print "Please sepcify the data file"
	exit(1)
else:
	inputName = sys.argv[1]

inputFile = open(inputName,"r")

# read the term a, b, c
strLine = ReadLine(inputFile)
strArray = strLine.split(",")
termA = float(strArray[0])
termB = float(strArray[1])
termC = float(strArray[2])

# read the detected positions 
strLine = ReadLine(inputFile)
count = int(strLine)
positions = []
xs = []
ys = []
zs = []
ts = []
for i in range(0,count):
	strLine = ReadLine(inputFile)
	strArray = strLine.split(",")
	x = float(strArray[0])
	y = float(strArray[1])
	z = float(strArray[2])
	timeStamp = long(strArray[3])
	xs.append(x)
	ys.append(y)
	zs.append(z)
	ts.append(timeStamp)
	positions.append([x,y,z,timeStamp])

# read the real positions 
strLine = ReadLine(inputFile)
count = int(strLine)
positions_r = []
xs_r = []
ys_r = []
zs_r = []
for i in range(0,count):
	strLine = ReadLine(inputFile)
	strArray = strLine.split(",")
	x = float(strArray[0])
	y = float(strArray[1])
	z = float(strArray[2])
	xs_r.append(x)
	ys_r.append(y)
	zs_r.append(z)	
	positions_r.append([x,y,z])


# plot the data

#-- get some values for plotting --
zmax = max(zs)
zmin = min(zs)
zmax_r = max(zs_r)
zmin_r = min(zs_r)
lTrajectory = 0
rTrajectory = zmax_r

#-- plot the detected positions --
if(ReadLine(inputFile)[0]=="Y"):
	plt.plot(zs,ys,"r--", label = "D")
	plt.plot(zs,ys,"rD")

#-- plot the real positions --
if(ReadLine(inputFile)[0]=="Y"):
	plt.plot(zs_r,ys_r,"g^", label = "R")

#-- plot the theoretical line --
if(ReadLine(inputFile)[0]=="Y"):
	PlotABC(termA, termB, termC, lTrajectory, rTrajectory, "T")



#-- plot the trendlines of the detected data --
if(ReadLine(inputFile)[0]=="O"):
	fit = numpy.polyfit(zs,ys,2)
	PlotABC(fit[0], fit[1], fit[2], lTrajectory, rTrajectory, "F")

#-- plot the predicted trajectory
if(ReadLine(inputFile)[0]=="Y"):
	fit = ballPredictor.TrajectoryTerms_reject(zs,ys)
	#PlotABC(fit[0], fit[1], fit[2], lTrajectory, rTrajectory, "MRJ")

	fit = ballPredictor.TrajectoryTerms_reject2(zs,ys)
	PlotABC(fit[0], fit[1], fit[2], lTrajectory, rTrajectory, "MRJ2")

	fit = ballPredictor.TrajectoryTerms_reject2_timestamp(zs,ys,ts)
	PlotABC(fit[0], fit[1], fit[2], lTrajectory, rTrajectory, "MRJ2T")

plt.legend(loc = "lower center")
plt.show()

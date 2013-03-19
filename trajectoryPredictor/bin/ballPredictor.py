import numpy
import math

# Reject the positions which are out of the specified range
def Reject_OutofRange(xs, ys, xMinRange, xMaxRange, yMinRange, yMaxRange):
	positions = []

	xs_tmp = []
	ys_tmp = []
	listLen = len(xs)
	for i in range(0,listLen):
		if(xs[i]>=xMinRange and xs[i]<=xMaxRange and ys[i]>=yMinRange and ys[i]<=yMaxRange):
			xs_tmp.append(xs[i])
			ys_tmp.append(ys[i])

	positions.append(xs_tmp)
	positions.append(ys_tmp)
	return positions

# Reject the positions which are out of the specified range (Timestamp Version)
def Reject_OutofRange_T(xs, ys, ts, xMinRange, xMaxRange, yMinRange, yMaxRange):
	positions = []

	xs_tmp = []
	ys_tmp = []
	ts_tmp = []
	listLen = len(xs)
	for i in range(0,listLen):
		if(xs[i]>=xMinRange and xs[i]<=xMaxRange and ys[i]>=yMinRange and ys[i]<=yMaxRange):
			xs_tmp.append(xs[i])
			ys_tmp.append(ys[i])
			ts_tmp.append(ts[i])

	positions.append(xs_tmp)
	positions.append(ys_tmp)
	positions.append(ts_tmp)
	return positions

# Reject the positions which are unordered
def Reject_UnOrdered(xs, ys):
	listLen = len(xs)

	if(listLen < 2):
		return [xs, ys]
	positions = []	

	xs_tmp = []
	ys_tmp = []
	listLen = len(xs)
	for i in range(0,listLen-1):
		if(xs[i]>xs[i+1]):
			xs_tmp.append(xs[i])
			ys_tmp.append(ys[i])
	xs_tmp.append(xs[listLen-1])
	ys_tmp.append(ys[listLen-1])

	positions.append(xs_tmp)
	positions.append(ys_tmp)
	return positions

# Reject the positions which are unordered (Timestamp version)
def Reject_UnOrdered_T(xs, ys, ts):
	listLen = len(xs)

	if(listLen < 2):
		return [xs, ys, ts]	
	
	positions = []	
	xs_tmp = []
	ys_tmp = []
	ts_tmp = []
	
	for i in range(0,listLen-1):
		if(xs[i]>xs[i+1]):
			xs_tmp.append(xs[i])
			ys_tmp.append(ys[i])
			ts_tmp.append(ts[i])
	xs_tmp.append(xs[listLen-1])
	ys_tmp.append(ys[listLen-1])
	ts_tmp.append(ts[listLen-1])

	positions.append(xs_tmp)
	positions.append(ys_tmp)
	positions.append(ts_tmp)
	return positions

def Mean1(l):
	return sum(l)/len(l)

def Mean2(l):
	sum2 = 0.0
	for i in range(0, len(l)):
		sum2 = sum2 + l[i]*l[i]
	res = math.sqrt(sum2/len(l))
	if (Mean1(l)<0):
		 res = res * -1.0
	return res

def Mean3(l):
	product = 1.0
	for i in range(0, len(l)):
		product = product * l[i]
	res = math.pow(abs(product),1.0/len(l))
	if (Mean1(l)<0):
		 res = res * -1.0
	return res

def TrajectoryTerms_reject2_timestamp(xs, ys, ts):
	# Parameters
	xMinRange = 0.5
	xMaxRange = 10.5
	yMinRange = -3.5
	yMaxRange = 5.5
	gravity = 9.8/1000000.0

	positions = Reject_OutofRange_T(xs, ys, ts, xMinRange, xMaxRange, yMinRange, yMaxRange)
	xs = positions[0]
	ys = positions[1]
	ts = positions[2]
	
	listLen = len(xs)
	if(listLen<3):
		return [0.0, 0.0, 0.0]
	positions = Reject_UnOrdered_T(xs, ys, ts)
	xs = positions[0]
	ys = positions[1]
	ts = positions[2]
	
	listLen = len(xs)
	# Too little points to get the terms
	if(listLen<3):
		return [0.0, 0.0, 0.0]
	
	xSpeeds = []
	ySpeeds = []

	as_ = []
	bs = []
	cs = []
	for i in range(1, listLen):
		t = (ts[i] - ts[i-1])/1000000.0 # unit (ms)
		xSpeed = (xs[i] - xs[i-1])/t # unit (m/ms)
		ySpeed = (ys[i] - ys[i-1])/t # unit (m/ms)
		ySpeedInitial = ySpeed + (t/2 + (ts[i-1] - ts[0])/1000000.0) * (gravity)
		xSpeeds.append(xSpeed)
		ySpeeds.append(ySpeedInitial)
		#xSpeedAverage = (Mean1(xSpeeds) + Mean3(xSpeeds))/2
		xSpeedAverage = Mean2(xSpeeds)	
		ySpeedInitialAverage = Mean3(ySpeeds)
	#print "xspeed", Mean1(xSpeeds), Mean2(xSpeeds), Mean3(xSpeeds)
	#print "yspeed", Mean1(ySpeeds), Mean2(ySpeeds), Mean3(ySpeeds)


	for i in range(0, listLen):
		a = -0.5 * (gravity) / (xSpeedAverage * xSpeedAverage)
		b = ySpeedInitialAverage/xSpeedAverage-0.5*gravity*(-2*xs[i])/(xSpeedAverage * xSpeedAverage)
		c = ys[i] - ySpeedInitialAverage * xs[i] / xSpeedAverage - 0.5 * gravity * (xs[i] * xs[i])/(xSpeedAverage * xSpeedAverage)
		as_.append(a)		
		bs.append(b)
		cs.append(c)
	a = Mean2(as_)
	b = Mean2(bs)
	c = Mean2(cs)
	
	a = -0.5 * (gravity) / (xSpeedAverage * xSpeedAverage)
	b = ySpeedInitialAverage/xSpeedAverage-0.5*gravity*(-2*xs[1])/(xSpeedAverage * xSpeedAverage)
	c = ys[1] - ySpeedInitialAverage * xs[1] / xSpeedAverage - 0.5 * gravity * (xs[1] * xs[1])/(xSpeedAverage * xSpeedAverage)

	a = as_[0]
	b = bs[0]
	c = cs[0]
	

	fit = [a, b, c]
	return fit


def TrajectoryTerms_reject2(xs, ys):
	
	# Parameters
	xMinRange = 0.5
	xMaxRange = 10.5
	yMinRange = -3.5
	yMaxRange = 5.5

	positions = Reject_OutofRange(xs, ys, xMinRange, xMaxRange, yMinRange, yMaxRange)
	xs = positions[0]
	ys = positions[1]
	
	lenList = len(xs)
	if(lenList<3):
		return [0.0, 0.0, 0.0]

	positions = Reject_UnOrdered(xs,ys)
	xs = positions[0]
	ys = positions[1]
	
	lenList = len(xs)
	if(lenList<3):
		return [0.0, 0.0, 0.0]	

	fit = numpy.polyfit(xs, ys, 2)
	return fit

def TrajectoryTerms_reject(xs, ys):
	positions = Reject_UnOrdered(xs,ys)
	xs = positions[0]
	ys = positions[1]

	fit = numpy.polyfit(xs, ys, 2)
	return fit

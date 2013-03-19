#!/usr/bin/env python
import roslib; roslib.load_manifest('trajectoryPredictor')
import rospy
from ballPositioner.msg import ballPosition
from trajectoryPredictor.msg import termABC

import ballPredictor

xs = []
ys = []
zs = []
ts = []
def ReceiveBallPosition(data):	
	global xs, ys, zs, ts	
	if(data.z < 0):
		xs = []
		ys = []
		zs = []
		ts = []
		return
	xs.append(data.x)
	ys.append(data.y)
	zs.append(data.z)
	ts.append(data.nSecond)
	fit = ballPredictor.TrajectoryTerms_reject2_timestamp(zs,ys,ts)
	print "TrajectoryTerms_reject2_timestamp", fit
	fit = ballPredictor.TrajectoryTerms_reject2(zs,ys)
	print "TrajectoryTerms_reject2", fit

def Init():
	print "ballPredictor Started"
	positions = []
	pub = rospy.Publisher('trajectoryTerms', termABC)
	rospy.init_node('trajectory_Predictor')
	sub = rospy.Subscriber('ballPosition', ballPosition, ReceiveBallPosition)
	rospy.spin()
if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException: pass

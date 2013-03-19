import subprocess, time, random
xMin = 4.1
xMax = 4.3
yMin = -0.2
yMax = 0.2
zMin = 1.4
zMax = 1.6
xLinearMin = 4.0
xLinearMax = 4.5
yLinearMin = 0.0
yLinearMax = 0.0
zLinearMin = 2.0
zLinearMax = 2.2
period = 1
ballModelList = ["small_ball_red","small_ball_yellow","small_ball_blue"]
while True:
	ballModelName = ballModelList[random.randrange(0,len(ballModelList))]
	x = xMin + random.random()*(xMax-xMin)
	y = yMin + random.random()*(yMax-yMin)
	z = zMin + random.random()*(zMax-zMin)
	xLinear = -1.0 * (xLinearMin + random.random()*(xLinearMax-xLinearMin))
	yLinear = yLinearMin + random.random()*(yLinearMax-yLinearMin)
	zLinear = zLinearMin + random.random()*(zLinearMax-zLinearMin)
	throwBallCommand = "rostopic pub -1 gazebo/set_model_state gazebo/ModelState '{model_name: " + ballModelName + ", pose: { position: { x: " + str(x) +", y: " + str(y) + ", z: " + str(z) + "}, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: " + str(xLinear) + ", y: " + str(yLinear) + ", z: " + str(zLinear) + "}, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'"
	subprocess.call([throwBallCommand],shell = True)
	time.sleep(period)
	

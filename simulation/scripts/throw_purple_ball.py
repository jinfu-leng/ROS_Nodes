import subprocess, time, random
x = 5.0
y = 0
z = 0.5 
xLinear = -6.0
yLinear = 0.0
zLinear = 4.5
ballModelName = "small_ball_purple"
throwBallCommand = "rostopic pub -1 gazebo/set_model_state gazebo/ModelState '{model_name: " + ballModelName + ", pose: { position: { x: " + str(x) +", y: " + str(y) + ", z: " + str(z) + "}, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: " + str(xLinear) + ", y: " + str(yLinear) + ", z: " + str(zLinear) + "}, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'"
subprocess.call([throwBallCommand],shell = True)
	

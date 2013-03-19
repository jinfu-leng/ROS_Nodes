import subprocess, sys
ballModelName = "quadrotor"
x = 0
y = 0 
z = 1.0

xLinear = 0
yLinear = 0
zLinear = 0

moveBallCommand = "rostopic pub -1 gazebo/set_model_state gazebo/ModelState '{model_name: " + ballModelName + ", pose: { position: { x: " + str(x) +", y: " + str(y) + ", z: " + str(z) + "}, orientation: {x: 0, y: 0, z: -0.380188350389, w: 0.924909086467 } }, twist: { linear: { x: " + str(xLinear) + ", y: " + str(yLinear) + ", z: " + str(zLinear) + "}, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'"
subprocess.call([moveBallCommand],shell = True)
	

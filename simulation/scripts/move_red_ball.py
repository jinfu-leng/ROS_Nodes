import subprocess, sys
ballModelName = "small_ball_red"
x = 5
y = 0 
z = 5
# the first argument is the name of the program
if(len(sys.argv)==4):
	x = sys.argv[1]
	y = sys.argv[2]
	z = sys.argv[3]

xLinear = 0
yLinear = 0
zLinear = 0

moveBallCommand = "rostopic pub -1 gazebo/set_model_state gazebo/ModelState '{model_name: " + ballModelName + ", pose: { position: { x: " + str(x) +", y: " + str(y) + ", z: " + str(z) + "}, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: { x: " + str(xLinear) + ", y: " + str(yLinear) + ", z: " + str(zLinear) + "}, angular: { x: 0, y: 0, z: 0}  }, reference_frame: world }'"
subprocess.call([moveBallCommand],shell = True)
	

import subprocess
import sys

x = 5
y = 5
z = 5
# the first argument is the name of the program
if(len(sys.argv)==4):
	x = sys.argv[1]
	y = sys.argv[2]
	z = sys.argv[3]

createYellowBall = "rosrun gazebo spawn_model  -gazebo -file /home/leng/ros/projects/simulation/models/small_ball_red_nogravity.model  -z {2} -x {0} -y {1} -model small_ball_red".format(x,y,z)

subprocess.call([createYellowBall], shell = True)

import subprocess
subprocess.call(["rosrun gazebo spawn_model  -gazebo -file /home/leng/ros/projects/simulation/models/small_ball_yellow.model  -z 0.2 -x 4.0 -y 1.0 -model small_ball_yellow"], shell = True)
subprocess.call(["rosrun gazebo spawn_model  -gazebo -file /home/leng/ros/projects/simulation/models/small_ball_red.model  -z 0.2 -x 4.0 -y 0.0 -model small_ball_red"], shell = True)
subprocess.call(["rosrun gazebo spawn_model  -gazebo -file /home/leng/ros/projects/simulation/models/small_ball_blue.model  -z 0.2 -x 4.0 -y -1.0 -model small_ball_blue"], shell = True)


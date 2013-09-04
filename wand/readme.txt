How to run it:
1. Plug in the Xbee and right click the USB icon at the bottom right of the virtual machine, and click the "FTDI FT232 USB UART" one. And then run "sudo chmod 777 /dev/ttyUSB0".
2. Change the parameter("object_name") of the node("conv_q_p") in the launch file "wand.launch" according to the drone you want to fly;
3. Put the drone (heading north) and the wand (point to north) in the cage and make sure they are shown up in the vicon system (the object name of the wand is "nwand");
4. Launch the vicon by "roslaunch vicon_bridge vicon.launch".
5. Launch the launch file "wand.launch";
6. Once you lift the wand higher than 1.8m, the drone should launch. 
7. Once you put the wand lower than 0.3m, the drone should land.
8. When you hold the wand horizontally (point the wand forward and make all markers face up), the drone should follow the wand.
9. When you hold the wand vertically (point the wand to the ceiling), the drone should hover without follow the wand.
10. when you hold the wand horizontally and turn the wand and make the roll of the wand almost 90 or -90 degree, the drone will rotate itself.


Notes about the package:

This package is for the demo of flying the drone with the wand.

The "wand.launch" are the example launch file. The node "wand_ctrl" does the wand control job. The specific parameters of the node are as follows:
-The parameter "away" indicates how far away will the drone keep between the wand and itself.

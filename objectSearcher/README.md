objectSearcher
=========

Use the onboard camera to detect the marker on the ground, and then try to hover and land on this marker.


How to set up?
=========
1. Set up the Vicon

2. Prepare the hardware
Connect your laptop with the correct zigbees, one for control, and one for camera (the zigbee for control should be plugged in firstly);
Put the battery on the UAV; 

3. Check the Ball Detection
Launch the file “rxtx_receive.launch”;
Turn on the bottom board of the UAV, which is used for the camera; (if everything goes well, you would see ball detection outputs on the terminal  of “rxtx_receive.launch” after around one minute. If there is nothing on the terminal, the launch of the bottom board may be stuck, please connect the screen and keyboard to see what happened. Normally, you just need the press “enter” to assist the launch of the system)

4. Start your demo
Put the UAV on the center of the cage, and turn on the top board;
Make sure that the purple spot in the cage is uncovered;
Launch the file “objectSearcher.launch”;
Give the control of the UAV to the computer with the switch on the controller;

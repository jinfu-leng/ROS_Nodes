#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Char.h>
#include <rxtx/rxtxData.h>

class Controller
{
public:
	Controller();  // constructor
		
private:
	ros::NodeHandle nh;	
			
	void callbackMotorRotation(std_msgs::Char motor_rotation);
	
	// publisher to rxtx
	ros::Publisher motor_rotation_pub;
		
	// subscribe to ball
	ros::Subscriber motor_rotation_sub; 
};

Controller::Controller()
{		
// Publish
	motor_rotation_pub = nh.advertise<rxtx::rxtxData>("/rxtx/send", 1);	
// Subscribe	
	motor_rotation_sub = nh.subscribe<std_msgs::Char>("motor_rotation", 1, &Controller::callbackMotorRotation, this);
}

void Controller::callbackMotorRotation(std_msgs::Char motor_rotation)
{	
	ROS_INFO("Received a message!");
	rxtx::rxtxData out_data;
	out_data.header.stamp = ros::Time::now();
	// serial data
	std::vector<unsigned char> data;

	char var = motor_rotation.data;
	//data.push_back((var>>24)&0xFF);	
	//data.push_back((var>>16)&0xFF);	
	//data.push_back((var>>8)&0xFF);	
	data.push_back(var);	
	
	out_data.data = data;
	
	ROS_INFO("Sent out the data!");
	motor_rotation_pub.publish(out_data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_rotation_controller");
	Controller c;

	ros::spin();
	return 0;
}	

// includes
#include <demo_UAV_controller.h>

// defines
#define ROS_PKG "demo_UAV_controller"
#define ROS_NODE "demo_UAV_controller"

// main function
int main(int32_t argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s\n", ROS_NODE);

  // create class instance
  demo_UAV_controller controller;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s\n", ROS_NODE);

  // return success
  return(0);           
}

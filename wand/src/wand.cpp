


// includes
#include <wand.h>

// defines
#define ROS_PKG "replay"
#define ROS_NODE "replay"


// class constructor
Wand::Wand(void)
  : nh_(),
    private_nh_("~")
{
  // get params
  initParams();

  // publishers
  initPublishers();

  // subscribers
  initSubscribers();

  // start ctrl thread
  startCtrlThread();
}


// class destructor
Wand::~Wand(void) { }

// main function
int main(int argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s", ROS_NODE);

  // create class instance
  Wand wand;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s", ROS_NODE);

  // return success
  return(0);           
}

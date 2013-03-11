/**
 * \file arb_subject_ctrl.cpp
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 May 2011
 *
 */


// includes
#include <topic_arbitration.h>


// defines
#define ROS_PKG "topic_arbitration"
#define ROS_NODE "topic_arbitration"


// constructor
ArbSubjectCtrl::ArbSubjectCtrl(void)
  : nh_(),
    private_nh_("~")
{
  // get params
  initParams();

  // publishers
  initPublishers();

  // subscribers
  initSubscribers();
}


// destructor
ArbSubjectCtrl::~ArbSubjectCtrl(void) { }


// main function
int main(int32_t argc, char **argv)
{
  // initialize ros node handle
  ros::init(argc, argv, ROS_NODE);

  // print start to log file
  ROS_INFO("Started node %s\n", ROS_NODE);

  // create class instance
  ArbSubjectCtrl arb_subject_ctrl;

  // ros spin
  ros::spin();

  // print termination to log file
  ROS_INFO("Stopped node %s\n", ROS_NODE);

  // return success
  return(0);           
}

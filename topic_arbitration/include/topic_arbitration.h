/**
 * \file arb_subject_ctrl.h
 *
 * \author Brian J. Julian
 *
 * \version 0.1
 *
 * \date 01 May 2011
 *
 */


#ifndef __ARB_SUBJECT_CTRL_H__
#define __ARB_SUBJECT_CTRL_H__


// includes
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <topic_tools/shape_shifter.h>
#include "topic_arbitration/topic_source.h"


// namespaces
using namespace std;


// class
class ArbSubjectCtrl
{
public:

  // constructor
  ArbSubjectCtrl(void);

  // destructor
  ~ArbSubjectCtrl(void);

private:

  // ros callbacks
  void callbackShapeShifterMsg(const boost::shared_ptr<topic_tools::ShapeShifter const> &shape_shifter_msg, const vector<uint8_t> &state_mappings);
  void callbackSubjectCtrlStateMsg(const boost::shared_ptr<topic_arbitration::topic_source const> &subject_ctrl_state_msg);

  // init
  void initParams(void);
  void initPublishers(void);
  void initSubscribers(void);

  // ros objects
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // ros parameters
  XmlRpc::XmlRpcValue state_mappings_;

  // ros publishers
  ros::Publisher shape_shifter_pub_;

  // ros subscribers
  ros::Subscriber subject_ctrl_state_sub_;
  vector<ros::Subscriber> shape_shifter_subs_;

  // message containers
  //collab_msgs::SubjectCtrlState subject_ctrl_state_msg_;
  topic_arbitration::topic_source subject_ctrl_state_msg_;
  
  // volatile bools
  volatile bool is_shape_shifter_advertised_;
  volatile bool received_subject_ctrl_state_msg_;
};


#endif // __ARB_SUBJECT_CTRL_H__

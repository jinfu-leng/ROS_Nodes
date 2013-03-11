#include "stdio.h"
#include "ros/ros.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"
#include "math.h"
#include <iostream>

const double min_dist = 1.5;
const double UAV_height = 1.2;
// namespaces
using namespace std;

// class
class demo_UAV_controller
{
public:
  // constructor
  demo_UAV_controller();

private:
  // ros callbacks
  void callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg);
  void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
  void callbackControllerSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
  collab_msgs::SubjectPose ComputeRelativePose(const collab_msgs::SubjectPose &center, const collab_msgs::SubjectPose &subject);
  
  // init functions
  void InitParams();
  void InitPublishers();
  void InitSubscribers();

  // other functions
  void FlytoPoint(double x, double y, double z, double w);
  void ChangeState(int state, double sleepTime=5);
  void Launch();
  void Land();


  // ros objects
  ros::NodeHandle nh_;

  // ros publishers
  ros::Publisher UAV_state_pub_;
  ros::Publisher UAV_pose_pub_;
  ros::Publisher UAV_subject_pose_pub_; 

  // ros subscribers
  ros::Subscriber UAV_subject_ctrl_state_sub_;
  ros::Subscriber UAV_subject_pose_sub_;
  ros::Subscriber controller_subject_pose_sub_;

  // parameters
  collab_msgs::SubjectCtrlState UAV_subject_ctrl_state_;
  collab_msgs::SubjectPose UAV_subject_pose_;
  collab_msgs::SubjectPose controller_subject_pose_;

  // parameters
  bool received_new_UAV_state;

};






// includes
#include <wand.h>


// init param
void Wand::initParams(void)
{
  //ros param
  private_nh_.param("away", away_, 1.0);
  private_nh_.param("ctrl_rate", ctrl_rate_, 20.0);
  
  received_wand_pose_msg_ = false;
  received_subject_ctrl_state_msg_ = false;

  launch_ = false;
  land_ = false;
}


// init publishers
void Wand::initPublishers(void)
{
  task_waypose_pub_ = nh_.advertise<collab_msgs::SubjectPose>("task_waypose", 10, true);
  cmd_subject_ctrl_state_pub_ = nh_.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state", 10, true);
}


// init subscribers
void Wand::initSubscribers(void)
{
  wand_pose_sub_ = nh_.subscribe<collab_msgs::SubjectPose>("wand_pose", 10, &Wand::callbackWandPoseMsg, this);
  subject_ctrl_state_sub_ = nh_.subscribe<collab_msgs::SubjectCtrlState>("subject_ctrl_state", 10, &Wand::callbackSubjectCtrlStateMsg, this);
}

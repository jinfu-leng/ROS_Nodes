// includes
#include <demo_UAV_controller.h>

// constructor
demo_UAV_controller::demo_UAV_controller()
  : nh_()
{
  // ros parameters
  InitParams();

  // create publishers
  InitPublishers();

  // create subscribers
  InitSubscribers();
}

// init ros param
void demo_UAV_controller::InitParams()
{ 
  // init
  UAV_subject_ctrl_state_.state = -1;
  received_new_UAV_state = false;
}

// create publishers
void demo_UAV_controller::InitPublishers()
{
  UAV_state_pub_ = nh_.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state", 10, true);
  UAV_pose_pub_ = nh_.advertise<collab_msgs::SubjectPose>("task_waypose", 1, true);
}

// create subscribers
void demo_UAV_controller::InitSubscribers()
{
  UAV_subject_ctrl_state_sub_ = nh_.subscribe("subject_ctrl_state", 1, &demo_UAV_controller::callbackUAVSubjectCtrlStateMsg, this);
  UAV_subject_pose_sub_ = nh_.subscribe("UAV_pose", 1, &demo_UAV_controller::callbackUAVSubjectPoseMsg, this);
  controller_subject_pose_sub_ = nh_.subscribe("controller_pose", 1, &demo_UAV_controller::callbackControllerSubjectPoseMsg, this); 
}

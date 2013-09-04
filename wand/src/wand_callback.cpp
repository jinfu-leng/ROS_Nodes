


// includes
#include <wand.h>

void Wand::callbackWandPoseMsg(const boost::shared_ptr<collab_msgs::SubjectPose const> &wand_pose_msg){
  mutex_.lock();
  received_wand_pose_msg_ = true;
  wand_pose_msg_ = *wand_pose_msg;
  mutex_.unlock();
}


void Wand::callbackSubjectCtrlStateMsg(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &subject_ctrl_state_msg) {
  mutex_.lock();
  received_subject_ctrl_state_msg_ = true;
  subject_ctrl_state_msg_ = *subject_ctrl_state_msg;

  if (launch_)
  {
    switch(cmd_subject_ctrl_state_msg_.state)
    {
      case(0):
        if(subject_ctrl_state_msg_.state == 0) cmd_subject_ctrl_state_msg_.state = 1;
        break;
      case(1):
        if(subject_ctrl_state_msg_.state == 1) cmd_subject_ctrl_state_msg_.state = 2;
        break;
      case(2):
        if(subject_ctrl_state_msg_.state == 4) cmd_subject_ctrl_state_msg_.state = 5;
		break;
      case(5):
        if(subject_ctrl_state_msg_.state == 7) cmd_subject_ctrl_state_msg_.state = 8;
        break;
      case(8):
        if(subject_ctrl_state_msg_.state == 8) launch_ = false;
        break;
      default:
        break;
    }
  }

  if (land_)
  {
    switch(cmd_subject_ctrl_state_msg_.state)
    {
      case(8):
        if(subject_ctrl_state_msg_.state == 8) cmd_subject_ctrl_state_msg_.state = 7;
        break;
      case(7):
        if(subject_ctrl_state_msg_.state == 7) cmd_subject_ctrl_state_msg_.state = 6;
		break;
      case(6):
        mutex_.unlock();
        ros::Duration(5.0).sleep();
        mutex_.lock();
        if(subject_ctrl_state_msg_.state == 6) cmd_subject_ctrl_state_msg_.state = 4;
		break;
      case(4):
        if(subject_ctrl_state_msg_.state == 4) cmd_subject_ctrl_state_msg_.state = 3;
		break;
      case(3):
        if(subject_ctrl_state_msg_.state == 1) cmd_subject_ctrl_state_msg_.state = 0;
		break;
      case(0):
        if(subject_ctrl_state_msg_.state == 0) land_= false;
		break;
      default:
        break;
    }
  }
  mutex_.unlock();
}

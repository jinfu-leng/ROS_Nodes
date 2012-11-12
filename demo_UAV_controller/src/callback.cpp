// includes
#include <demo_UAV_controller.h>
#include <math.h>

void demo_UAV_controller::callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg)
{
  UAV_subject_ctrl_state_ = subject_ctrl_state_msg;
  received_new_UAV_state = true;
}

void demo_UAV_controller::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg)
{
  UAV_subject_pose_ = subject_pose_msg;
}

void demo_UAV_controller::callbackControllerSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg)
{
  controller_subject_pose_ = subject_pose_msg;

  if(controller_subject_pose_.translation.z>1.5){
    if(received_new_UAV_state==true){
      if(UAV_subject_ctrl_state_.state!=8){
        Launch();
        received_new_UAV_state=false;
      }
    }
  }
  else if(controller_subject_pose_.translation.z<0.3){
     if(received_new_UAV_state==true){
      if(UAV_subject_ctrl_state_.state!=0){
        Land();
        received_new_UAV_state=false;
      }
    }
  }
  else
  {
     double dx= UAV_subject_pose_.translation.x-controller_subject_pose_.translation.x;
     double dy= UAV_subject_pose_.translation.y-controller_subject_pose_.translation.y;
     double dist = sqrt(dx*dx+dy*dy);
     if(dist<min_dist)
     {
        dx=dx*min_dist/dist;
	dy=dy*min_dist/dist;
     }
     
     // compute the angle between the UAV and the controller
     if(dx==0.0) dx=0.0000001;
     double angle = atan(dy/dx);
     const double PI = 3.1415926;
     if(angle>=0)
     {
          if(dy<0) angle=angle+PI;
          else angle=angle;
     }
     else
     {
          if(dy<0) angle=angle+2*PI;
          else angle=angle+PI;
     }

     // angle conversion from the vicon_pose coordinate to normal Gauss coordiante
     if(angle<=0.5*PI) angle = 0.5*PI - angle;
     else angle = 2.5*PI -angle;


     // adjust the head to face to wand
     angle= angle - PI;
     if(angle<0) angle=angle+2*PI;

     if(dist<min_dist||abs(angle-UAV_subject_pose_.rotation.z)>0.5)FlytoPoint(controller_subject_pose_.translation.x+dx,controller_subject_pose_.translation.y+dy,UAV_height,angle);
  }
}

/*
// handle subject control state msg
void Task::callbackSubjectCtrlStateMsg(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &subject_ctrl_state_msg)
{
  subject_ctrl_state_msg_ = *subject_ctrl_state_msg;
  received_subject_ctrl_state_msg_ = true;
  if (!get_state_ && received_subject_pose_msg_)
  {
    //mutex lock
    mutex_.lock();

    switch(ctrl_state_.state)
    {
      case(0):
        if(ctrl_state_.state == 0) get_state_ = true;
        break;
      case(1):
        if(subject_ctrl_state_msg_.state == 1) get_state_ = true;
        break;
      case(2):
        if(subject_ctrl_state_msg_.state == 4) 	get_state_ = true;
		break;
      case(3):
        if(subject_ctrl_state_msg_.state == 1) get_state_ = true;
        break;
      case(4):
        if(subject_ctrl_state_msg_.state == 4) get_state_ = true;
        break;
      case(5):
        if(subject_ctrl_state_msg_.state == 7) get_state_ = true;
        break;
      case(6):
        if(subject_ctrl_state_msg_.state == 6) get_state_ = true;
        break;
      case(7):
        if(subject_ctrl_state_msg_.state == 7) get_state_ = true;
        break;
      case(8):
        if(subject_ctrl_state_msg_.state == 8) get_state_ = true;
        break;
      default:
        break;
    }
    
    // mutex unlock
    mutex_.unlock();
  }
}


// handle subject pose msg
void Task::callbackSubjectPoseMsg(const boost::shared_ptr<collab_msgs::SubjectPose const> &subject_pose_msg)
{

  subject_pose_msg_ = *subject_pose_msg;
  received_subject_pose_msg_ = true;

  if (!get_way_pose_)
  {
    //mutex lock
    mutex_.lock();

    double distX = task_pose_.translation.x - subject_pose_msg_.translation.x;
    double distY = task_pose_.translation.y - subject_pose_msg_.translation.y;
    double distZ = task_pose_.translation.z - subject_pose_msg_.translation.z;
    double dist = sqrt(distX*distX + distY*distY + distZ*distZ);

    double pi = 3.1416;
	double dir_diff = abs(task_pose_.rotation.z - subject_pose_msg_.rotation.z);
    if(dir_diff > pi){
      dir_diff = 2*pi - dir_diff;
    }

    if (dist<pose_error_ && dir_diff<dir_error_){
      get_way_pose_ = true;
    }

    // mutex unlock
    mutex_.unlock();
  }
}


// handle script file msg
void Task::callbackScriptFileMsg(const boost::shared_ptr<std_msgs::String const> &script_file_msg)
{
  mutex_.lock();
  if(!received_script_file_msg_)
  {
    script_file_msg_=*script_file_msg;
    received_script_file_msg_ = true;
  }
  mutex_.unlock();
}
*/



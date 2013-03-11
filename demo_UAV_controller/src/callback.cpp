// includes
#include <demo_UAV_controller.h>
#include <math.h>

collab_msgs::SubjectPose demo_UAV_controller::ComputeRelativePose(const collab_msgs::SubjectPose &center, const collab_msgs::SubjectPose &subject){
  collab_msgs::SubjectPose res = subject;
  res.translation.x = subject.translation.x - center.translation.x;
  res.translation.y = subject.translation.y - center.translation.y;
  res.translation.z = subject.translation.z - center.translation.z;
  //res.rotation.x = 0;
  //res.rotation.y = 0;
  res.rotation.z = subject.rotation.z - center.rotation.z;
  return res;
}

void demo_UAV_controller::callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg)
{
  UAV_subject_ctrl_state_ = subject_ctrl_state_msg;
  received_new_UAV_state = true;
}

void demo_UAV_controller::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg)
{
  UAV_subject_pose_ = subject_pose_msg;
  collab_msgs::SubjectPose UAV_relative_pose = ComputeRelativePose(controller_subject_pose_,UAV_subject_pose_);
  UAV_subject_pose_pub_.publish(UAV_relative_pose);
}

void demo_UAV_controller::callbackControllerSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg)
{
  controller_subject_pose_ = subject_pose_msg;

  if(controller_subject_pose_.translation.z>2.0){
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
    if(UAV_subject_ctrl_state_.state!=8) return;
    FlytoPoint(1,0,0.2,0);

/*
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

     if(dist<min_dist||abs(angle-UAV_subject_pose_.rotation.z)>0.5)FlytoPoint(controller_subject_pose_.translation.x+dx,controller_subject_pose_.translation.y+dy,UAV_height,angle);*/
  }
}

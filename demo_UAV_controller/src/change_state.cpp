#include <demo_UAV_controller.h>

void demo_UAV_controller::ChangeState(int state, double sleepTime) {
  collab_msgs::SubjectCtrlState subject_ctrl_state_;
  if (UAV_subject_ctrl_state_.state != state) {
    subject_ctrl_state_ = UAV_subject_ctrl_state_;
    subject_ctrl_state_.state = state;
    UAV_state_pub_.publish(subject_ctrl_state_);
    ros::Duration(sleepTime).sleep();
  }
}

void demo_UAV_controller::Launch() {
  if(UAV_subject_ctrl_state_.state!=8){
    if(UAV_subject_ctrl_state_.state<1) ChangeState(1);
    if(UAV_subject_ctrl_state_.state<2) ChangeState(2,7);
    if(UAV_subject_ctrl_state_.state<5) ChangeState(5);
    if(UAV_subject_ctrl_state_.state<8) ChangeState(8,2);
    FlytoPoint(UAV_subject_pose_.translation.x,UAV_subject_pose_.translation.y,UAV_height,UAV_subject_pose_.rotation.x);
  }
}

void demo_UAV_controller::Land() {
  if (UAV_subject_ctrl_state_.state==8) {
    if(UAV_subject_ctrl_state_.state>7) ChangeState(7);
    if(UAV_subject_ctrl_state_.state>6) ChangeState(6,5);
    if(UAV_subject_ctrl_state_.state>4) ChangeState(4);
    if(UAV_subject_ctrl_state_.state>3) ChangeState(3);
    if(UAV_subject_ctrl_state_.state>0) ChangeState(0,1);
  }
}

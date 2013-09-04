

// includes
#include <wand.h>

// start ctrl thread
void Wand::startCtrlThread(void)
{
  ctrl_thread_ = boost::shared_ptr<boost::thread>
    (new boost::thread(boost::bind(&Wand::ctrlThread, this)));
}


// ctrl thread
void Wand::ctrlThread(void)
{
  startTaskPubThread();
  startStatePubThread();

  ros::Rate rate(ctrl_rate_);
  while(ros::ok())
  {
    mutex_.lock();
    if(received_wand_pose_msg_
       && received_subject_ctrl_state_msg_){

      if(wand_pose_msg_.translation.z > 1.8
         && subject_ctrl_state_msg_.state == 0){
        launch_ = true;
      }else if(wand_pose_msg_.translation.z < 0.3
	           && subject_ctrl_state_msg_.state == 8){
        land_= true;
      }else if(wand_pose_msg_.rotation.y < 0.4
	           && wand_pose_msg_.rotation.y > -0.4){
        const double PI = 3.1416;
        if(wand_pose_msg_.rotation.x > -0.8
	       && wand_pose_msg_.rotation.x < 0.8){
          task_waypose_msg_.translation.x = wand_pose_msg_.translation.x + away_*sin(wand_pose_msg_.rotation.z);
          task_waypose_msg_.translation.y = wand_pose_msg_.translation.y + away_*cos(wand_pose_msg_.rotation.z);
          task_waypose_msg_.translation.z = wand_pose_msg_.translation.z + away_*sin(wand_pose_msg_.rotation.y);
		  task_waypose_msg_.rotation.z = wand_pose_msg_.rotation.z + PI;
        }else if(wand_pose_msg_.rotation.x <= -0.8){
		  task_waypose_msg_.rotation.z = fmod(task_waypose_msg_.rotation.z - PI/30, 2*PI);
        }else if(wand_pose_msg_.rotation.x >= 0.8){
		  task_waypose_msg_.rotation.z = fmod(task_waypose_msg_.rotation.z + PI/30, 2*PI);
        }
      }
    }
    
    mutex_.unlock();
    rate.sleep();
  }
}


// stop ctrl thread
void Wand::stopCtrlThread(void)
{
  ctrl_thread_->join();
}


// start task pub thread
void Wand::startTaskPubThread(void)
{
  task_pub_thread_ = boost::shared_ptr<boost::thread>
    (new boost::thread(boost::bind(&Wand::taskPubThread, this)));
}


// task pub thread
void Wand::taskPubThread(void)
{
  ros::Rate pub_rate(ctrl_rate_);
  while(ros::ok())
  { 
    mutex_.lock();
    task_waypose_pub_.publish(task_waypose_msg_);
    mutex_.unlock();
    pub_rate.sleep();
  }
}


// stop task pub thread
void Wand::stopTaskPubThread(void)
{
  task_pub_thread_->join();
}


// start state pub thread
void Wand::startStatePubThread(void)
{
  state_pub_thread_ = boost::shared_ptr<boost::thread>
    (new boost::thread(boost::bind(&Wand::statePubThread, this)));
}


// state pub thread
void Wand::statePubThread(void)
{
  ros::Rate pub_rate(ctrl_rate_);
  while(ros::ok())
  {
    mutex_.lock();
    cmd_subject_ctrl_state_pub_.publish(cmd_subject_ctrl_state_msg_);
    mutex_.unlock();
    pub_rate.sleep();
  }
}


// stop state pub thread
void Wand::stopStatePubThread(void)
{
  state_pub_thread_->join();
}


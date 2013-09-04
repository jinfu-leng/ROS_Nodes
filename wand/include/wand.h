


// includes
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <collab_msgs/SubjectPose.h>
#include <collab_msgs/SubjectCtrlState.h>
#include <math.h>



// namespace
using namespace std;


// class
class Wand
{
public:
 
  // constructor
  Wand();

  // destructor
  ~Wand();

private:

  // ctrl thread
  void startCtrlThread(void);
  void ctrlThread(void);
  void stopCtrlThread(void);

  void startTaskPubThread(void);
  void taskPubThread(void);
  void stopTaskPubThread(void);

  void startStatePubThread(void);
  void statePubThread(void);
  void stopStatePubThread(void);

  // subscriber callbacks
  void callbackWandPoseMsg(const boost::shared_ptr<collab_msgs::SubjectPose const> &wand_pose_msg);
  void callbackSubjectCtrlStateMsg(const boost::shared_ptr<collab_msgs::SubjectCtrlState const> &subject_ctrl_state_msg);

  // initializers
  void initParams(void);
  void initPublishers(void);
  void initSubscribers(void);

  // ros objects
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // thread objects
  boost::mutex mutex_;
  boost::shared_ptr<boost::thread> ctrl_thread_;
  boost::shared_ptr<boost::thread> task_pub_thread_;
  boost::shared_ptr<boost::thread> state_pub_thread_;

  // ros parameters
  double ctrl_rate_;
  double away_;

  // ros publishers
  ros::Publisher task_waypose_pub_;
  ros::Publisher cmd_subject_ctrl_state_pub_;

  // ros subscribers
  ros::Subscriber wand_pose_sub_;
  ros::Subscriber subject_ctrl_state_sub_;

  // message containers
  collab_msgs::SubjectPose wand_pose_msg_;
  collab_msgs::SubjectPose task_waypose_msg_;
  collab_msgs::SubjectCtrlState subject_ctrl_state_msg_;
  collab_msgs::SubjectCtrlState cmd_subject_ctrl_state_msg_;

  // volatile bools
  volatile bool received_wand_pose_msg_;
  volatile bool received_subject_ctrl_state_msg_;
  volatile bool launch_;
  volatile bool land_;

};



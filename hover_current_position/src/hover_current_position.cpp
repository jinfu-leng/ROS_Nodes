#include "ros/ros.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"
#include "topic_arbitration/topic_source.h"

using namespace std;


class HoverCurrentPosition{
public:
	HoverCurrentPosition();
	~HoverCurrentPosition(){
	}
private:
	ros::NodeHandle nh;

	// subscribe	
	//ros::Subscriber UAV_subject_ctrl_state_sub_;
	ros::Subscriber UAV_subject_pose_sub_;
	ros::Subscriber pose_source_sub_;

	// publish
	//ros::Publisher UAV_cmd_ctrl_state_pub_;
	ros::Publisher UAV_subject_pose_pub_;


	// other parameters
	//collab_msgs::SubjectCtrlState UAV_subject_ctrl_state_;
	collab_msgs::SubjectPose UAV_subject_pose_;
	int lastState;
	

	// callback functions
	//void callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg);
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
	void callbackPoseTopicSourceMsg(const topic_arbitration::topic_source &topic_source_msg);
	
	// other function
	void FlytoPoint(double x, double y, double z, double w);
};

// construction
HoverCurrentPosition::HoverCurrentPosition(){
	
	// initial
	UAV_subject_pose_.translation.x = UAV_subject_pose_.translation.y = 0;
	UAV_subject_pose_.translation.z = 1;
	lastState = 0;
	
	// subscribe	
	// the state and pose of UAV
	//UAV_subject_ctrl_state_sub_ = nh.subscribe("subject_ctrl_state", 1, &ObjectSearcher::callbackUAVSubjectCtrlStateMsg, this);
	UAV_subject_pose_sub_ = nh.subscribe("UAV_pose", 1, &HoverCurrentPosition::callbackUAVSubjectPoseMsg, this);
	pose_source_sub_ = nh.subscribe("pose_source", 10, &HoverCurrentPosition::callbackPoseTopicSourceMsg, this);
	
	// publish
	// the state and pose of UAV
	//UAV_state_pub_ = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state", 10, true);
  	UAV_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("task_waypose", 1, true);
}

// callback functions
void HoverCurrentPosition::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	UAV_subject_pose_ = subject_pose_msg;	
}
void HoverCurrentPosition::callbackPoseTopicSourceMsg(const topic_arbitration::topic_source &topic_source_msg){
	//printf("laststate %d\n",lastState);
	if(topic_source_msg.state==1){
		lastState = topic_source_msg.state;
		return;
	}
	
	if(lastState==0) return;
	else lastState = 0;
	//printf("Fly\n");
	FlytoPoint(UAV_subject_pose_.translation.x,UAV_subject_pose_.translation.y,UAV_subject_pose_.translation.z,UAV_subject_pose_.rotation.z);
}

// other functions
void HoverCurrentPosition::FlytoPoint(double x, double y, double z, double w){
	//collab_msgs::SubjectPose subject_pose = UAV_subject_pose_;
	collab_msgs::SubjectPose subject_pose;
	subject_pose.translation.x = x;
	subject_pose.translation.y = y;
	subject_pose.translation.z = z;
	subject_pose.rotation.z = w;
	UAV_subject_pose_pub_.publish(subject_pose);
}

// main
int main(int argc, char** argv){
	ros::init(argc, argv, "HoverCurrentPosition");
	ROS_INFO("HoverCurrentPosition Started\n");
	HoverCurrentPosition hcp;
	ros::spin();
	return 0;
}

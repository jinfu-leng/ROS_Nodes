#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"
#include "collab_msgs/PidParams.h"
#include "topic_arbitration/topic_source.h"

using namespace std;


class ImageFlySafety{
public:
	ImageFlySafety();
	~ImageFlySafety(){
	}
private:
	ros::NodeHandle nh;
	collab_msgs::SubjectPose last_task_waypose;
	bool get_subject_pose;
	ros::Timer timer;
	int current_topic_source;
	collab_msgs::PidParams global_pid_params, relative_pid_params;

	// subscribe	
	ros::Subscriber UAV_subject_pose_sub_;
	ros::Subscriber UAV_task_waypose_sub_;
	ros::Subscriber topic_source_sub_;
	ros::Subscriber UAV_relative_pid_sub_;
	
	// publish
	ros::Publisher UAV_subject_pose_pub_;
	ros::Publisher UAV_pid_pub_;

	// callback functions
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
	void callbackUAVTaskWayposeMsg(const collab_msgs::SubjectPose &task_waypose_msg);
	void callbackTimer(const ros::TimerEvent& event);
	void callbackTopicSourceMsg(const topic_arbitration::topic_source &topic_source_msg);	
	void callbackRelativePIDParamsMsg(const collab_msgs::PidParams &pid_params_msg);

	// other function
	void GiveFakeSubjectPose(double x, double y, double z, double w);
};

// construction
ImageFlySafety::ImageFlySafety(){
	// parameters
	last_task_waypose.translation.x = 0;
	last_task_waypose.translation.y = 0;
	last_task_waypose.translation.z = 2;

	global_pid_params.pitch.prop = relative_pid_params.pitch.prop = -0.2;
	global_pid_params.pitch.intg = relative_pid_params.pitch.intg = -0.06;
	global_pid_params.pitch.derv = relative_pid_params.pitch.derv = -0.3;
	global_pid_params.pitch.min_sat = relative_pid_params.pitch.min_sat = -1;
	global_pid_params.pitch.max_sat = relative_pid_params.pitch.max_sat = 1;
	
	global_pid_params.roll.prop = relative_pid_params.roll.prop = -0.2;
	global_pid_params.roll.intg = relative_pid_params.roll.intg = -0.06;
	global_pid_params.roll.derv = relative_pid_params.roll.derv = -0.3;
	global_pid_params.roll.min_sat = relative_pid_params.roll.min_sat = -1;
	global_pid_params.roll.max_sat = relative_pid_params.roll.max_sat = 1;
	
	global_pid_params.yaw.prop = relative_pid_params.yaw.prop = -0.5;
	global_pid_params.yaw.intg = relative_pid_params.yaw.intg = 0;
	global_pid_params.yaw.derv = relative_pid_params.yaw.derv = 0;
	global_pid_params.yaw.min_sat = relative_pid_params.yaw.min_sat = 0;
	global_pid_params.yaw.max_sat = relative_pid_params.yaw.max_sat = 0;
	
	global_pid_params.thrust.prop = relative_pid_params.thrust.prop = 0.8;
	global_pid_params.thrust.intg = relative_pid_params.thrust.intg = 0.0;
	global_pid_params.thrust.derv = relative_pid_params.thrust.derv = 0.2;
	global_pid_params.thrust.min_sat = relative_pid_params.thrust.min_sat = 0.0;
	global_pid_params.thrust.max_sat = relative_pid_params.thrust.max_sat = 0.0;
	
	get_subject_pose = false;
	current_topic_source = 0;	

	// subscribe	
	UAV_subject_pose_sub_ = nh.subscribe("subject_pose", 1, &ImageFlySafety::callbackUAVSubjectPoseMsg, this);
	UAV_task_waypose_sub_ = nh.subscribe("task_waypose", 1, &ImageFlySafety::callbackUAVTaskWayposeMsg, this);
	topic_source_sub_ = nh.subscribe("topic_source", 1, &ImageFlySafety::callbackTopicSourceMsg, this);
	UAV_relative_pid_sub_ = nh.subscribe("relative_pid_params", 1, &ImageFlySafety::callbackRelativePIDParamsMsg, this);
	
	// publish
  	UAV_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("subject_pose", 1, true);
	UAV_pid_pub_ = nh.advertise<collab_msgs::PidParams>("pid_params", 1, true);	

	// timer
	timer = nh.createTimer(ros::Duration(0.5), &ImageFlySafety::callbackTimer, this);
}

// callback functions
void ImageFlySafety::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	get_subject_pose = true;
}

void ImageFlySafety::callbackUAVTaskWayposeMsg(const collab_msgs::SubjectPose &task_waypose_msg){
	last_task_waypose = task_waypose_msg;
}

void ImageFlySafety::callbackTimer(const ros::TimerEvent& event){
	//ROS_INFO("Timer call back!!!!\n");
	
	if(get_subject_pose == false){
		ROS_INFO("I sent a fake subject_pose!!!!\n");
		GiveFakeSubjectPose(last_task_waypose.translation.x,last_task_waypose.translation.y,last_task_waypose.translation.z,last_task_waypose.rotation.z);
	}
	get_subject_pose = false;
}

void ImageFlySafety::callbackTopicSourceMsg(const topic_arbitration::topic_source &topic_source_msg){
	if(topic_source_msg.state == current_topic_source) return;
	current_topic_source = topic_source_msg.state;
	if(current_topic_source == 0) UAV_pid_pub_.publish(global_pid_params);
	else UAV_pid_pub_.publish(relative_pid_params);
}

void ImageFlySafety::callbackRelativePIDParamsMsg(const collab_msgs::PidParams &pid_params_msg){
	relative_pid_params = pid_params_msg;
}
// other functions
void ImageFlySafety::GiveFakeSubjectPose(double x, double y, double z, double w){
	//ROS_INFO("%lf %lf %lf\n",x,y,z);
	collab_msgs::SubjectPose subject_pose;
	subject_pose.translation.x = x;
	subject_pose.translation.y = y;
	subject_pose.translation.z = z;
	subject_pose.rotation.z = w;
	subject_pose.header.stamp = ros::Time::now();
	UAV_subject_pose_pub_.publish(subject_pose);
}

// main
int main(int argc, char** argv){
	ros::init(argc, argv, "ImageFlySafety");
	ROS_INFO("ImageFlySafety Started\n");
	ImageFlySafety ifs;
	ros::spin();
	return 0;
}

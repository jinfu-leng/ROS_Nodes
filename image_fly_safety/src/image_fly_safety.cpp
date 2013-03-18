#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"

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

	// subscribe	
	ros::Subscriber UAV_subject_pose_sub_;
	ros::Subscriber UAV_task_waypose_sub_;
	
	// publish
	ros::Publisher UAV_subject_pose_pub_;

	// callback functions
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
	void callbackUAVTaskWayposeMsg(const collab_msgs::SubjectPose &task_waypose_msg);
	void callbackTimer(const ros::TimerEvent& event);
	
	// other function
	void GiveFakeSubjectPose(double x, double y, double z, double w);
};

// construction
ImageFlySafety::ImageFlySafety(){
	// parameters
	last_task_waypose.translation.x = 0;
	last_task_waypose.translation.y = 0;
	last_task_waypose.translation.z = 2; // set 2 as the initial value to let UAV fly down
	get_subject_pose = false;
	
	// subscribe	
	UAV_subject_pose_sub_ = nh.subscribe("subject_pose", 1, &ImageFlySafety::callbackUAVSubjectPoseMsg, this);
	UAV_task_waypose_sub_ = nh.subscribe("task_waypose", 1, &ImageFlySafety::callbackUAVTaskWayposeMsg, this);
	
	// publish
  	UAV_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("subject_pose", 1, true);
	
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
	printf("Timer call back!!!!\n");
	
	if(get_subject_pose == false){
		GiveFakeSubjectPose(last_task_waypose.translation.x,last_task_waypose.translation.y,last_task_waypose.translation.z,last_task_waypose.rotation.z);
	}
	get_subject_pose = false;
}
// other functions
void ImageFlySafety::GiveFakeSubjectPose(double x, double y, double z, double w){
	collab_msgs::SubjectPose subject_pose;
	subject_pose.translation.x = x;
	subject_pose.translation.y = y;
	subject_pose.translation.z = z;
	subject_pose.rotation.z = w;
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

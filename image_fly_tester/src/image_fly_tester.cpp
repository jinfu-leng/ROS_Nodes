#include "ros/ros.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"

using namespace std;


class ImageFlyTester{
public:
	ImageFlyTester();
	~ImageFlyTester(){
	}
private:
	ros::NodeHandle nh;

	// subscribe	
	ros::Subscriber UAV_subject_pose_sub_;

	// publish
	ros::Publisher UAV_subject_pose_pub_;

	// callback functions
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
	
	// other function
	void FlytoPoint(double x, double y, double z, double w);
};

// construction
ImageFlyTester::ImageFlyTester(){	
	// subscribe	
	UAV_subject_pose_sub_ = nh.subscribe("subject_pose", 1, &ImageFlyTester::callbackUAVSubjectPoseMsg, this);
	
	// publish
  	UAV_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("task_waypose", 1, true);
}

// callback functions
void ImageFlyTester::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	FlytoPoint(0,0,1,0);
}

// other functions
void ImageFlyTester::FlytoPoint(double x, double y, double z, double w){
	//collab_msgs::SubjectPose subject_pose = UAV_subject_pose_;
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
	ros::init(argc, argv, "ImageFlyTester");
	ROS_INFO("ImageFlyTester Started\n");
	ImageFlyTester ift;
	ros::spin();
	return 0;
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"

using namespace std;

const double durationTime = 6;
const double movingOffset = 0.0;

class ImageFlyTester{
public:
	ImageFlyTester();
	~ImageFlyTester(){
	}
private:
	ros::NodeHandle nh;

	// publish
	ros::Publisher UAV_subject_pose_pub_;

	// callback functions
	void callbackFlyNextPoint(const ros::TimerEvent& event);
	
	// other function
	void FlytoPoint(double x, double y, double z, double w);

	// timer
	ros::Timer timer;
	vector<double> x, y;
	int current;
};

// construction
ImageFlyTester::ImageFlyTester(){	
	// publish
  	UAV_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("task_waypose", 1, true);
	// timer
	timer = nh.createTimer(ros::Duration(durationTime), &ImageFlyTester::callbackFlyNextPoint,this);
	current = 0;
	x.push_back(movingOffset);
	y.push_back(movingOffset);

	x.push_back(-1 * movingOffset);
	y.push_back(movingOffset);

	x.push_back(-1 * movingOffset);
	y.push_back(-1 * movingOffset);

	x.push_back(movingOffset);
	y.push_back(-1 * movingOffset);
}

// callback functions
void ImageFlyTester::callbackFlyNextPoint(const ros::TimerEvent& event){
	FlytoPoint(x[current],y[current],1,0);
	current = (current + 1) % x.size();
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

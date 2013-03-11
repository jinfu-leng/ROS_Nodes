#include "ros/ros.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectPose.h"
#include "ar_pose/ARMarker.h"
#include "tf/tf.h"

using namespace std;
const double PI = 3.14159265359;

class PoseBottomImage{
public:
	PoseBottomImage();
	~PoseBottomImage(){
	}
private:
	ros::NodeHandle nh;

	// subscribe	
	ros::Subscriber ar_pose_marker_sub_;

	// publish
	ros::Publisher camera_subject_pose_pub_;

	// other parameters
	//collab_msgs::SubjectPose camera_subject_pose_;	

	// callback functions
	void callbackARPoseMarkerMsg(const ar_pose::ARMarker &ar_pose_marker_msg);
};

// construction
PoseBottomImage::PoseBottomImage(){	
	// subscribe
	ar_pose_marker_sub_ = nh.subscribe("ar_pose_marker", 1, &PoseBottomImage::callbackARPoseMarkerMsg, this);
	
	// publish
  	camera_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("camera_subject_pose", 1, true);
}

// callback functions
void PoseBottomImage::callbackARPoseMarkerMsg(const ar_pose::ARMarker &ar_pose_marker_msg){
	geometry_msgs::Point  position= ar_pose_marker_msg.pose.pose.position;
	geometry_msgs::Quaternion quaternion = ar_pose_marker_msg.pose.pose.orientation;
	//printf("old position:\n %lf\n %lf\n %lf\n",position.x,position.y,position.z);
	//printf("old orientation:\n %lf\n %lf\n %lf\n %lf\n",quaternion.x,quaternion.y,quaternion.z,quaternion.w);
	
	
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(quaternion, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	collab_msgs::SubjectPose subjectPose;
	subjectPose.translation.x = -position.x;
	subjectPose.translation.y = position.y;
	subjectPose.translation.z = position.z;
	//subjectPose.rotation.x = pitch;
	//subjectPose.rotation.y = roll;
	subjectPose.rotation.z = -yaw;
	if(subjectPose.rotation.z<0) subjectPose.rotation.z = 2*PI + subjectPose.rotation.z;
	camera_subject_pose_pub_.publish(subjectPose);
	//printf("new position:\n %lf\n %lf\n %lf\n",subjectPose.translation.x,subjectPose.translation.y,subjectPose.translation.z);
	//printf("new orientation:\n %lf\n %lf\n %lf\n",subjectPose.rotation.x,subjectPose.rotation.y,subjectPose.rotation.z);
}

// other functions
/*void HoverCurrentPosition::FlytoPoint(double x, double y, double z, double w){
	//collab_msgs::SubjectPose subject_pose = UAV_subject_pose_;
	collab_msgs::SubjectPose subject_pose;
	subject_pose.translation.x = x;
	subject_pose.translation.y = y;
	subject_pose.translation.z = z;
	subject_pose.rotation.z = w;
	UAV_subject_pose_pub_.publish(subject_pose);
}*/

// main
int main(int argc, char** argv){
	ros::init(argc, argv, "PoseBottomImage");
	ROS_INFO("PoseBottomImage Started\n");
	PoseBottomImage pbi;
	ros::spin();
	return 0;
}

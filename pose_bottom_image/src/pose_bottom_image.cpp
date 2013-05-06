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
	ros::Subscriber UAV_subject_pose_sub_; // from global

	// publish
	ros::Publisher camera_subject_pose_pub_;

	// other parameters
	collab_msgs::SubjectPose UAV_subject_pose_;

	// callback functions
	void callbackARPoseMarkerMsg(const ar_pose::ARMarker &ar_pose_marker_msg);
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg); // get global subject pose
};

// construction
PoseBottomImage::PoseBottomImage(){	
	// subscribe
	ar_pose_marker_sub_ = nh.subscribe("ar_pose_marker", 1, &PoseBottomImage::callbackARPoseMarkerMsg, this);
	UAV_subject_pose_sub_ = nh.subscribe("global_subject_pose", 1, &PoseBottomImage::callbackUAVSubjectPoseMsg, this);
	
	// publish
  	camera_subject_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("camera_subject_pose", 1, true);
}

// callback functions
void PoseBottomImage::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	UAV_subject_pose_ = subject_pose_msg;	
}

void PoseBottomImage::callbackARPoseMarkerMsg(const ar_pose::ARMarker &ar_pose_marker_msg){
	collab_msgs::SubjectPose subjectPose;
	subjectPose.header.stamp = ros::Time::now();
	
	geometry_msgs::Point  position= ar_pose_marker_msg.pose.pose.position;
	geometry_msgs::Quaternion quaternion = ar_pose_marker_msg.pose.pose.orientation;
	
	tf::Quaternion q;
	double roll, pitch, yaw;
	tf::quaternionMsgToTF(quaternion, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	//subjectPose.translation.x = -position.x;
	//subjectPose.translation.y = position.y;
	subjectPose.translation.z = position.z + 0.2;
	//subjectPose.rotation.x = pitch;// need adjustement
	//subjectPose.rotation.y = roll;// need adjustment
	//subjectPose.rotation.z = -yaw;
	//if(subjectPose.rotation.z<0) subjectPose.rotation.z = 2*PI + subjectPose.rotation.z;
	
	subjectPose.translation.x = UAV_subject_pose_.translation.x;
	subjectPose.translation.y = UAV_subject_pose_.translation.y;
	//subjectPose.translation.z = UAV_subject_pose_.translation.z;
	subjectPose.rotation.x = UAV_subject_pose_.rotation.x;
	subjectPose.rotation.y = UAV_subject_pose_.rotation.y;
	subjectPose.rotation.z = UAV_subject_pose_.rotation.z;	
	camera_subject_pose_pub_.publish(subjectPose);
	//printf("new position:\n %lf\n %lf\n %lf\n",subjectPose.translation.x,subjectPose.translation.y,subjectPose.translation.z);
	//printf("new orientation:\n %lf\n %lf\n %lf\n",subjectPose.rotation.x,subjectPose.rotation.y,subjectPose.rotation.z);
}

// main
int main(int argc, char** argv){
	ros::init(argc, argv, "PoseBottomImage");
	ROS_INFO("PoseBottomImage Started\n");
	PoseBottomImage pbi;
	ros::spin();
	return 0;
}

// includes
#include <demo_UAV_controller.h>
void demo_UAV_controller::FlytoPoint(double x, double y, double z, double w){
  collab_msgs::SubjectPose subject_pose = UAV_subject_pose_;
  subject_pose.translation.x = x;
  subject_pose.translation.y = y;
  subject_pose.translation.z = z;
  subject_pose.rotation.z = w;
  UAV_pose_pub_.publish(subject_pose);
}

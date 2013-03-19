#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ballDetector/ballLocation.h"
#include "objectSearcher/hoverStatus.h"
#include "collab_msgs/SubjectCtrlState.h"
#include "collab_msgs/SubjectPose.h"
#include "math.h"

const int NO_OBJECT = -1, TRYING_HOVER = 0, TRYING_LAND = 1;
const int ON_GROUND = -1, SEARCHING = 0, FINISHED = 1;
const int CAMERA_OFFSET_X = 0, CAMERA_OFFSET_Y = 30;
const double UAV_HEIGHT = 1.2;
const double startX = 0, startY = -0.4;
const double UAV_ROTATION_X = 0;
const double SEARCH_R = 2.5, SEARCH_STEP = 0.3;
const double PI = 3.1415926;
const double deadZoneX = 0.2, deadZoneY = 0.2;
const int toleratedRange = 80;
const double adjustHoverStep = 0.05;
const double adjustLandStep = 0.2;
const double LAND_HEIGHT = 0.8; // the height that the UAV can turn off the motor and then landi if the objection is in the tolerated range


const double objectDetectionWaitingTime = 5; //if the UAV can not get the position of the ball during this period(second), then the UAV will give up
double lastObjectDetectionTime = -1;

/* Debug */
double SEARCH_STEP_X_DEBUG = 0.2;
double SEARCH_STEP_Y_DEBUG = 0.0;
const double xMin = -2.2, xMax = 2.2;
const double yMin = -2, yMax = 2;
const int SEARCH_TIMES = 44;
int searchTimes = 0;

using namespace std;


class ObjectSearcher{
public:
	ObjectSearcher();
	~ObjectSearcher(){
	}
private:
	ros::NodeHandle nh;

	// subscribe	
	ros::Subscriber UAV_subject_ctrl_state_sub_;
	ros::Subscriber UAV_subject_pose_sub_;
	ros::Subscriber objectLocation_sub_;

	// publish
	ros::Publisher UAV_state_pub_;
	ros::Publisher UAV_pose_pub_;
	ros::Publisher hoverStatus_pub_;

	// other parameters
	collab_msgs::SubjectCtrlState UAV_subject_ctrl_state_;
	collab_msgs::SubjectPose UAV_subject_pose_;
	objectSearcher::hoverStatus hoverStatus;
	ballDetector::ballLocation ballLocation;

	// search
	int searchStatus;
	double r, angle;
	double lastX, lastY;
	double lastHoverX, lastHoverY, lastHoverZ;

	// callback functions
	void callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg);
	void callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
	void callbackReceiveLocation(const ballDetector::ballLocation& location);

	// other functions
	void Hover();
	void LandOnObject();
	void Launch();
	void Land();
	void FlytoPoint(double x, double y, double z=UAV_HEIGHT, double w=UAV_ROTATION_X);
	void ChangeState(int state, double sleepTime=3);
	double ABS(double a)
	{ 
		if(a<0) return -a;
		else return a;
	}

	/* Debug */
	ros::Subscriber UAV_Height_sub_;
	ros::Subscriber SEARCH_STEP_X_sub_;
	ros::Subscriber SEARCH_STEP_Y_sub_;
};

// construction
ObjectSearcher::ObjectSearcher(){
	// subscribe	
	// the state and pose of UAV
	UAV_subject_ctrl_state_sub_ = nh.subscribe("subject_ctrl_state", 1, &ObjectSearcher::callbackUAVSubjectCtrlStateMsg, this);
	UAV_subject_pose_sub_ = nh.subscribe("UAV_pose", 1, &ObjectSearcher::callbackUAVSubjectPoseMsg, this);
	// the location of the ball
	objectLocation_sub_ = nh.subscribe("objectLocation",1,&ObjectSearcher::callbackReceiveLocation,this);

	// publish
	// the state and pose of UAV
	UAV_state_pub_ = nh.advertise<collab_msgs::SubjectCtrlState>("cmd_subject_ctrl_state", 10, true);
  	UAV_pose_pub_ = nh.advertise<collab_msgs::SubjectPose>("task_waypose", 1, true);
	// the status of the hover
	hoverStatus_pub_ = nh.advertise<objectSearcher::hoverStatus>("hoverStatus",1);

	// initialize parameters
	hoverStatus.status = NO_OBJECT;
	UAV_subject_ctrl_state_.state = 0;
	UAV_subject_pose_.translation.x = 0;
	UAV_subject_pose_.translation.y = 0;
	UAV_subject_pose_.translation.z = UAV_HEIGHT;
	UAV_subject_pose_.rotation.x =0;
	searchStatus = ON_GROUND;
	r = 0;
	angle = 0;
	lastX = startX;
	lastY = startY;
}

// callback functions
void ObjectSearcher::callbackUAVSubjectCtrlStateMsg(const collab_msgs::SubjectCtrlState &subject_ctrl_state_msg){
	UAV_subject_ctrl_state_ = subject_ctrl_state_msg;
	if(UAV_subject_ctrl_state_.state == 8 && searchStatus == ON_GROUND) searchStatus = SEARCHING;
}

void ObjectSearcher::callbackUAVSubjectPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	UAV_subject_pose_ = subject_pose_msg;
	if(ABS(UAV_subject_pose_.translation.x-startX)>SEARCH_R || ABS(UAV_subject_pose_.translation.y-startY)>SEARCH_R){
		searchStatus = FINISHED;
		Land();
	}


	if(searchStatus == ON_GROUND){
		Launch();
	}
	else if(searchStatus == SEARCHING){
		if(hoverStatus.status == NO_OBJECT){
			if(ABS(UAV_subject_pose_.translation.x-lastX)<deadZoneX&&ABS(UAV_subject_pose_.translation.y-lastY)<deadZoneY){
				if(searchTimes >= SEARCH_TIMES){
					searchStatus = FINISHED;
				}
				else{
					if(lastX <= xMin || lastX >= xMax) SEARCH_STEP_X_DEBUG = -SEARCH_STEP_X_DEBUG;
					if(lastY <= yMin || lastY >= yMax) SEARCH_STEP_Y_DEBUG = -SEARCH_STEP_Y_DEBUG;
					double x = lastX + SEARCH_STEP_X_DEBUG;
					double y = lastY + SEARCH_STEP_Y_DEBUG;
					lastX = x;
					lastY = y;
					FlytoPoint(x,y);
					searchTimes++;
				}
/*
				if(r>SEARCH_R){
					searchStatus = FINISHED;
				}
				else{

					if(2*PI -angle <= 0.01) angle = 0;
					if(angle==0) r += SEARCH_STEP;
					double x = startX + cos(angle) * r;
					double y = startY + sin(angle) * r;
					lastX = x;
					lastY = y;
					FlytoPoint(x, y);
					angle += 2*PI/12;					
				}
*/
			}
			else{
				FlytoPoint(lastX,lastY);
			}
		}
		else if(hoverStatus.status==TRYING_HOVER||hoverStatus.status==TRYING_LAND){
			double currentTime = ros::Time::now().toSec();
			ROS_INFO("current time: %lf last ball detection %lf",currentTime,lastObjectDetectionTime);
			if(currentTime-lastObjectDetectionTime>objectDetectionWaitingTime){
				hoverStatus.status=NO_OBJECT;
				ROS_INFO("No object");
			}
		}
	}
	else if(searchStatus == FINISHED){
		Land();
	}
	else{
		ROS_INFO("Search Status Error!");
	}
	hoverStatus_pub_.publish(hoverStatus);
}

void ObjectSearcher::callbackReceiveLocation(const ballDetector::ballLocation& location){
	if(location.radius<8) return;
	ROS_INFO("Received object location");
	if(searchStatus == FINISHED) return;
	lastObjectDetectionTime = ros::Time::now().toSec(); 
	if(hoverStatus.status == NO_OBJECT){
		hoverStatus.status = TRYING_HOVER;
		lastHoverX = lastX;
		lastHoverY = lastY;
		lastHoverZ = UAV_HEIGHT;
	}
	ballLocation = location;
	Hover();
}


// other functions
void ObjectSearcher::ChangeState(int state, double sleepTime) {
	collab_msgs::SubjectCtrlState subject_ctrl_state_;
	if (UAV_subject_ctrl_state_.state != state) {
		subject_ctrl_state_ = UAV_subject_ctrl_state_;
		subject_ctrl_state_.state = state;
		UAV_state_pub_.publish(subject_ctrl_state_);
		ros::Duration(sleepTime).sleep();
	}
}

void ObjectSearcher::Launch() {
	if(UAV_subject_ctrl_state_.state!=8){
		if(UAV_subject_ctrl_state_.state<1) ChangeState(1);
		if(UAV_subject_ctrl_state_.state<2) ChangeState(2,3);
		if(UAV_subject_ctrl_state_.state<5) ChangeState(5);
		if(UAV_subject_ctrl_state_.state<8) ChangeState(8,2);
		FlytoPoint(startX,startY,UAV_HEIGHT,UAV_ROTATION_X);
	}
}

void ObjectSearcher::Land() {
	if (UAV_subject_ctrl_state_.state==8) {
		//FlytoPoint(UAV_subject_pose_.translation.x,UAV_subject_pose_.translation.y,0.2);
		if(UAV_subject_ctrl_state_.state>7) ChangeState(7,1);
		if(UAV_subject_ctrl_state_.state>6) ChangeState(6,5);
		if(UAV_subject_ctrl_state_.state>4) ChangeState(4,1);
		if(UAV_subject_ctrl_state_.state>3) ChangeState(3,1);
		if(UAV_subject_ctrl_state_.state>0) ChangeState(0,1);
	}
}

void ObjectSearcher::FlytoPoint(double x, double y, double z, double w){
	collab_msgs::SubjectPose subject_pose = UAV_subject_pose_;
	subject_pose.translation.x = x;
	subject_pose.translation.y = y;
	subject_pose.translation.z = z;
	subject_pose.rotation.z = w;
	UAV_pose_pub_.publish(subject_pose);
}

void ObjectSearcher::Hover(){
	if(UAV_subject_pose_.translation.z<0.4){
		searchStatus = FINISHED;
		Land();
		return;
	}
	if(ABS(UAV_subject_pose_.translation.x-lastHoverX)>deadZoneX||ABS(UAV_subject_pose_.translation.y-lastHoverY)>deadZoneY||UAV_subject_pose_.translation.z-lastHoverZ>0.1){
		FlytoPoint(lastHoverX,lastHoverY,lastHoverZ);
		return;
	}
	bool inLandZone = true;
	double x = lastHoverX, y = lastHoverY;
	if(abs(ballLocation.x)>toleratedRange){
		if(ballLocation.x>0) x -= adjustHoverStep;
		else x +=  adjustHoverStep;
		inLandZone = false;
	}
	if(abs(ballLocation.y-CAMERA_OFFSET_Y)>toleratedRange){
		if(ballLocation.y-CAMERA_OFFSET_Y>0) y -= adjustHoverStep;
		else y +=  adjustHoverStep;
		inLandZone = false;
	}
	if(inLandZone == true){
		lastHoverZ -= adjustLandStep;
		hoverStatus.status = TRYING_LAND;	
	}
	lastHoverX = x;
	lastHoverY = y;

	if((inLandZone==true && UAV_subject_pose_.translation.z<=LAND_HEIGHT)){
		searchStatus = FINISHED;
		Land();		
	}
	else{
		FlytoPoint(lastHoverX,lastHoverY,lastHoverZ);
	}
}

// main
int main(int argc, char** argv){
	ros::init(argc, argv, "object_searcher");
	ROS_INFO("Object Searcher Started\n");
	ObjectSearcher os;
	ros::spin();
	return 0;
}

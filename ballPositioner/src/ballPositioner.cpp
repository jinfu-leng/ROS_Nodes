#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ballDetector/ballLocation.h"
#include "ballPositioner/ballPosition.h"
#include <vector>

#define BALLPOSITIONER_DEBUG


#ifdef BALLPOSITIONER_DEBUG
#include "gazebo/ModelStates.h"
#endif

using namespace std;


class BallPositioner{
public:
	BallPositioner();
	~BallPositioner(){
	}
private:
	ros::NodeHandle nh;
	
	ros::Subscriber ballLocation_sub_;
	ros::Publisher ballPosition_pub_;

	double normalFocalDist, minFocalDist, maxFocalDist, minRange, maxRange, ballRadius;
	
	void ReceiveLocation(const ballDetector::ballLocation& location);
	ballPositioner::ballPosition Location2Position(const ballDetector::ballLocation& location);

#ifdef BALLPOSITIONER_DEBUG
	ros::Subscriber modelStates_sub_;
	void ReceiveModelStates(const gazebo::ModelStates& modelState);
	string modelName;
	double xLast, yLast, zLast;
	vector<ballPositioner::ballPosition> detectedPositions, realPositions;
	void OutputPositions();
	double cameraHeight;
	ros::Time modelStates_time;
#endif
};

BallPositioner::BallPositioner(){
	nh.getParam("normalFocalDist",normalFocalDist);
	nh.getParam("minFocalDist",minFocalDist);
	nh.getParam("maxFocalDist",maxFocalDist);
	nh.getParam("minRange",minRange);
	nh.getParam("maxRange",maxRange);
	nh.getParam("ballRadius",ballRadius);
	ballLocation_sub_ = nh.subscribe("ballLocation",1,&BallPositioner::ReceiveLocation,this);
	ballPosition_pub_ = nh.advertise<ballPositioner::ballPosition>("ballPosition",1);
#ifdef BALLPOSITIONER_DEBUG
	modelStates_sub_ = nh.subscribe("gazebo/model_states",1,&BallPositioner::ReceiveModelStates,this);
	nh.getParam("modelName",modelName);
	nh.getParam("cameraHeight",cameraHeight);
#endif
}

void BallPositioner::ReceiveLocation(const ballDetector::ballLocation& location)
{
	if(location.radius<0)
	{
		ballPositioner::ballPosition position;
		position.z = -1;
		ballPosition_pub_.publish(position);
#ifdef BALLPOSITIONER_DEBUG
		if(detectedPositions.size()>0)
		{	
			OutputPositions();
			detectedPositions.clear();
			realPositions.clear();
		}
#endif
	}
	else
	{
		ballPositioner::ballPosition position = Location2Position(location);
		ballPosition_pub_.publish(position);
	}
	//ROS_INFO("%u %u %lf %lf %lf",location.imageWidth,location.imageHeight,location.x,location.y,location.radius);
	return;
}

ballPositioner::ballPosition BallPositioner::Location2Position(const ballDetector::ballLocation& location)
{
	double focalDist = normalFocalDist;	
	double dist = focalDist * ballRadius / location.radius;
	if(dist <= minRange) focalDist = minFocalDist;
	else if (dist >= maxRange) focalDist = maxFocalDist;
	else focalDist = (dist - minRange)/(maxRange - minRange) * (maxFocalDist - minFocalDist) + minFocalDist;

	ballPositioner::ballPosition position;
	//printf("focaldist ballradius location.radius\n");
	//printf("%lf %lf %lf\n", focalDist, ballRadius, location.radius);
	position.z = focalDist * ballRadius / location.radius;
	position.x = location.x * ballRadius / location.radius;
	position.y = location.y * ballRadius / location.radius;
	position.nSecond = location.nSecond;
	detectedPositions.push_back(position);
	//ROS_INFO("%lf, %lf, %lf", position.x,position.y,position.z);
#ifdef BALLPOSITIONER_DEBUG
	ballPositioner::ballPosition realPosition;	
	realPosition.x= yLast;
	realPosition.y= zLast - cameraHeight;
	realPosition.z= xLast;
	realPosition.nSecond = modelStates_time.toNSec();
	realPositions.push_back(realPosition);
#endif
	return position;
}

#ifdef BALLPOSITIONER_DEBUG
void BallPositioner::OutputPositions()
{
	ROS_INFO("New Throwing");	

	char strs[2000];	
	strs[0]='\0';
	int len = strlen(strs);
	sprintf(strs+len,"Detected Positions\n");
	len = strlen(strs);
	sprintf(strs+len,"%u\n",(unsigned int)detectedPositions.size());
	for( unsigned int i=0; i<detectedPositions.size(); i++)
	{
		len = strlen(strs);		
		sprintf(strs+len,"%lf, %lf, %lf, %llu\n", detectedPositions[i].x,detectedPositions[i].y,detectedPositions[i].z, detectedPositions[i].nSecond);
	}
	ROS_INFO("%s", strs);

	strs[0]='\0';
	len = strlen(strs);
	sprintf(strs+len,"Real Positions\n");
	len = strlen(strs);
	sprintf(strs+len,"%u\n",(unsigned int)realPositions.size());
	for( unsigned int i=0; i<realPositions.size(); i++)
	{
		len = strlen(strs);		
		sprintf(strs+len,"%lf, %lf, %lf, %llu\n", realPositions[i].x,realPositions[i].y,realPositions[i].z, realPositions[i].nSecond);
	}
	ROS_INFO("%s", strs);

}
void BallPositioner::ReceiveModelStates(const gazebo::ModelStates& modelStates)
{
	modelStates_time = ros::Time::now();
	for(unsigned int i=0;i<modelStates.name.size();i++)
	{
		if(modelStates.name[i]==modelName)
		{
			xLast = modelStates.pose[i].position.x;			
			yLast = modelStates.pose[i].position.y;	
			zLast = modelStates.pose[i].position.z;	
			break;
		}
	}	
}
#endif


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_positioner");
	ROS_INFO("Ball Positioner Started\n");
	BallPositioner bp;
	ros::spin();
	return 0;
}

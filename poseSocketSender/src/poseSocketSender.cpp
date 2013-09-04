#include "ros/ros.h"
#include "std_msgs/String.h"
#include "collab_msgs/SubjectPose.h"
#include "math.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <algorithm>
using namespace std;

// return sockfd (-1 if failed)
int SocketConnect(string hostname, int port){
	int sockfd, portno;
	struct sockaddr_in serv_addr;
	struct hostent *server;

    	portno = port;
    	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    	if (sockfd < 0){
        	fprintf(stderr,"ERROR opening socket\n");
		return -1;
	}
    	server = gethostbyname(hostname.c_str());
    	if (server == NULL) {
        	fprintf(stderr,"ERROR no such host\n");
        	return -1;
    	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
		fprintf(stderr,"ERROR connecting\n");
		return -1;
	}
	return sockfd;
}

class PoseSocketSender{
public:
	PoseSocketSender();
	~PoseSocketSender(){
	}
private:
	ros::NodeHandle nh;

	// socket
	int sockfd,port,UAVID;
	string hostName;

	// subscribe	
	ros::Subscriber UAV_pose_sub_;

	// callback
	void callbackUAVPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg);
};

// construction
PoseSocketSender::PoseSocketSender(){
	// get param
	nh.getParam("pss_UAVID",UAVID);
	nh.getParam("pss_hostName",hostName);
	nh.getParam("pss_port",port);
	printf("%d %s %d\n",UAVID,hostName.c_str(),port);

	// socket connect
	int retry=10;
	while(retry--){
		sockfd=SocketConnect(hostName,port);
		if(sockfd!=-1) break;
		ros::Duration(5).sleep();
	}
	if(sockfd!=-1)
		printf("Connected to the server\n");
	else
		exit(1);
	// subscribe	
	UAV_pose_sub_ = nh.subscribe("UAV_pose", 1, &PoseSocketSender::callbackUAVPoseMsg, this);
}

// callback functions
void PoseSocketSender::callbackUAVPoseMsg(const collab_msgs::SubjectPose &subject_pose_msg){
	swap(subject_pose_msg.translation.y,subject_pose_msg.translation.z);
	swap(subject_pose_msg.rotation.y,subject_pose_msg.rotation.z);

	char buffer[1000];
	sprintf(buffer,"[%d %lf %lf %lf %lf %lf %lf]\n",UAVID
,subject_pose_msg.translation.x,subject_pose_msg.translation.y,subject_pose_msg.translation.z
,subject_pose_msg.rotation.x,subject_pose_msg.rotation.y,subject_pose_msg.rotation.z);
	int n=write(sockfd,buffer,strlen(buffer));
	if(n<0)
		fprintf(stderr,"ERROR writing to socket\n");
}
// main
int main(int argc, char** argv){
	ros::init(argc, argv, "PoseSocketSender");
	ROS_INFO("PoseSocketSender Started\n");
	PoseSocketSender pss;
	ros::spin();
	return 0;
}

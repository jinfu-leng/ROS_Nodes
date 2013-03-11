#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <topic_arbitration/topic_source.h>

class TopicSwitcher{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber joy_subscriber_;
    ros::Publisher topic_source_publisher_;

public:
  TopicSwitcher(){
    ros::NodeHandle params("~");
    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&TopicSwitcher::JoyCallback, this, _1));
    topic_source_publisher_ = node_handle_.advertise<topic_arbitration::topic_source>("topic_source", 10);
  }

  ~TopicSwitcher()
  {
    Stop();
  }

  void JoyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    topic_arbitration::topic_source source;
    for(int i=0;i< joy->axes.size();i++){
      if(joy->buttons[i]){
        //source.state = i;
        source.state = 1;
        topic_source_publisher_.publish(source);
        return;
      }
    }
    source.state = 0;
    topic_source_publisher_.publish(source);
  }

  void Stop()
  {
    topic_arbitration::topic_source source;
    source.state=0;
    topic_source_publisher_.publish(source);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_switcher");

  TopicSwitcher ts;
  ros::spin();

  return 0;
}

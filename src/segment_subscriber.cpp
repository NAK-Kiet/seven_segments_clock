
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"



void callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
	int hour, minute, second;
	hour = msg->data.at(0);
	minute = msg->data.at(1);
	second = msg->data.at(2);
	ROS_INFO("Now it is %d:%d:%d",hour,minute,second);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "segment_subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, callback);
  ros::spin();
  return 0;
}


#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv){
	ros::init(argc,argv,"segment_publisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("chatter",1000);
	ros::Rate loop_rate(1);
	std_msgs::Int32MultiArray msg;
	int hour=0;
	int minute =0;
	int second = 0;
	int time[3];
	
	while (ros::ok()){
		pub.publish(msg);
		second = second +1;
		if (second == 60){
			minute = minute +1;
			second =0;
			if (minute == 60){
				hour = hour +1;
				minute = 0;
			
			}
		}
		
		time[0] = hour;
		time[1] = minute;
		time[2] = second;
		msg.data = {time[0], time[1], time[2]};
		ROS_INFO("%d:%d:%d",hour,minute,second);
		ros::spinOnce();
		loop_rate.sleep();
		
	}
	return 0;

}

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

def callback(msg):
	print("It is %d:%d:%d"%(msg.data[0],msg.data[1],msg.data[2]))


rospy.init_node("segment_subscriber")
sub = rospy.Subscriber("chatter",Int16MultiArray,callback)
rospy.spin()

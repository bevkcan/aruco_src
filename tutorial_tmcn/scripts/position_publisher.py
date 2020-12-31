#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print(msg.pose.pose.position)

def get_position():
    rospy.init_node('position_publisher',anonymous=True)
    rospy.Subscriber('odom',Odometry,callback)
    rospy.spin()


if __name__ == "__main__":
    get_position()
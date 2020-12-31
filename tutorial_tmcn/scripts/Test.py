#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

def main():
    rospy.init_node('Test')
    my_pub1 = rospy.Publisher('mytopic1',Odometry,queue_size=1)
    my_pub2 =rospy.Publisher('mytopic2',Odometry,queue_size=1)

    

    odom1 = Odometry()
    odom2 = Odometry()

    odom1.twist.twist.angular.x=1

    odom2.twist.twist.angular.x=2
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        my_pub1.publish(odom1)
        my_pub2.publish(odom2)
        rate.sleep()



if __name__ == "__main__":
    main()
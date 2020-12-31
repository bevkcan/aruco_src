#!/usr/bin/env python3

import rospy
import numpy
from std_msgs.msg import Float64MultiArray

def publisher():
    
    pub = rospy.Publisher('loc_rot',Float64MultiArray, queue_size = 10)
    
    rate = rospy.Rate(5)

    msg_to_publish = Float64MultiArray()

    counter = 0

    while not rospy.is_shutdown():
        a = counter % 3 + 1
        b = counter % 3 + 2
        c = counter % 3 + 3
        
        counter+=1
        msg_to_publish_data = [a, b, c]
        pub.publish(msg_to_publish)

        rospy.loginfo("["+str(a)+","+str(b)+","+str(c)+"]")

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('loc_rot')
    publisher()




 

#! /usr/bin/env python

##### This script shows how to create a new topic
import rospy
from geometry_msgs.msg import Pose

def main():
    rospy.init_node('new_topic')
    my_pub = rospy.Publisher('my_new_topic',Pose,queue_size=10)
    pose_msg=Pose()
    pose_msg.position.x = 0.0
    pose_msg.position.y = 0.0
    pose_msg.position.z = 0.0
    pose_msg.orientation.x = 0.0
    pose_msg.orientation.y = 0.0
    pose_msg.orientation.z = 0.0
    pose_msg.orientation.w = 0.0

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        my_pub.publish(pose_msg)
        rate.sleep()
        pose_msg.position.x +=1



if __name__ == "__main__":
    main()
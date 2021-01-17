#!/usr/bin/env python3

import rospy
from test_pub_sub.msg import poseerrors
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
import time
import math
import sys

class errorpublisher:
    vpx = 0
    vpy = 0
    vpz = 0

    vpqx=0
    vpqy=0
    vpqz=0
    vpqw=0

    gtpx = 0
    gtpy = 0
    gtpz = 0

    gtpqx=0
    gtpqy=0
    gtpqz=0
    gtpqw=0


    def __init__(self):
        self.errorpub = rospy.Publisher('/errors_Publish', poseerrors, queue_size = 1)
        self.visualpose = rospy.Subscriber('/visualPose', Pose, self.posecallback, queue_size = 1)
        self.truth = rospy.Subscriber('/gazebo/model_states',ModelStates,self.truthcallback,queue_size = 1)

    def posecallback(self,data):
        self.vpx = data.position.x
        self.vpy = data.position.y
        self.vpz = data.position.z

        self.vpqx = data.orientation.x
        self.vpqy = data.orientation.y
        self.vpqz = data.orientation.z
        self.vpqw = data.orientation.w

    def truthcallback(self,data):
        self.gtpx = data.pose[5].position.x
        self.gtpy = data.pose[5].position.y
        self.gtpz = data.pose[5].position.z

        self.gtpqx = data.pose[5].orientation.x
        self.gtpqy = data.pose[5].orientation.y
        self.gtpqz = data.pose[5].orientation.z
        self.gtpqw = data.pose[5].orientation.w

    def calculation(self):
        
        p = poseerrors()
        p.quaternionerror = 0
        p.translateerror = math.sqrt((self.vpx-self.gtpx)**2+(self.vpy-self.gtpy)**2+(self.vpz-self.gtpz)**2)
        p.quaternionerror = math.sqrt((self.vpqx-self.gtpqx)**2+(self.vpqy-self.gtpqy)**2+(self.vpqz-self.gtpqz)**2+(self.vpqw-self.gtpqw)**2)
        if(p.quaternionerror > 1):
            p.quaternionerror = 2-p.quaternionerror
             
        self.errorpub.publish(p)
        
        print("TranslateError: " + str(p.translateerror) + "\tQuaternionError: " + str(p.quaternionerror))








def main(args):
    
    ic = errorpublisher()
    rospy.init_node('testing', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ic.calculation()
        r.sleep()

if __name__ == '__main__':
    main(sys.argv)











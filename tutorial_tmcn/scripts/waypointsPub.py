#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tutorial_tmcn.msg import Waypoints
import numpy as np

class WaypointClass(object):

    def __init__(self):
        self.pub = rospy.Publisher('WaypointsTopic', Waypoints,queue_size=10)
        self.waypoints = Waypoints()
        self.rate = rospy.Rate(1)

    def talker(self):
        ## WAYPOINT 1 ##
        #self.waypoints.waypointsx = np.linspace(0,20,2000)
        #self.waypoints.waypointsy = np.ones(2000)*(0.5)

        ## WAYPOINT 2 ##
        self.waypoints.waypointsx = np.linspace(1,20,2000)
        self.waypoints.waypointsy = np.ones(2000)*(5)
        
        
        while not rospy.is_shutdown():
            #A = np.array([1,2,3])
            #b=2
            #C = (np.power(A,3))*(2)+(np.power(A,2))*(2)
            #print(C)
            self.pub.publish(self.waypoints)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        rospy.init_node('waypointsPub',anonymous=True)
        waypointobj = WaypointClass()
        waypointobj.talker()
    except rospy.ROSInterruptException():
        pass




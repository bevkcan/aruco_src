#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tutorial_tmcn.msg import GlobWaypoints
from matplotlib import pyplot as plt
import math
import numpy as np
from nav_msgs.msg import Odometry

class GlobalPath(object):
    
    def __init__(self):
        self.xpoints = [0.25 ,9 ,9.141 ,9.25 ,9.394 ,9.553 ,9.65 ,9.723 ,9.744 ,9.749 ,9.75 ,9.75 ,9.718 ,
                        9.638 ,9.54 ,9.429 ,9.258 ,9 ,1 ,0.9013 ,0.7908 ,0.7108 ,0.5981 ,0.4598 ,0.3428 ,0.25 ,0.25]
        self.ypoints = [0.25 ,0.25 ,0.2633 ,0.2928 ,0.3618 ,0.4935 ,0.6264 ,0.7998 ,0.906 ,0.967 ,1 ,9 ,
                        9.218 ,9.384 ,9.52 ,9.615 ,9.704 ,9.75 ,9.75 ,9.743 ,9.72 ,9.692 ,9.633 ,9.52 ,9.361 ,9 ,0.25]
        self.numOfWaypoints = 100

        self.pub = rospy.Publisher('GlobalPath',GlobWaypoints,queue_size=1) # Publishes to GlobalPath
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        self.x = 0
        self.y = 0
        self.rate = rospy.Rate(7)
        self.waypoints = GlobWaypoints()

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.rate.sleep()
    
    def main(self,xpoints,ypoints,numOfWaypoints):
        """ Creates global path using waypoints
        args:
                xpoints: X positions of the waypoints [m]
                ypoints: Y positions of the waypoints [m]
                numOfWaypoints: Number of the samples between two waypoints  
        returns:
                WaypointsX: Global path (x points) [m]
                WaypointsY: Global path (y points) [m]
        """
        WaypointsX = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))
        WaypointsY = np.zeros(((numOfWaypoints)*(len(xpoints)-1)))

        for i in range(len(xpoints)-1):
            x = np.linspace(xpoints[i],xpoints[i+1],numOfWaypoints)
            try:
                slope = ((ypoints[i+1])-(ypoints[i]))/((xpoints[i+1])-(xpoints[i]))
                coefficient = (ypoints[i])-(slope)*(xpoints[i])
                y = (slope)*x+coefficient
            except ZeroDivisionError:
                x = np.linspace(xpoints[i],xpoints[i],numOfWaypoints)
                y = np.linspace(ypoints[i],ypoints[i+1],numOfWaypoints)
            WaypointsX[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=x
            WaypointsY[((numOfWaypoints)*(i)):((numOfWaypoints)*(i+1))]=y

        return WaypointsX, WaypointsY

    def loop(self):
        self.waypoints.globwaypointsx,self.waypoints.globwaypointsy = self.main(self.xpoints,self.ypoints,self.numOfWaypoints)
        while not rospy.is_shutdown():
            self.pub.publish(self.waypoints)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('CreateGlobalPathv4',anonymous=True)
        globalPathobj = GlobalPath()
        globalPathobj.loop()
    except rospy.ROSInterruptException():
        pass

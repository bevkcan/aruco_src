#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tutorial_tmcn.msg import GlobWaypoints
from matplotlib import pyplot as plt
import math
import numpy as np

class RefWaypointClass(object):
    
    def __init__(self):
        self.xpoints = [0.25 ,9 ,9.141 ,9.25 ,9.394 ,9.553 ,9.65 ,9.723 ,9.744 ,9.749 ,9.75 ,9.75 ,9.718 ,9.638 ,9.54 ,9.429 ,9.258 ,9 ,1 ,0.9013 ,0.7908 ,0.7108 ,0.5981 ,0.4598 ,0.3428 ,0.25 ,0.25]
        self.ypoints = [0.25 ,0.25 ,0.2633 ,0.2928 ,0.3618 ,0.4935 ,0.6264 ,0.7998 ,0.906 ,0.967 ,1 ,9 ,9.218 ,9.384 ,9.52 ,9.615 ,9.704 ,9.75 ,9.75 ,9.743 ,9.72 ,9.692 ,9.633 ,9.52 ,9.361 ,9 ,0.25]
        
        self.slopes = np.zeros((1,len(self.xpoints)))
        self.coeffs = np.zeros((1,len(self.xpoints)))
        self.numOfWaypoints = 100
        self.x = np.zeros(self.numOfWaypoints)
        self.y = np.zeros(self.numOfWaypoints)
        self.WaypointsX = np.zeros(((self.numOfWaypoints)*(len(self.xpoints)-1)))
        self.WaypointsY = np.zeros(((self.numOfWaypoints)*(len(self.xpoints)-1)))

        self.rate = rospy.Rate(4)
        self.waypoints = GlobWaypoints()

        self.pub = rospy.Publisher('GlobalPath',GlobWaypoints,queue_size=10)


    def createRefWaypoint(self):
        for i in range(len(self.xpoints)-1):

            x = np.linspace(self.xpoints[i],self.xpoints[i+1],self.numOfWaypoints)
            try:
                slope = ((self.ypoints[i+1])-(self.ypoints[i]))/((self.xpoints[i+1])-(self.xpoints[i]))
                coefficient = (self.ypoints[i])-(slope)*(self.xpoints[i])
                y = (slope)*x+coefficient
            except ZeroDivisionError:
                x = np.linspace(self.xpoints[i],self.xpoints[i],self.numOfWaypoints)
                y = np.linspace(self.ypoints[i],self.ypoints[i+1],self.numOfWaypoints)
            self.WaypointsX[((self.numOfWaypoints)*(i)):((self.numOfWaypoints)*(i+1))]=x
            self.WaypointsY[((self.numOfWaypoints)*(i)):((self.numOfWaypoints)*(i+1))]=y

        self.waypoints.globwaypointsx = self.WaypointsX
        self.waypoints.globwaypointsy = self.WaypointsY 
            

        while not rospy.is_shutdown():

            self.pub.publish(self.waypoints)

            self.rate.sleep()




if __name__ == "__main__":
    try:
        rospy.init_node('CreateGlobalPathv2',anonymous=True)
        RefWaypointObj = RefWaypointClass()
        RefWaypointObj.createRefWaypoint()
    except rospy.ROSInterruptException():
        pass

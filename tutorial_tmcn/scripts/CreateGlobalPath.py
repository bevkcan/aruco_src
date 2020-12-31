#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tutorial_tmcn.msg import GlobWaypoints
from matplotlib import pyplot as plt
import math
import numpy as np

## args:
        # (xc): is the x coordinates of the waypoints except the last one [m]
        # [x0 x1 .... xn]
        # (yc):is they coordinates of the waypoints except the last one [m]
        # [y0 y1 ...yn] 
        # (thetac):is the yaw angle of the waypoints except the last [rad]
        # [theta0 theta1 ... thetan]
        # (xf): is the x coordinates of the waypoints except the first one [m]
        # [x0 x1 .... xn]
        # (yf):is they coordinates of the waypoints except the first one [m]
        # [y0 y1 ...yn] 
        # (thetaf):is the yaw angle of the waypoints except the first [rad]
        # [theta0 theta1 ... thetan]
        
## outputs:
            # RefWaypointsX : x points of the path
            # [x0 x1 ... xn]
            # RefWaypointsY : y points of the path
            # [y0 y1 ... yn]

class RefWaypointClass(object):
    
    def __init__(self):
        ###### SETS OF WAYPOİNTS #########
        #self.xc = [0,3,6]
        #self.yc = [0.25,0.25,0.25]
        #self.thetac = [0,0,0]
        #self.xf = [3,6,8]
        #self.yf = [0.25,0.25,0.25]
        #self.thetaf = [0,0,0]

        ###### SETS OF WAYPOİNTS 2 #########
        self.xc = [6.5,7.5,8.5,9,9.2318,9.402,9.578,9.744]
        self.xf = [7.5,8.5,9,9.2318,9.402,9.578,9.744,9.7485]
        self.yc = [0.25,0.25,0.25,0.25,0.2867,0.3668,0.5219,0.906]
        self.yf = [0.25,0.25,0.25,0.2867,0.3668,0.5219,0.906,0.9529]
        self.thetac= [0,0,0,0,0.3466,0.5963,0.9133,1.413]
        self.thetaf= [0,0,0,0.3466,0.5963,0.9133,1.413,1.5240]

        ###### SETS OF WAYPOİNTS 3 #########
        #self.xc=[0,3.5]
        #self.yc = [0.25,0.25]
        #self.xf = [3.5,9.75]
        #self.yf = [0.25,10]
        #self.thetac = [0,0]
        #self.thetaf= [0,math.pi/2]

        self.rate = rospy.Rate(1)
        self.numOfWaypoints = 300
        self.waypoints = GlobWaypoints()
        self.flag=1

        self.pub = rospy.Publisher('GlobalPath',GlobWaypoints,queue_size=10)


    def createRefWaypoint(self):
        ## PRELOCATION AND INITIALATION ##

        self.thetac = np.tan(self.thetac) # Turns into slope
        self.thetaf = np.tan(self.thetaf)

        self.waypoints.globwaypointsx = np.zeros((self.numOfWaypoints)*(len(self.xc)))
        self.waypoints.globwaypointsy = np.zeros((self.numOfWaypoints)*(len(self.xc)))

        ## LOOP ##

        for i in range(len(self.xc)):
            A = [[pow(self.xf[i],3),pow(self.xf[i],2),self.xf[i],1],
                 [pow(self.xc[i],3),pow(self.xc[i],2),self.xc[i],1],
                 [(3)*(pow(self.xf[i],2)),(2)*(self.xf[i]),1,0],
                 [(3)*(pow(self.xc[i],2)),(2)*(self.xc[i]),1,0]   ]
            B = [[self.yf[i]],
                 [self.yc[i]],
                 [self.thetaf[i]],
                 [self.thetac[i]]  ]

            C = np.linalg.solve(A,B)
            #print("C:",C)
            x = np.linspace(self.xc[i],self.xf[i],self.numOfWaypoints)
            y = (C[0])*(np.power(x,3))+(C[1])*(np.power(x,2))+(C[2])*(np.power(x,1))+(C[3])

            self.waypoints.globwaypointsx[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = x
            self.waypoints.globwaypointsy[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = y

        while not rospy.is_shutdown():
            #plt.plot(self.waypoints.globwaypointsx,self.waypoints.globwaypointsy)
            #plt.show(block=True)
            self.pub.publish(self.waypoints)

            self.rate.sleep()




if __name__ == "__main__":
    try:
        rospy.init_node('CreateGlobalPath',anonymous=True)
        RefWaypointObj = RefWaypointClass()
        RefWaypointObj.createRefWaypoint()
    except rospy.ROSInterruptException():
        pass

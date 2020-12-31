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
        #self.xc=[0,3.5]
        #self.yc = [0.25,0.25]
        #self.xf = [3.5,9.75]
        #self.yf = [0.25,10]
        #self.thetac = [0,0]
        #self.thetaf= [0,math.pi/2]

        ###### SETS OF WAYPOİNTS 3 #########
        self.xc = [0, 6.5,7.5,8.5,9,9.2318,9.402,9.578,9.744,9.7485]
        self.xf = [6.5, 7.5,8.5,9,9.2318,9.402,9.578,9.744,9.7485,9.75]
        self.yc = [0.25, 0.25,0.25,0.25,0.25,0.2867,0.3668,0.5219,0.906, 0.9529]
        self.yf = [0.25, 0.25,0.25,0.25,0.2867,0.3668,0.5219,0.906,0.9529,10]
        self.thetac= [0,0,0,0,0,0.3466,0.5963,0.9133,1.413, 1.5240]
        self.thetaf= [0, 0,0,0,0.3466,0.5963,0.9133,1.413,1.5240,1.57]

        ###### SETS OF WAYPOİNTS 4 #########
        #self.xc = np.array([-5,0,0.611,1.006,1.988,2.929,3.444,3.832,3.954,3.998])
        #self.yc = np.array([-4,-4,-3.953,-3.871,-3.471,-2.724,-2.034,-1.147,-0.6049,-0.1265])
        #self.thetac = np.array([0,0,0.1534,0.2543,0.5201,0.8216,1.0373,1.28,1.4190,1.5392])
        #self.xf = np.array([0,0.611,1.006,1.988,2.929,3.444,3.832,3.954,3.998,3.998])
        #self.yf= np.array([-4,-3.953,-3.871,-3.471,-2.724,-2.034,-1.147,-0.6049,-0.1265,9])
        #self.thetaf= np.array([0,0.1534,0.2543,0.5201,0.8216,1.0373,1.2800,1.4190,1.5392,1.57])

        self.tempxc=self.xc
        self.tempxf=self.xf
        self.tempyf=self.yf
        self.tempyc=self.yc


        self.rate = rospy.Rate(5)
        self.numOfWaypoints = 500
        self.waypoints = GlobWaypoints()
        self.flag=1

        self.pub = rospy.Publisher('GlobalPath',GlobWaypoints,queue_size=10)


    def createRefWaypoint(self):
        ## PRELOCATION AND INITIALATION ##

        self.thetac = np.tan(self.thetac) # Turns into slope
        self.thetaf = np.tan(self.thetaf)

        self.waypoints.globwaypointsx = np.zeros((self.numOfWaypoints)*(len(self.xc)))
        self.waypoints.globwaypointsy = np.zeros((self.numOfWaypoints)*(len(self.xc)))
        #coefficient = np.zeros((13,4))
        ## LOOP ##
        j=0
        for i in range(len(self.xc)):
            d=np.array([self.xf[-1]])
            allx=np.concatenate((self.xc,d))
            b=allx[j+1]
            c=allx[j]
            if abs(b-c)>0.0001:
                A = [[pow(self.xf[i],3),pow(self.xf[i],2),self.xf[i],1],
                 [pow(self.xc[i],3),pow(self.xc[i],2),self.xc[i],1],
                 [(3)*(pow(self.xf[i],2)),(2)*(self.xf[i]),1,0],
                 [(3)*(pow(self.xc[i],2)),(2)*(self.xc[i]),1,0]   ]
                B = [[self.yf[i]],
                     [self.yc[i]],
                    [self.thetaf[i]],
                    [self.thetac[i]]  ]

                C = np.linalg.solve(A,B)
                #D = np.transpose(C)
                #coefficient[i,:]= D
                #print("C:",C)
                #print("C Shape:",C.shape)
                #print("D",D)
                #print("D Shape", D.shape)
                #print("coefficient",coefficient)
                x = np.linspace(self.xc[i],self.xf[i],self.numOfWaypoints)
                y = (C[0])*(np.power(x,3))+(C[1])*(np.power(x,2))+(C[2])*(np.power(x,1))+(C[3])

                self.waypoints.globwaypointsx[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = x
                self.waypoints.globwaypointsy[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = y
            else:
                self.xc=self.tempyc
                self.xf=self.tempyf
                self.yc=self.tempxc

                x = np.linspace(self.xc[i],self.xf[i],self.numOfWaypoints)
                x[:]=self.yc[i]
                y = np.linspace(self.xc[i],self.xf[i],self.numOfWaypoints)
                self.waypoints.globwaypointsx[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = x
                self.waypoints.globwaypointsy[(self.numOfWaypoints)*(i):(self.numOfWaypoints)*(i+1)] = y
                self.xc=self.tempxc
                self.xf=self.tempxf
                self.yc=self.tempyc
                self.yf=self.tempyf

            j=j+1
            

        while not rospy.is_shutdown():
            #plt.plot(self.waypoints.globwaypointsx,self.waypoints.globwaypointsy)
            #plt.show()
            self.pub.publish(self.waypoints)

            self.rate.sleep()




if __name__ == "__main__":
    try:
        rospy.init_node('CreateGlobalPathv2',anonymous=True)
        RefWaypointObj = RefWaypointClass()
        RefWaypointObj.createRefWaypoint()
    except rospy.ROSInterruptException():
        pass

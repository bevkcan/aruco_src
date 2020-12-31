#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from tutorial_tmcn.msg import GlobWaypoints, AngularVelocity, CrossTrackError
import math
import numpy as np

class FindCoeffClass(object):

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0 # in radians
        self.lf = 0.053
        self.L = 0.106
        self.CurrWheelX = 0
        self.CurrWheelY = 0
        self.distance = [None]*5000
        self.index = 10
        self.slope = 0
        self.n = 0
        self.a = 0
        self.c = 0
        self.angularVel = AngularVelocity()
        self.longVel = 0
        self.thetadot = 0
        self.wheelVel = 0
        self.e = 0
        self.crossTrackSteer = 0
        self.headingErr = 0
        self.controlGain = 1.5

        self.waypoints = GlobWaypoints()
        self.crosstrackerr= CrossTrackError()

        self.rate = rospy.Rate(10)

        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=10)

        rospy.Subscriber('GlobalPath',GlobWaypoints, self.get_ref_path,queue_size=10)

        rospy.Subscriber('cmd_vel',Twist,self.get_long_vel,queue_size=10)

        self.pub = rospy.Publisher('angular_velocity',AngularVelocity)
        self.pub2=rospy.Publisher('CrossTrackError',CrossTrackError)


    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        #print("X Position: ",self.x)
        print("Theta:",self.theta)
        self.calculate_coeff()

    def get_long_vel(self,msg):
        self.longVel = msg.linear.x
        self.wheelVel = (self.longVel)/(math.cos(self.totalSteering))
        # print(self.wheelVel)
        

    def calculate_coeff(self):
        self.CurrWheelX = self.x +(self.lf)*(math.cos(self.theta))
        self.CurrWheelY = self.y +(self.lf)*(math.sin(self.theta))
        
        for i in range(len(self.distance)):
            self.distance[i] = float('inf')

        for i in range(len(self.waypoints.globwaypointsx)):
            d = math.sqrt((self.CurrWheelX-self.waypoints.globwaypointsx[i])**2+(self.CurrWheelY-self.waypoints.globwaypointsy[i])**2)
            print("d:",d)
            self.distance[i] = d
        
        print("Distanceee:",self.distance)
        print("Frst Distance:",self.distance[0])
        print("length of waypointsx",len(self.waypoints.globwaypointsx))

        #print("Self distance", self.distance)
        if len(self.distance) > 0:
            self.index = self.distance.index(min(self.distance))
        else:
            self.index=2
                
        if self.index<2:
            self.index = 2

        print("self.index:",self.index)

        if len(self.waypoints.globwaypointsx)==0:
            self.slope = 0
            self.a = 0
            self.n = 0
            self.c =0
        else:
            self.slope = ((self.waypoints.globwaypointsy[self.index])-(self.waypoints.globwaypointsy[self.index-1]))/((self.waypoints.globwaypointsx[self.index])-(self.waypoints.globwaypointsx[self.index-1]))
            self.a = -self.slope
            self.n = self.waypoints.globwaypointsy[self.index]-(self.slope)*(self.waypoints.globwaypointsx[self.index])
            self.c = -self.n
            
        #print("WheelX position: ",self.CurrWheelX)
        self.e = abs(((self.a)*(self.x)+(self.y)+(self.c))/(math.sqrt((self.a)*(self.a)+1)))
        Test = (self.a)*(self.x)+(self.y)+(self.c)
        print("e:",self.e)
        if Test > 0 :
            self.e = -self.e
        
        if self.longVel > 0.01:
            self.crossTrackSteer = math.atan(((self.controlGain)*(self.e))/(self.wheelVel))
        else:
            self.crossTrackSteer = 0
        
        self.headingErr = math.atan((-self.a)/(1))-self.theta
        self.totalSteering = self.headingErr+ self.crossTrackSteer

        if self.totalSteering > 1:
            self.totalSteering = 1

        if self.totalSteering < -1:
            self.totalSteering = -1

        self.angularVel = ((self.longVel)*(math.tan(self.totalSteering)))/(self.L)
        self.crosstrackerr.e=self.e
        self.pub.publish(self.angularVel)
        self.pub2.publish(self.crosstrackerr.e)
        print("Total Steering:",self.totalSteering)

        


    def get_ref_path(self,msg):
        self.waypoints.globwaypointsx = msg.globwaypointsx
        self.waypoints.globwaypointsy = msg.globwaypointsy
        

    def loop(self):
        rospy.spin()





if __name__ == "__main__":
    try:
        rospy.init_node('findCoeff',anonymous=True)
        findCoeffObj= FindCoeffClass()
        findCoeffObj.loop()
    except rospy.ROSInterruptException():
        pass

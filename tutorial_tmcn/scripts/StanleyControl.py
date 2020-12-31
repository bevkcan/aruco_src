#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from tutorial_tmcn.msg import GlobWaypoints, AngularVelocity, CrossTrackError, LocalWaypoints
import math
import numpy as np

class FindCoeffClass(object):

    def __init__(self):
        self.x = 0.0
        self.prevx=0
        self.prevy=0
        self.y = 0.0
        self.theta = 0.0 # in radians
        self.lf = 0.053
        self.L = 0.106
        self.CurrWheelX = 0
        self.CurrWheelY = 0
        self.distance = [None]*2600
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
        self.controlGain = 1

        self.waypoints = LocalWaypoints()
        self.crosstrackerr= CrossTrackError()

        self.rate = rospy.Rate(10)

        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=10)

        rospy.Subscriber('LocalWaypoints',LocalWaypoints, self.get_ref_path,queue_size=10)

        rospy.Subscriber('cmd_vel',Twist,self.get_long_vel,queue_size=10)

        self.pub = rospy.Publisher('angular_velocity',AngularVelocity,queue_size=10)
        self.pub2=rospy.Publisher('CrossTrackError',CrossTrackError,queue_size=10)


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

        ###################### Findcoeff of The Line ###############################
        self.CurrWheelX = self.x +(self.lf)*(math.cos(self.theta))
        self.CurrWheelY = self.y +(self.lf)*(math.sin(self.theta))
        
        for i in range(len(self.distance)):
            self.distance[i] = float('inf')

        for i in range(len(self.waypoints.localwaypointsx)):
            d = math.sqrt((self.CurrWheelX-self.waypoints.localwaypointsx[i])**2+(self.CurrWheelY-self.waypoints.localwaypointsy[i])**2)
            #print("d:",d)
            self.distance[i] = d
        
        #print("Distanceee:",self.distance)
        #print("Frst Distance:",self.distance[0])
        #print("length of waypointsx",len(self.waypoints.globwaypointsx))

        #print("Self distance", self.distance)
        if len(self.distance) > 0:
            self.index = self.distance.index(min(self.distance))
        else:
            self.index=2
                
        if self.index<2:
            self.index = 2

        print("self.index:",self.index)

        if len(self.waypoints.localwaypointsx)==0:
            self.slope = 0
            self.a = 0
            self.n = 0
            self.c =0
        else:
            try:
                self.slope = ((self.waypoints.localwaypointsy[self.index])-(self.waypoints.localwaypointsy[self.index-1]))/((self.waypoints.localwaypointsx[self.index])-(self.waypoints.localwaypointsx[self.index-1]))
            except ZeroDivisionError:
                self.slope = float("inf")
            
            self.a = -self.slope
            self.n = self.waypoints.localwaypointsy[self.index]-(self.slope)*(self.waypoints.localwaypointsx[self.index])
            self.c = -self.n
            
        ######################### CALCULATE TOTAL STEERING ##########################
        convert_Theta=math.atan(math.tan(self.theta))
        #print("self.index",self.index)
        if self.index == len(self.waypoints.localwaypointsy)-1:
            self.index = self.index-1
        delta_y=self.waypoints.localwaypointsy[self.index+1]-self.waypoints.localwaypointsy[self.index]
        delta_x=self.waypoints.localwaypointsx[self.index+1]-self.waypoints.localwaypointsx[self.index]
        try:
            constant=(delta_y)/(delta_x)
        except ZeroDivisionError:
            constant = float("inf")
        
        LineHeadingError=math.atan(constant)
        #if abs(LineHeadingError) < 0.001:
         #   LineHeadingError = 0
        print("LineHeadingError:", LineHeadingError)
        self.headingErr=LineHeadingError-convert_Theta
        if abs(self.headingErr) > (math.pi)/(2):
            self.headingErr=-LineHeadingError+convert_Theta

        if abs(self.headingErr) < 0.001:
            self.headingErr=0

        print("headingErr",self.headingErr)
        print("yaw of the rosbot:",self.theta)
        self.e = abs(((self.a)*(self.x)+(self.y)+(self.c))/(math.sqrt((self.a)*(self.a)+1)))
        if math.isnan(self.e) or math.isinf(self.e):
            self.e=self.waypoints.localwaypointsx[self.index]-self.x

        if abs(self.e) < 0.0001:
            self.e=0

        Test = (self.a)*(self.x)+(self.y)+(self.c)
        Test2=self.waypoints.localwaypointsx[self.index]-self.x
       

        Test3=self.waypoints.localwaypointsy[self.index]-self.y
        
        if math.isnan(Test) or math.isinf(Test):
            Test=Test2

        deriv_x = self.x - self.prevx
        deriv_y = self.y - self.prevy
        #print("deriv_x",deriv_x)
        #print("deriv_y",deriv_y)
        #print("Test",Test)
        #print("Test2",Test2)
        #print("Test3",Test3)
        ############ SPECIAL CASE 0<X<Pİ/2 ################################
        if 0 < LineHeadingError < (math.pi)/(2):
            if Test > 0:
                if deriv_y > 0:
                    self.e=-abs(self.e)
                else:
                    self.e=abs(self.e)
            else:
                if deriv_y > 0:
                    self.e=abs(self.e)
                else:
                    self.e=-abs(self.e)

        ############ SPECIAL CASE x==pi/2 or x==-pi/2 #####################
        if LineHeadingError == (math.pi)/(2) or LineHeadingError == -(math.pi)/(2):
            if Test2 > 0:
                if deriv_y > 0:
                    self.e=-abs(self.e)
                else:
                    self.e=abs(self.e)
            else:
                if deriv_y > 0:
                    self.e=abs(self.e)
                else:
                    self.e=-abs(self.e)

        ############ SPECIAL CASE x==0 ####################################
        if LineHeadingError == 0:
            if Test3 > 0:
                if deriv_x > 0:
                    self.e = abs(self.e)
                else:
                    self.e = -abs(self.e)
            else:
                if deriv_x > 0:
                    self.e = -abs(self.e)
                else:
                    self.e = abs(self.e)
        ############ SPECIAL CASE -pi/2<x<0 ###############################
        if -(math.pi)/(2) < LineHeadingError and LineHeadingError < 0:
            if Test > 0:
                if deriv_y > 0:
                    self.e = abs(self.e)
                else:
                    self.e = -abs(self.e)
            else:
                if deriv_y > 0:
                    self.e = -abs(self.e)
                else:
                    self.e = abs(self.e)
        print("e:",self.e)

        if self.longVel > 0.01:
            self.crossTrackSteer = math.atan(((self.controlGain)*(self.e))/(self.wheelVel))
        else:
            self.crossTrackSteer = 0
        
        #self.headingErr = math.atan((-self.a)/(1))-self.theta
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
        self.prevx = self.x
        self.prevy = self.y



        


    def get_ref_path(self,msg):
        self.waypoints.localwaypointsx = msg.localwaypointsx
        self.waypoints.localwaypointsy = msg.localwaypointsy
        print("first term of x point:",self.waypoints.localwaypointsx[0])
        print("last term of x point:",self.waypoints.localwaypointsx[-1])
        print("first term of y point:",self.waypoints.localwaypointsy[0])
        print("last term of y point:",self.waypoints.localwaypointsy[-1])
        #self.calculate_coeff()
        #plt.plot(self.waypoints.localwaypointsx,self.waypoints.localwaypointsy)
        #plt.ylim(-3,3)
        #plt.xlim(0,10)
        #plt.show()
        

    def loop(self):
        rospy.spin()





if __name__ == "__main__":
    try:
        rospy.init_node('StanleyControl',anonymous=True)
        findCoeffObj= FindCoeffClass()
        findCoeffObj.loop()
    except rospy.ROSInterruptException():
        pass

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
        self.L = 0.106
        self.distance = [None]*2600
        self.angularVel = AngularVelocity()
        self.longVel = 0
        self.thetadot = 0
        self.wheelVel = 0
        self.controlGain = 1.5
        self.totalSteering = 0
        self.waypoints = GlobWaypoints()
        self.crosstrackerr= CrossTrackError()

        self.rate = rospy.Rate(200)

        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)


        rospy.Subscriber('GlobalPath',GlobWaypoints, self.get_ref_path,queue_size=1)

        rospy.Subscriber('cmd_vel',Twist,self.get_long_vel,queue_size=1)

        self.pub = rospy.Publisher('angular_velocity',AngularVelocity,queue_size=1)
        self.pub2=rospy.Publisher('CrossTrackError',CrossTrackError,queue_size=1)


    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        print("Time in get_states:", rospy.get_time())
        self.angularVel.LineHeadingError,self.angularVel.headingErr,self.angularVel.angularVelocity = self.calculate_angular_velocity(self.waypoints)
        self.pub.publish(self.angularVel)
        
        
        #self.rate.sleep()
    def main(self):
        pass
        #data = None
        #while data is None:
         #   try:
          #      data = rospy.wait_for_message('GlobalPath',GlobWaypoints)
           # except:
            #    pass

        #while not rospy.is_shutdown():
            #self.angularVel.LineHeadingError,self.angularVel.headingErr,self.angularVel.angularVelocity = self.calculate_angular_velocity(self.waypoints)
            #self.pub.publish(self.angularVel)
            #print("Time in main:",rospy.get_time())
            #self.rate.sleep()

    def get_long_vel(self,msg):
        self.longVel = msg.linear.x
        self.wheelVel = (self.longVel)/(math.cos(self.totalSteering))
        

    def calculate_angular_velocity(self, waypoints):
        """ CALCULATE REQUIRED ANGULAR VELOCITY TO FOLLOE THE GIVEN PATH

            args:   
                    waypoints: Global path

                        waypoints.localwaypointsx - x points of local path [m]
                        waypoints.localwaypointsx - y points of local path [m]

            returns:
                    angularVel: Calculted angular velocity [rad/s]
        """
        headingErr = 0 
        crossTrackSteer = 0
        e = 0
        lf = 0.053
        index = 10
        a = 0
        n = 0
        c = 0
        slope = 0
        ###################### Findcoeff of The Line ###############################
        CurrWheelX = self.x +(lf)*(math.cos(self.theta))
        CurrWheelY = self.y +(lf)*(math.sin(self.theta))
        
        for i in range(len(self.distance)):
            self.distance[i] = float('inf')

        for i in range(len(waypoints.globwaypointsx)):
            d = math.sqrt((CurrWheelX-waypoints.globwaypointsx[i])**2+(CurrWheelY-waypoints.globwaypointsy[i])**2)
            #print("d:",d)
            self.distance[i] = d

        if len(self.distance) > 0:
            index = self.distance.index(min(self.distance))
        else:
            index=2
                
        if index<2:
            index = 2

        if len(waypoints.globwaypointsx)==0:
            slope = 0
            a = 0
            n = 0
            c =0
        else:
            try:
                slope = ((waypoints.globwaypointsy[index])-(waypoints.globwaypointsy[index-1]))/((waypoints.globwaypointsx[index])-(waypoints.globwaypointsx[index-1]))
            except ZeroDivisionError:
                slope = float("inf")
            
            a = -slope
            n = waypoints.globwaypointsy[index]-(slope)*(waypoints.globwaypointsx[index])
            c = -n
            
        ######################### CALCULATE TOTAL STEERING ##########################
        convert_Theta=math.atan(math.tan(self.theta))

        if index == len(waypoints.globwaypointsy)-1:
            index = index-1
        delta_y=waypoints.globwaypointsy[index+1]-waypoints.globwaypointsy[index]
        delta_x=waypoints.globwaypointsx[index+1]-waypoints.globwaypointsx[index]
        try:
            constant=(delta_y)/(delta_x)
        except ZeroDivisionError:
            constant = float("inf")
        
        LineHeadingError=math.atan(constant)
        deriv_x = self.x - self.prevx
        deriv_y = self.y - self.prevy

        headingErr=LineHeadingError-convert_Theta
        
        if abs(headingErr) > 2.5:
            if convert_Theta >0 and LineHeadingError <0:
                headingErr = math.pi-convert_Theta-LineHeadingError
            if convert_Theta < 0 and LineHeadingError > 0:
                headingErr = LineHeadingError -convert_Theta-math.pi
            if convert_Theta > 0 and LineHeadingError < 0 and deriv_y < 0:
                headingErr = -LineHeadingError - convert_Theta
            
        if abs(headingErr) < 0.0001:
            headingErr=0

        e = abs(((a)*(self.x)+(self.y)+(c))/(math.sqrt((a)*(a)+1)))
        if math.isnan(e) or math.isinf(e):
            e=waypoints.globwaypointsx[index]-self.x

        if abs(e) < 0.0001:
            e=0

        Test = (a)*(self.x)+(self.y)+(c)
        Test2=waypoints.globwaypointsx[index]-self.x
       
        print("Test2 in stanley:",Test2)
        print("deriv_y in stanley:",deriv_y)
        Test3=waypoints.globwaypointsy[index]-self.y
        
        if math.isnan(Test) or math.isinf(Test):
            Test=Test2

        ############ SPECIAL CASE 0<X<Pİ/2 ################################
        if 0 < LineHeadingError < (math.pi)/(2):
            if Test > 0:
                if deriv_y > 0:
                    e=-abs(e)
                else:
                    e=abs(e)
            else:
                if deriv_y > 0:
                    e=abs(e)
                else:
                    e=-abs(e)

        ############ SPECIAL CASE x==pi/2 or x==-pi/2 #####################
        if LineHeadingError == (math.pi)/(2) or LineHeadingError == -(math.pi)/(2):
            if Test2 > 0:
                if deriv_y > 0:
                    e=-abs(e)
                else:
                    e=abs(e)
            else:
                if deriv_y > 0:
                    e=abs(e)
                else:
                    e=-abs(e)

        ############ SPECIAL CASE x==0 ####################################
        if LineHeadingError == 0:
            if Test3 > 0:
                if deriv_x > 0:
                    e = abs(e)
                else:
                    e = -abs(e)
            else:
                if deriv_x > 0:
                    e = -abs(e)
                else:
                    e = abs(e)
        ############ SPECIAL CASE -pi/2<x<0 ###############################
        if -(math.pi)/(2) < LineHeadingError and LineHeadingError < 0:
            if Test > 0:
                if deriv_y > 0:
                    e = abs(e)
                else:
                    e = -abs(e)
            else:
                if deriv_y > 0:
                    e = -abs(e)
                else:
                    e = abs(e)
        #print("e:",e)

        if self.longVel > 0.01:
            crossTrackSteer = math.atan(((self.controlGain)*(e))/(self.wheelVel+1))
        else:
            crossTrackSteer = 0
        
        self.totalSteering = headingErr+ crossTrackSteer

        if self.totalSteering > 0.4:
            self.totalSteering = 0.4

        if self.totalSteering < -0.4:
            self.totalSteering = -0.4
        print("Total steering:",self.totalSteering)
        self.crosstrackerr.e=e
        self.pub2.publish(self.crosstrackerr.e)
        print("self.xin stanley:",self.x)
        self.prevx = self.x
        self.prevy = self.y
        angularVel = ((self.longVel)*(math.tan(self.totalSteering)))/(self.L)
        print("LineHeading in stanley:",LineHeadingError)
        print("headingErr in stanley:",headingErr)
        print("totalSteering in stanley:",self.totalSteering)
        print("e in stanley:",e)
        print("wheelvel in stanley",self.wheelVel)
        return LineHeadingError, headingErr, angularVel




        


    def get_ref_path(self,msg):
        self.waypoints.globwaypointsx = msg.globwaypointsx
        self.waypoints.globwaypointsy = msg.globwaypointsy

        

    def loop(self):
        #self.main()
        rospy.spin()





if __name__ == "__main__":
    try:
        rospy.init_node('StanleyControlv3',anonymous=True)
        findCoeffObj= FindCoeffClass()
        findCoeffObj.loop()
    except rospy.ROSInterruptException():
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates
from tf.transformations import euler_from_quaternion



class goalStateClass(object):

    def __init__(self):
        self.pub = rospy.Publisher('GoalIndexTopic',GoalIndex,queue_size=10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.globwaypoints = GlobWaypoints()
        self.goalIndex = GoalIndex()
        self.prevgoalIndex=0
        self.closest_index = 0
        self.lookahead = 0.6
        self.closest_len = 0
        self.arc_length = 0
        self.p1_info = 0
        self.p2_info = 0
        self.flag=1
        self.flag2 =1
        self.flag3=1
        self.longVel = 0
        self.rate =rospy.Rate(5)

        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=10)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=10)
        rospy.Subscriber('cmd_vel',Twist,self.get_long_vel,queue_size=10)
        

    def loop(self):
        
        rospy.spin()

    def  get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        


    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy
        distance = [None]*len(self.globwaypoints.globwaypointsx)
        for i in range(len(self.globwaypoints.globwaypointsx)):
            distance[i] = float('inf')
        self.calculateGoal(distance)
        
    def get_long_vel(self,msg):
        self.longVel = msg.linear.x

    def calculateGoal(self,distance):
        ## FIRST CALCULATE GOAL_INDEX ##
        ## Calculates closest length and index
        for i in range(len(self.globwaypoints.globwaypointsx)):
            distance[i] = math.sqrt(pow(self.x-self.globwaypoints.globwaypointsx[i],2)+pow(self.y-self.globwaypoints.globwaypointsy[i],2))
        
        if len(distance) > 0:
            self.closest_index = distance.index(min(distance))
        else:
            self.closest_index=2
                
        if self.closest_index<2:
            self.closest_index = 2

        self.closest_len = min(distance)

        ## Finds GOAL_INDEX Using Lookahead Distance
        self.arc_length =self.closest_len
        self.goalIndex.goalindex = self.closest_index
        if self.arc_length > self.closest_len:
            return
        if self.goalIndex.goalindex == len(self.globwaypoints.globwaypointsx):
            return
        
        while self.goalIndex.goalindex < len(self.globwaypoints.globwaypointsx)-1:
            self.arc_length = self.arc_length + math.sqrt(pow((self.globwaypoints.globwaypointsx[self.goalIndex.goalindex])-(self.globwaypoints.globwaypointsx[self.goalIndex.goalindex+1]),2)+pow((self.globwaypoints.globwaypointsy[self.goalIndex.goalindex])-(self.globwaypoints.globwaypointsy[self.goalIndex.goalindex+1]),2))
            while self.flag:
                if self.arc_length > (self.lookahead)/3:
                    self.p1_info = self.goalIndex.goalindex
                    self.flag = 0 
                else:
                    break

            while self.flag2:
                if self.arc_length > ((2)*(self.lookahead))/(3):
                    self.p2_info = self.goalIndex.goalindex
                    self.flag2 = 0
                else:
                    break
            
            if self.arc_length > self.lookahead:
                break
            ##if self.goalIndex.goalindex < self.prevgoalIndex:
              ##  self.goalIndex.goalindex =self.prevgoalIndex
            
            self.goalIndex.goalindex = self.goalIndex.goalindex + 1
            ##self.prevgoalIndex = self.goalIndex.goalindex
            #print("Arc len:",self.arc_length)
            #print("Goal Index:",self.goalIndex.goalindex)
            #print("X position rosbot:",self.x)
            #print("Goal x point",self.globwaypoints.globwaypointsx[self.goalIndex.goalindex])
        
        
        if self.longVel < 0.2:
            self.lookahead = 0.6
        else:
            self.lookahead = (self.longVel)*(3)
        print("Lookahead:",self.lookahead)
        self.pub.publish(self.goalIndex.goalindex)
        #self.rate.sleep()




if __name__ == "__main__":
    try:
        rospy.init_node('goalState',anonymous=True)
        goalStateObj = goalStateClass()
        goalStateObj.loop()
    except rospy.ROSInterruptException():
        pass

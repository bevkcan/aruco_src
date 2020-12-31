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
        self.pub = rospy.Publisher('GoalIndexTopic',GoalIndex,queue_size=1)
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=1)
        rospy.Subscriber('cmd_vel',Twist,self.get_long_vel,queue_size=1)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.globwaypoints = GlobWaypoints()
        self.goalIndex = GoalIndex()
        self.lookahead = 0.6
        self.longVel = 0
        self.rate =rospy.Rate(7)

    def  get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        
    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy
        
    def get_long_vel(self,msg):
        self.longVel = msg.linear.x
        
    def loop(self):
        while not rospy.is_shutdown():
            self.goalIndex.goalindex = self.main()
            self.pub.publish(self.goalIndex)
            self.rate.sleep()

    def closest_point(self):
        ## FIRST CALCULATE GOAL_INDEX ##
        ## Calculates closest length and index
        """Calculates the closest distance and index between the vehicle
            and the globalpath.
            Returns closest_index and closest_length

        returns:
                closest_len: The distance from the vehicle to the
                            closest point of the global path  [m]
                closest_index: The corresponding index 
        """
        distance = [None]*len(self.globwaypoints.globwaypointsx)
        for i in range(len(self.globwaypoints.globwaypointsx)):
            distance[i] = float('inf')

        
        for i in range(len(self.globwaypoints.globwaypointsx)):
            distance[i] = math.sqrt(pow(self.x-self.globwaypoints.globwaypointsx[i],2)+pow(self.y-self.globwaypoints.globwaypointsy[i],2))
        
        if len(distance) > 0:
            closest_index = distance.index(min(distance))
        else:
            closest_index=2
                
        if closest_index<2:
            closest_index = 2

        try:
            closest_len = min(distance)
        except ValueError:
            closest_len = 0

        return closest_index, closest_len

    def goal_index(self, closest_index, closest_len):
        """ Calculates the goal index in the global path using lookahead distance

        args: 
                closest_len: The distance from the vehicle to the
                            closest point of the global path  [m]
                closest_index: The corresponding index 
        returns:
                goalindex: Goal index in global path

        """
        flag = 1
        flag2 = 1
        arc_length = closest_len
        goalindex = closest_index

        if arc_length > closest_len:
            return
        if goalindex == len(self.globwaypoints.globwaypointsx):
            return
        
        while goalindex < len(self.globwaypoints.globwaypointsx)-1:
            arc_length = arc_length + math.sqrt(pow((self.globwaypoints.globwaypointsx[goalindex])-(self.globwaypoints.globwaypointsx[goalindex+1]),2)+pow((self.globwaypoints.globwaypointsy[goalindex])-(self.globwaypoints.globwaypointsy[goalindex+1]),2))
            while flag:
                if arc_length > (self.lookahead)/3:
                    flag = 0 
                else:
                    break

            while flag2:
                if arc_length > ((2)*(self.lookahead))/(3):
                    flag2 = 0
                else:
                    break
            
            if arc_length > self.lookahead:
                break
            
            goalindex = goalindex + 1

        return goalindex

    def update_lookahead(self):
        """ Updates lookahead distance"""

        if self.longVel < 0.3:
            self.lookahead = 0.6
        else:
            self.lookahead = (self.longVel)*(4)

    def main(self):
        """ Returns goalindex"""

        closest_index, closest_len = self.closest_point()
        goalindex = self.goal_index(closest_index, closest_len)
        self.update_lookahead()

        return goalindex


if __name__ == "__main__":
    try:
        rospy.init_node('goalStatev2',anonymous=True)
        goalStateObj = goalStateClass()
        goalStateObj.loop()
    except rospy.ROSInterruptException():
        pass

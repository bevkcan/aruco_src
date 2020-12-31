#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates
from tf.transformations import euler_from_quaternion

## args:
        # goal_index : Goal index on the REF PATH, is needed to create a local path
        # RefWaypointsX: Ref Path 
        # RefWaypointsY: Ref Path
        # CurrX [m]: Current x posiiton of the vehicle
        # CurrY [m]: Current y poisiton of the vehicle
        # Theta [rad]: Yaw angle
## outputs:        
        # Transformed_goal_state: [x_points y_points heading]   num_pathsx3

class TransGoalStateClass(object):

    def __init__(self):
        rospy.Subscriber('GoalIndexTopic',GoalIndex,self.get_goal_index,queue_size=1)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=1)
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        self.pub =rospy.Publisher('TransGoalStatesTopic',TransformedGoalStates,queue_size=1)
        
        self.globwaypoints = GlobWaypoints()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.num_paths = 7
        self.goal_state_set = np.zeros((self.num_paths,3))
        self.goal_state = TransformedGoalStates()
        self.path_offset = 0.6
        self.heading = 0
        self.rate = rospy.Rate(7)
        self.goalIndex = 0

    def get_goal_index(self,msg):
        
        self.goalIndex = msg.goalindex

    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw 
        

    def loop(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('GlobalPath',GlobWaypoints)
            except:
                pass
        while not rospy.is_shutdown():
            self.main()
            self.pub.publish(self.goal_state)
            self.rate.sleep()

    def main(self):
        goal_state_set = self.calc_trans_rot(self.globwaypoints, self.goalIndex, self.x, self.y, self.theta, self.num_paths, self.path_offset)
        self.trans_into_glob_frame(goal_state_set,self.theta, self.x, self.y)
        
    def calc_trans_rot(self,globwaypoints, goalIndex, x, y, theta, num_paths, path_offset):
        """TRANSLATION AND ROTATION TO HAVE GOAL_STATE_SET W.R.T VEHICLE FRAME"""
        goal_state_set = np.zeros((num_paths,3))
        goal_state_localX = globwaypoints.globwaypointsx[goalIndex]-x
        goal_state_localY = globwaypoints.globwaypointsy[goalIndex]-y
        theta = -theta
        goal_x = (math.cos(theta))*(goal_state_localX)-(math.sin(theta))*(goal_state_localY)
        goal_y = (math.cos(theta))*(goal_state_localY)+(math.sin(theta))*(goal_state_localX)
        self.heading = self.calculate_heading_angle(globwaypoints,goalIndex)
        goal_t = self.heading-self.theta

        if goal_t > math.pi :
            goal_t = goal_t - (2)*(math.pi)
        if goal_t < -math.pi :
            goal_t = goal_t + (2)*(math.pi)
        print("goal_t in calc_trans_rot",goal_t)
        for i in range(num_paths):
            offset = (i-round(num_paths/2))*(path_offset)
            x_offset = (offset)*(math.cos(goal_t+math.pi/2))
            y_offset = (offset)*(math.sin(goal_t+math.pi/2))
            
            goal_state_set[i,:] = [goal_x+x_offset, goal_y+y_offset,goal_t]
        self.goal_state.x = x
        self.goal_state.y = y
        self.goal_state.goal_state_vehicle_framex = goal_state_set[:,0]
        self.goal_state.goal_state_vehicle_framey = goal_state_set[:,1]
        self.goal_state.goal_state_vehicle_frametheta = goal_state_set[:,2]
        return goal_state_set
        
    def calculate_heading_angle(self, globwaypoints, goalIndex):
        """ CALCULTES HEADING ANGLE OF THE GOAL POINT IN GLOBAL FRAME

            """
        if goalIndex == len(globwaypoints.globwaypointsx):
            goalIndex = goalIndex-1
            delta_x = (globwaypoints.globwaypointsx[goalIndex+1])-(globwaypoints.globwaypointsx[goalIndex])
            delta_y = (globwaypoints.globwaypointsy[goalIndex+1])-(globwaypoints.globwaypointsy[goalIndex])
        else:
            delta_x = (globwaypoints.globwaypointsx[goalIndex+1])-(globwaypoints.globwaypointsx[goalIndex])
            delta_y = (globwaypoints.globwaypointsy[goalIndex+1])-(globwaypoints.globwaypointsy[goalIndex])
        
        #print("delta_y:",delta_y)
        try:
            heading = math.atan((delta_y)/(delta_x))
        except ZeroDivisionError:
            heading = (math.pi)/(2) 
        return heading



    def trans_into_glob_frame(self,goal_state_set, theta, x, y):
        """TRANSLATION AND ROTATION TO HAVE GOAL_STATE_SET W.R.T GLOBAL FRAME
            TRANSFORMS FROM VEHICLE FRAME INTO GLOBAL FRAME
        """
        ## TRANSFORMS INTO GLOBAL FRAME
        x1 = np.array(goal_state_set[:,0])
        y1 = np.array(goal_state_set[:,1])
        goal_state_set_heading = goal_state_set[:,2]
        x2 = (math.cos(theta))*(x1)-(math.sin(theta))*(y1)
        y2 = (math.cos(theta))*(y1)+(math.sin(theta))*(x1)
        x2 =x + x2
        y2 = y + y2
        self.goal_state.theta = theta
        self.heading = goal_state_set_heading+theta
        transformed_goal_state = np.dstack((x2,y2,self.heading),)
        transformed_goal_state = transformed_goal_state[0,:,:]
        self.goal_state.goal_state_global_framex = transformed_goal_state[:,0]
        self.goal_state.goal_state_global_framey = transformed_goal_state[:,1]
        self.goal_state.goal_state_global_frametheta = transformed_goal_state[:,2]
        


if __name__ == "__main__":
    try:
        rospy.init_node('TransformedGoalStatev3',anonymous=True)
        transGoalStateObj = TransGoalStateClass()
        transGoalStateObj.loop()
    except rospy.ROSInterruptException():
        pass


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import AngularVelocity,TransformedGoalStates
import numpy as np
import math
class setVelClass(object):

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.rate = rospy.Rate(10)
        self.twist = Twist()
        self.long_vel = 0.0
        self.x = 0
        self.pyval = 0

        rospy.Subscriber('angular_velocity',AngularVelocity,self.get_angular_vel,queue_size=1)
        rospy.Subscriber('TransGoalStatesTopic',TransformedGoalStates,self.get_trans_goal_states,queue_size=1)
        self.transformed_goal_state = TransformedGoalStates()

        self.derivtheta = 0
        self.prevglobaltheta = 0

    def get_trans_goal_states(self,msg):
        self.transformed_goal_state.goal_state_vehicle_framex = msg.goal_state_vehicle_framex
        self.transformed_goal_state.goal_state_vehicle_framey = msg.goal_state_vehicle_framey
        self.transformed_goal_state.goal_state_vehicle_frametheta = msg.goal_state_vehicle_frametheta
        self.transformed_goal_state.goal_state_global_framex = msg.goal_state_global_framex
        self.transformed_goal_state.goal_state_global_framey = msg.goal_state_global_framey
        self.transformed_goal_state.goal_state_global_frametheta = msg.goal_state_global_frametheta
        globalthetas = np.asarray(self.transformed_goal_state.goal_state_global_frametheta)
        globaltheta = np.arctan(np.tan(globalthetas[0]))
        print("globalframtheta:",self.transformed_goal_state.goal_state_global_frametheta[0])
        print("self.prevglobaltheta in get_trans_goal_states",self.prevglobaltheta)
        self.derivtheta = self.transformed_goal_state.goal_state_global_frametheta[0]-self.prevglobaltheta
        print("globalframtheta",self.transformed_goal_state.goal_state_global_frametheta[0])
        print("self.prevglobalthera",self.prevglobaltheta)

    def get_angular_vel(self,msg):
        self.twist.linear.x = self.long_vel
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = msg.angularVelocity
        print("self.derivtheta in get_angular_vel",self.derivtheta)
        b = math.atan(math.tan(self.prevglobaltheta))
        globalthetas = np.asarray(self.transformed_goal_state.goal_state_global_frametheta)
        globaltheta = np.arctan(np.tan(globalthetas[0]))
        constant = abs(abs(b)-abs(globaltheta))
        print("b in get_angular_vel",b)
        print("globaltheta in get_angular_vel",globaltheta)
        print("constant in get_angular_vel",constant)
        
        if  constant > 0.01:
            self.long_vel = self.long_vel-0.05
            if self.long_vel < 0.19:
                self.long_vel = 0.2
        else:
            if self.long_vel < 0.5:
                self.long_vel = self.long_vel+0.005
            else:
                self.long_vel = 0.5

        print("self.long vel in get_angular_vel",self.long_vel)
        #print("VelocityOfTheRosBot:",self.long_vel)
        
        self.prevglobaltheta = self.transformed_goal_state.goal_state_global_frametheta[0]
        self.pub.publish(self.twist)
        self.rate.sleep()
        


    def loop(self):
        rospy.spin()




if __name__ == "__main__":
    try:
        rospy.init_node('set_long_vel')
        setVelObj = setVelClass()
        setVelObj.loop()
    except rospy.ROSInterruptException:
        pass
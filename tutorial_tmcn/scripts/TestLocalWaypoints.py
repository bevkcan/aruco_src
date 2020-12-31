#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tutorial_tmcn.msg import GlobWaypoints,GoalIndex,TransformedGoalStates,LocalWaypoints
from tf.transformations import euler_from_quaternion

waypoints = LocalWaypoints()

def listener():

    rospy.init_node('TestLocalWaypoints',anonymous=True)

    rospy.Subscriber('LocalWaypoints',LocalWaypoints, get_ref_path,queue_size=5)

    
    rospy.spin()

def get_ref_path(msg):
    waypoints.localwaypointsx = msg.localwaypointsx
    waypoints.localwaypointsy = msg.localwaypointsy
    theta2 = np.linspace(0, 2*np.pi, 100)
    a = (0.1)*np.cos(theta2) + 5
    b = (0.1)*np.sin(theta2) + 0.25
    plt.plot(waypoints.localwaypointsx,waypoints.localwaypointsy)
    plt.plot(a,b)
    plt.ylim(-3,3)
    plt.xlim(0,10)
    #plt.show()




if __name__ == "__main__":
    listener()
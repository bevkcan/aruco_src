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

class PlanPath(object):

    def __init__(self):
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=10)
        rospy.Subscriber('GoalIndexTopic',GoalIndex,self.get_goal_index,queue_size=10)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=10)
        rospy.Subscriber('TransGoalStatesTopic',TransformedGoalStates,self.get_trans_goal_states,queue_size=1)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.goalIndex = GoalIndex()
        self.globwaypoints = GlobWaypoints()
        self.transformed_goal_state = TransformedGoalStates()
        self.NumberofWaypoints = 5000
        self.CenterX = 4.25
        self.CenterY = 0.25
        self.Radius = 0.1
        self.rate = rospy.Rate(3)
        self.localWaypoints = LocalWaypoints()
        self.pub = rospy.Publisher('LocalWaypoints',LocalWaypoints,queue_size=10)

    def loop(self):
        rospy.spin()

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw

    def get_goal_index(self,msg):
        self.goalIndex.goalindex = msg.goalindex

    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy

    def get_trans_goal_states(self,msg):
        self.transformed_goal_state.transformedx = msg.transformedx
        self.transformed_goal_state.transformedy = msg.transformedy
        self.transformed_goal_state.transformedtheta = msg.transformedtheta

        self.generatepath()

    def generatepath(self):
        ################# FIRST PART OPTIONAL PATH ###################3
        X = self.transformed_goal_state.transformedx # Goal state x points
        Y = self.transformed_goal_state.transformedy # Goal state y points
        theta = self.transformed_goal_state.transformedtheta # Goal state theta points
        print("theta in generatepath:",theta)
        theta = np.asarray(theta)
        print("theta in generatepath:",theta)
        for i in range(len(theta)):
            #theta[i] = math.tan(theta[i])
            theta[i] = np.tan(theta[i])

        coefficient = np.zeros((len(theta),4)) # 7x4
        if self.theta <0.01 and self.theta > -0.01:
            self.theta = 0

        Theta = math.tan(self.theta)
        print("X",X)
        print("Y",Y)
        print("Theta",Theta)
        ############### CALCULATES REQUIRED COEFFICIENT ############
        for i in range(len(theta)):
            A = [[pow(X[i],3),pow(X[i],2),X[i],1],
                 [pow(self.x,3),pow(self.x,2),self.x,1],
                 [(3)*(pow(X[i],2)),(2)*(X[i]),1,0],
                 [(3)*(pow(self.x,2)),(2)*(self.x),1,0]   ]
            B = [[Y[i]],
                 [self.y],
                [theta[i]],
                [Theta]  ]
            
            C = np.linalg.solve(A,B)
            D = np.transpose(C)
            coefficient[i,:] = D

        ############### CALCULATES WAYPOİNTS OF ALL PATH ##########3
        coefficient = np.transpose(coefficient) #4x7
        coefficient = np.reshape(coefficient,(1,4,len(theta))) # 1x4x7
        LocalWaypointsX = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsY = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        print("LocalWaypointsX Shape",LocalWaypointsX.shape)
        for i in range(len(theta)):
            m = np.linspace(self.x, self.globwaypoints.globwaypointsx[self.goalIndex.goalindex],self.NumberofWaypoints)
            m = np.reshape(m,(1,self.NumberofWaypoints))
            LocalWaypointsX[:,:,i] = m

        d1 = np.power(LocalWaypointsX,3)
        d2 = np.power(LocalWaypointsX,2)
        d3 = np.ones((1,self.NumberofWaypoints,len(theta)))
        X_vectorized = np.stack((d1,d2,LocalWaypointsX,d3),axis=0) 
        X_vectorized = X_vectorized[:,0,:,:] # 4x5000x7
        print("X_vectorısed:",X_vectorized.shape)
        for i in range(len(theta)):
            #LocalWaypointsY[:,:,i] = (coefficient[:,:,i])*(X_vectorized[:,:,i])
            LocalWaypointsY[:,:,i] = np.matmul(coefficient[:,:,i],X_vectorized[:,:,i])
        
        print("LocalWaypointsY Shape",LocalWaypointsY.shape)
        #plt.plot(LocalWaypointsX[0,:,0],LocalWaypointsY[0,:,0])
        #plt.plot(LocalWaypointsX[0,:,1],LocalWaypointsY[0,:,1])
        #plt.plot(LocalWaypointsX[0,:,2],LocalWaypointsY[0,:,2])
        #plt.plot(LocalWaypointsX[0,:,3],LocalWaypointsY[0,:,3])
        #plt.plot(LocalWaypointsX[0,:,4],LocalWaypointsY[0,:,4])
        #plt.plot(LocalWaypointsX[0,:,5],LocalWaypointsY[0,:,5])
        #plt.plot(LocalWaypointsX[0,:,6],LocalWaypointsY[0,:,6])
        #plt.ylim(-3,3)
        #plt.show()
        
        ####################### BU KISIMDA DİK GELME DURUMUNU YAPMADIN!!!!!########

        ####################### SELECT BEST PATH #############################3
        DistanceCenter = np.zeros((1,self.NumberofWaypoints,len(theta)))
        DistanceGoal = np.zeros((1,self.NumberofWaypoints,len(theta)))
        path_validity = np.ones((len(theta),1))
        Objective = [None]*len(theta)
        LocalWaypointsX2 = np.zeros((1,self.NumberofWaypoints))
        LocalWaypointsY2 = np.zeros((1,self.NumberofWaypoints))
        best_index = int(round((len(theta)-1)/(2)))
        best_index_score = 0.1
        Distancecheck = np.zeros((1,len(theta)))

        for j in range(len(theta)):
            for i in range(self.NumberofWaypoints):
                DistanceCenter[0,i,j] = math.sqrt(pow(LocalWaypointsX[0,i,j]-self.CenterX,2)+pow(LocalWaypointsY[0,i,j]-self.CenterY,2))
                DistanceGoal[0,i,j] = math.sqrt(pow(LocalWaypointsX[0,i,j]-(self.transformed_goal_state.transformedx[int(round((len(theta)-1))/(2))]),2)+pow(LocalWaypointsY[0,i,j]-(self.transformed_goal_state.transformedy[int(round((len(theta)-1))/(2))]),2))

        DistanceGoal[:,:,0] = (DistanceGoal[:,:,0])*(1.7)
        DistanceGoal[:,:,1] = (DistanceGoal[:,:,1])*(1.6)
        DistanceGoal[:,:,2] = (DistanceGoal[:,:,2])*(1.5)
        DistanceGoal[:,:,3] = (DistanceGoal[:,:,3])*(1)
        DistanceGoal[:,:,4] = (DistanceGoal[:,:,4])*(1.2)
        DistanceGoal[:,:,5] = (DistanceGoal[:,:,5])*(1.3)
        DistanceGoal[:,:,6] = (DistanceGoal[:,:,6])*(1.4)
        for j in range(len(theta)):
            Distancecheck[0,j] = np.amin(DistanceCenter[:,:,j])
            Objective[j] = np.sum(DistanceGoal[:,:,j])
            if Distancecheck[0,j] < self.Radius:
                path_validity[j,0] = 0
        
        Observe = Objective
        for j in range(len(theta)):
            I = Objective.index(min(Objective))
            Objective[I] = float("inf")
            if path_validity[I,0] == 1 :
                best_index = I
                break
        LocalWaypointsX2 = LocalWaypointsX[0,:,best_index]
        LocalWaypointsY2 = LocalWaypointsY[0,:,best_index]
        print("LocalWaypointsX2 shape:",LocalWaypointsX2.shape)
        self.localWaypoints.localwaypointsx = LocalWaypointsX2
        self.localWaypoints.localwaypointsy = LocalWaypointsY2
        self.pub.publish(self.localWaypoints)
        
        self.rate.sleep()
        #theta2 = np.linspace(0, 2*np.pi, 100)
        #a = (self.Radius)*np.cos(theta2) + self.CenterX
        #b = (self.Radius)*np.sin(theta2) + self.CenterY
        #plt.plot(LocalWaypointsX2[:],LocalWaypointsY2[:])
        #plt.plot(a,b)
        #plt.ylim(-3,3)
        #plt.xlim(0,10)
        #plt.show()

if __name__ == "__main__":
    try:
        rospy.init_node('PlanPath',anonymous=True)
        planpath = PlanPath()
        planpath.loop()
    except rospy.ROSInterruptException:
        pass
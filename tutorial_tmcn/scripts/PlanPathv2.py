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
        self.NumberofWaypoints = 2600
        self.CenterX = 5
        self.CenterY = 0.25
        self.Radius = 0.1
        self.rate = rospy.Rate(10)
        self.localWaypoints = LocalWaypoints()
        self.pub = rospy.Publisher('LocalWaypoints',LocalWaypoints,queue_size=10)

    def loop(self):
        rospy.spin()

    def get_states(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        self.theta = yaw
        #print("self.theta in planlathv2",self.theta)
        if abs(self.theta) < 0.02:
            self.theta = 0
        

    def get_goal_index(self,msg):
        self.goalIndex.goalindex = msg.goalindex

    def get_glob_path(self,msg):
        self.globwaypoints.globwaypointsx = msg.globwaypointsx
        self.globwaypoints.globwaypointsy = msg.globwaypointsy
        

    def get_trans_goal_states(self,msg):
        self.transformed_goal_state.goal_state_vehicle_framex = msg.goal_state_vehicle_framex
        self.transformed_goal_state.goal_state_vehicle_framey = msg.goal_state_vehicle_framey
        self.transformed_goal_state.goal_state_vehicle_frametheta = msg.goal_state_vehicle_frametheta
        self.transformed_goal_state.goal_state_global_framex = msg.goal_state_global_framex
        self.transformed_goal_state.goal_state_global_framey = msg.goal_state_global_framey
        self.transformed_goal_state.goal_state_global_frametheta = msg.goal_state_global_frametheta
        self.transformed_goal_state.theta = msg.theta
        self.transformed_goal_state.x = msg.x
        self.transformed_goal_state.y= msg.y
        self.generatepath()

        

    def generatepath(self):
        ################# FIRST PART OPTIONAL PATH ###################3
        distance = [None]*len(self.globwaypoints.globwaypointsx)
        X = self.transformed_goal_state.goal_state_vehicle_framex # Goal state x points
        Y = self.transformed_goal_state.goal_state_vehicle_framey # Goal state y points
        theta = self.transformed_goal_state.goal_state_vehicle_frametheta # Goal state theta points
        x = self.transformed_goal_state.x
        y = self.transformed_goal_state.y
        """print("xin generatepath",x)
        print("yin generatepath",y)
        print("self.x in generatepath",self.x)
        print("self.y in generatepath",self.y)
        print("self.theta in generatepath",self.theta)
        print("Vehicle goalx:",self.transformed_goal_state.goal_state_vehicle_framex[3])
        print("Vehicle goaly:",self.transformed_goal_state.goal_state_vehicle_framey[3])
        print("Vehicle vehicle_frametheta:",self.transformed_goal_state.goal_state_vehicle_frametheta[3])
        print("Global goalx:",self.transformed_goal_state.goal_state_global_framex[3])
        print("Global goaly:",self.transformed_goal_state.goal_state_global_framey[3])"""
        globaltheta = self.transformed_goal_state.goal_state_global_frametheta
        """print("Global goaltheta:",globaltheta[3])"""
        globaltheta = np.asarray(globaltheta)
        theta = np.asarray(theta)
        for i in range(len(theta)):
            #theta[i] = math.tan(theta[i])
            theta[i] = np.tan(theta[i])
            globaltheta[i] = np.arctan(np.tan(globaltheta[i]))
            #if abs(theta[i]) < 0.002:
             #   theta[i] = 0

        coefficient = np.zeros((len(theta),4)) # 7x4
        #if self.theta <0.01 and self.theta > -0.01:
         #   self.theta = 0
        #print("X",X)
        #print("Y",Y)
        #print("Theta",Theta)
        ############### CALCULATES REQUIRED COEFFICIENT ############
        for i in range(len(theta)):
            A = [[pow(X[i],3),pow(X[i],2),X[i],1],
                 [0,0,0,1],
                 [(3)*(pow(X[i],2)),(2)*(X[i]),1,0],
                 [0,0,1,0]   ]
            B = [[Y[i]],
                 [0],
                [theta[i]],
                [0]  ]
            """print("theta[i]",theta[i])"""
            
            C = np.linalg.solve(A,B)
            D = np.transpose(C)
            coefficient[i,:] = D

        ############### CALCULATES WAYPOİNTS OF ALL PATH ##########3
        coefficient = np.transpose(coefficient) #4x7
        coefficient = np.reshape(coefficient,(1,4,len(theta))) # 1x4x7
        LocalWaypointsX = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsX22 = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsY = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsY22 = np.zeros((1,self.NumberofWaypoints,len(theta))) # 1x5000x7
        for i in range(len(theta)):
            m = np.linspace(0, X[i],self.NumberofWaypoints)
            m = np.reshape(m,(1,self.NumberofWaypoints))
            LocalWaypointsX[:,:,i] = m

        d1 = np.power(LocalWaypointsX,3)
        d2 = np.power(LocalWaypointsX,2)
        d3 = np.ones((1,self.NumberofWaypoints,len(theta)))
        X_vectorized = np.stack((d1,d2,LocalWaypointsX,d3),axis=0) 
        X_vectorized = X_vectorized[:,0,:,:] # 4x5000x7
        for i in range(len(theta)):
            #LocalWaypointsY[:,:,i] = (coefficient[:,:,i])*(X_vectorized[:,:,i])
            LocalWaypointsY[:,:,i] = np.matmul(coefficient[:,:,i],X_vectorized[:,:,i])
        """print("vehicle local path first x",LocalWaypointsX[0,0,3])
        print("vehicle local path last x",LocalWaypointsX[0,-1,3])
        print("vehicle local path first y",LocalWaypointsY[0,0,3])
        print("vehicle local path last y",LocalWaypointsY[0,-1,3])"""
        print("Onceki self.theta",self.transformed_goal_state.theta)
        for i in range(len(theta)):
            for j in range(self.NumberofWaypoints):
                LocalWaypointsX22[:,j,i] = self.x + (LocalWaypointsX[:,j,i])*(math.cos(self.transformed_goal_state.theta))-(LocalWaypointsY[:,j,i])*(math.sin(self.transformed_goal_state.theta))
                LocalWaypointsY22[:,j,i] = self.y + (LocalWaypointsX[:,j,i])*(math.sin(self.transformed_goal_state.theta))+(LocalWaypointsY[:,j,i])*(math.cos(self.transformed_goal_state.theta))
                #LocalWaypointsX22[:,j,i] = self.x + (LocalWaypointsX[:,j,i])*(math.cos(self.theta))-(LocalWaypointsY[:,j,i])*(math.sin(self.theta))
                #LocalWaypointsY22[:,j,i] = self.y + (LocalWaypointsX[:,j,i])*(math.sin(self.theta))+(LocalWaypointsY[:,j,i])*(math.cos(self.theta))
        #print("LocalWaypointsY Shape",LocalWaypointsY.shape)
        #plt.plot(LocalWaypointsX[0,:,0],LocalWaypointsY[0,:,0])
        #plt.plot(LocalWaypointsX[0,:,1],LocalWaypointsY[0,:,1])
        #plt.plot(LocalWaypointsX[0,:,2],LocalWaypointsY[0,:,2])
        #plt.plot(LocalWaypointsX[0,:,3],LocalWaypointsY[0,:,3])
        #plt.plot(LocalWaypointsX[0,:,4],LocalWaypointsY[0,:,4])
        #plt.plot(LocalWaypointsX[0,:,5],LocalWaypointsY[0,:,5])
        #plt.plot(LocalWaypointsX[0,:,6],LocalWaypointsY[0,:,6])
        #plt.ylim(-3,3)
        #plt.show()
        for i in range(len(self.globwaypoints.globwaypointsx)):
            distance[i] = math.sqrt(pow(self.x-self.globwaypoints.globwaypointsx[i],2)+pow(self.y-self.globwaypoints.globwaypointsy[i],2))
        
        if len(distance) > 0:
            closest_index = distance.index(min(distance))
        else:
            closest_index=2
                
        if closest_index<2:
            closest_index = 2
        ####################### BU KISIMDA DİK GELME DURUMU!!!!!########
        for i in range(len(theta)):
            if abs(abs((globaltheta[i]))-((math.pi)/(2))) <0.0001:
                LocalWaypointsX22[:,:,i] = (np.ones((1,self.NumberofWaypoints)))*(self.globwaypoints.globwaypointsx[self.goalIndex.goalindex])
                LocalWaypointsY22[:,:,i] = np.linspace(self.globwaypoints.globwaypointsy[closest_index],self.globwaypoints.globwaypointsy[self.goalIndex.goalindex],self.NumberofWaypoints)
                print("Dik gelme DURUMU")

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
                DistanceCenter[0,i,j] = math.sqrt(pow(LocalWaypointsX22[0,i,j]-self.CenterX,2)+pow(LocalWaypointsY22[0,i,j]-self.CenterY,2))
                DistanceGoal[0,i,j] = math.sqrt(pow(LocalWaypointsX22[0,i,j]-(self.transformed_goal_state.goal_state_vehicle_framex[int(round((len(theta)-1))/(2))]),2)+pow(LocalWaypointsY22[0,i,j]-(self.transformed_goal_state.goal_state_vehicle_framey[int(round((len(theta)-1))/(2))]),2))

        DistanceGoal[:,:,0] = (DistanceGoal[:,:,0])*(1.7)
        DistanceGoal[:,:,1] = (DistanceGoal[:,:,1])*(1.6)
        DistanceGoal[:,:,2] = (DistanceGoal[:,:,2])*(1.5)
        DistanceGoal[:,:,3] = (DistanceGoal[:,:,3])*(0.5)
        DistanceGoal[:,:,4] = (DistanceGoal[:,:,4])*(1.2)
        DistanceGoal[:,:,5] = (DistanceGoal[:,:,5])*(1.3)
        DistanceGoal[:,:,6] = (DistanceGoal[:,:,6])*(1.4)
        for j in range(len(theta)):
            Distancecheck[0,j] = np.amin(DistanceCenter[:,:,j])
            Objective[j] = np.sum(DistanceGoal[:,:,j])
            """print("GoalDistance:",Objective[j])"""
            if Distancecheck[0,j] < self.Radius:
                path_validity[j,0] = 0
        
        Observe = Objective
        for j in range(len(theta)):
            I = Objective.index(min(Objective))
            Objective[I] = float("inf")
            if path_validity[I,0] == 1 :
                best_index = I
                break
        LocalWaypointsX2 = LocalWaypointsX22[0,:,best_index]
        LocalWaypointsY2 = LocalWaypointsY22[0,:,best_index]
        #print("LocalWaypointsX2 shape:",LocalWaypointsX2.shape)
        
        self.localWaypoints.localwaypointsx = LocalWaypointsX2
        self.localWaypoints.localwaypointsy = LocalWaypointsY2
        self.pub.publish(self.localWaypoints)
        
        #self.rate.sleep()

        

if __name__ == "__main__":
    try:
        rospy.init_node('PlanPathv2',anonymous=True)
        planpath = PlanPath()
        planpath.loop()
    except rospy.ROSInterruptException:
        pass
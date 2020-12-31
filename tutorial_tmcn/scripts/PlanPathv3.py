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
        rospy.Subscriber('odom',Odometry,self.get_states,queue_size=1)
        rospy.Subscriber('GoalIndexTopic',GoalIndex,self.get_goal_index,queue_size=1)
        rospy.Subscriber('GlobalPath',GlobWaypoints,self.get_glob_path,queue_size=1)
        rospy.Subscriber('TransGoalStatesTopic',TransformedGoalStates,self.get_trans_goal_states,queue_size=1)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.goalIndex = GoalIndex()
        self.globwaypoints = GlobWaypoints()
        self.transformed_goal_state = TransformedGoalStates()
        self.NumberofWaypoints = 2600
        self.CenterX = 4
        self.CenterY = 0.25
        self.Radius = 0.35
        self.rate = rospy.Rate(7)
        self.localWaypoints = LocalWaypoints()
        self.pub = rospy.Publisher('LocalWaypoints',LocalWaypoints,queue_size=1)

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
        self.main()
        self.pub.publish(self.localWaypoints)

    def loop(self):
        rospy.spin()
        pass
        #while not rospy.is_shutdown():
         #   self.main()
          #  self.pub.publish(self.localWaypoints)
           # self.rate.sleep()


    def main(self):
        coefficient = self.calculate_coefficient(self.globwaypoints,self.transformed_goal_state)
        LocalWaypointsX22, LocalWaypointsY22 = self.local_path_vehicle(coefficient, self.transformed_goal_state, self.NumberofWaypoints)
        LocalWaypointsX2, LocalWaypointsY2 = self.select_best_path(LocalWaypointsX22, LocalWaypointsY22, self.globwaypoints, self.transformed_goal_state, self.NumberofWaypoints, self.goalIndex)
        self.localWaypoints.localwaypointsx = LocalWaypointsX2
        self.localWaypoints.localwaypointsy = LocalWaypointsY2



    def calculate_coefficient(self, globwaypoints, transformed_goal_state):
        """ CALCULATE REQUIRED COEFFICIENT FOR LOCAL PATH

            args:
                    globwaypoints: Global path

                        globwaypoint.globwaypointsx - x points of global path [m]
                        globwaypoint.globwaypointsy - y points of global path [m]

                    transformed_goal_state: Transformed goal states

                        transformed_goal_state.goal_state_vehicle_framex: Goal states w.r.t vehicle frame (x-points) [m]
                        transformed_goal_state.goal_state_vehicle_framey: Goal states w.r.t vehicle frame (y-points) [m]
                        transformed_goal_state.goal_state_vehicle_frametheta: Goal states w.r.t vehicle frame (theta) [rad]
                        transformed_goal_state.goal_state_global_framex: Goal states w.r.t global frame (x-points) [m]
                        transformed_goal_state.goal_state_global_framey: Goal states w.r.t global frame (y-points) [m]
                        transformed_goal_state.goal_state_global_frametheta: Goal states w.r.t global frame (theta) [rad]
                        transformed_goal_state.theta: It is not that important
            output:
                    coefficient: Required coefficient for local path. Uses cubic spline
        
        """
        X = transformed_goal_state.goal_state_vehicle_framex # Goal state x points
        Y = transformed_goal_state.goal_state_vehicle_framey # Goal state y points
        theta = transformed_goal_state.goal_state_vehicle_frametheta # Goal state theta points
        globaltheta = transformed_goal_state.goal_state_global_frametheta
        globaltheta = np.asarray(globaltheta)
        theta = np.asarray(theta)
        for i in range(len(theta)):
            theta[i] = np.tan(theta[i])
            globaltheta[i] = np.arctan(np.tan(globaltheta[i]))

        coefficient = np.zeros((len(theta),4)) # 7x4
        for i in range(len(theta)):
            A = [[pow(X[i],3),pow(X[i],2),X[i],1],
                 [0,0,0,1],
                 [(3)*(pow(X[i],2)),(2)*(X[i]),1,0],
                 [0,0,1,0]   ]
            B = [[Y[i]],
                 [0],
                [theta[i]],
                [0]  ]
            C = np.linalg.solve(A,B)
            D = np.transpose(C)
            coefficient[i,:] = D
        return coefficient

    def local_path_vehicle(self, coefficient, transformed_goal_state, NumberofWaypoints):
        """  CALCULATES LOCAL PATHS IN VEHICLE FRAME  
            args:   
                    coefficient: Required coefficient for local path. Uses cubic spline

                    transformed_goal_state: Transformed goal states

                        transformed_goal_state.goal_state_vehicle_framex: Goal states w.r.t vehicle frame (x-points) [m]
                        transformed_goal_state.goal_state_vehicle_framey: Goal states w.r.t vehicle frame (y-points) [m]
                        transformed_goal_state.goal_state_vehicle_frametheta: Goal states w.r.t vehicle frame (theta) [rad]
                        transformed_goal_state.goal_state_global_framex: Goal states w.r.t global frame (x-points) [m]
                        transformed_goal_state.goal_state_global_framey: Goal states w.r.t global frame (y-points) [m]
                        transformed_goal_state.goal_state_global_frametheta: Goal states w.r.t global frame (theta) [rad]
                        transformed_goal_state.theta: It is not that important
                    
                    NumberofWaypoints: Number of samples for the local paths

            outputs: 
                    LocalWaypointsX22: X points of the local optional paths in global frame [m]
                    LocalWaypointsY22: Y points of the local optional paths in global frame [m]

        
        """
        X = transformed_goal_state.goal_state_vehicle_framex # Goal state x points
        theta = transformed_goal_state.goal_state_vehicle_frametheta # Goal state theta points
        coefficient = np.transpose(coefficient) #4x7
        coefficient = np.reshape(coefficient,(1,4,len(theta))) # 1x4x7
        LocalWaypointsX = np.zeros((1,NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsX22 = np.zeros((1,NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsY = np.zeros((1,NumberofWaypoints,len(theta))) # 1x5000x7
        LocalWaypointsY22 = np.zeros((1,NumberofWaypoints,len(theta))) # 1x5000x7
        for i in range(len(theta)):
            m = np.linspace(0, X[i],NumberofWaypoints)
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
        for i in range(len(theta)):
            for j in range(self.NumberofWaypoints):
                LocalWaypointsX22[:,j,i] = self.x + (LocalWaypointsX[:,j,i])*(math.cos(transformed_goal_state.theta))-(LocalWaypointsY[:,j,i])*(math.sin(transformed_goal_state.theta))
                LocalWaypointsY22[:,j,i] = self.y + (LocalWaypointsX[:,j,i])*(math.sin(transformed_goal_state.theta))+(LocalWaypointsY[:,j,i])*(math.cos(transformed_goal_state.theta))

        return LocalWaypointsX22, LocalWaypointsY22

    def select_best_path(self,LocalWaypointsX22, LocalWaypointsY22, globwaypoints, transformed_goal_state, NumberofWaypoints, goalIndex):
        """ SELECT THE BEST FROM OPTIONAL PATHS
            args:
                    LocalWaypointsX22: X points of the local optional paths in global frame [m]
                    LocalWaypointsY22: Y points of the local optional paths in global frame [m]
                    globwaypoints: Global path

                        globwaypoint.globwaypointsx - x points of global path [m]
                        globwaypoint.globwaypointsy - y points of global path [m]
                    transformed_goal_state: Transformed goal states

                        transformed_goal_state.goal_state_vehicle_framex: Goal states w.r.t vehicle frame (x-points) [m]
                        transformed_goal_state.goal_state_vehicle_framey: Goal states w.r.t vehicle frame (y-points) [m]
                        transformed_goal_state.goal_state_vehicle_frametheta: Goal states w.r.t vehicle frame (theta) [rad]
                        transformed_goal_state.goal_state_global_framex: Goal states w.r.t global frame (x-points) [m]
                        transformed_goal_state.goal_state_global_framey: Goal states w.r.t global frame (y-points) [m]
                        transformed_goal_state.goal_state_global_frametheta: Goal states w.r.t global frame (theta) [rad]
                        transformed_goal_state.theta: It is not that important
                    NumberofWaypoints: Number of samples for the local paths
                    goalIndex: Goal index for the global path
                        goalIndex.goalindex

            outputs:
                    LocalWaypointsX2: The best path from optional path in global frame (x-points)
                    LocalWaypointsY2: The best path from optional path in global frame (y-points)
        """
        distance = [None]*len(globwaypoints.globwaypointsx)
        theta = transformed_goal_state.goal_state_vehicle_frametheta # Goal state theta points
        globaltheta = transformed_goal_state.goal_state_global_frametheta
        globaltheta = np.asarray(globaltheta)
        theta = np.asarray(theta)
        for i in range(len(theta)):
            theta[i] = np.tan(theta[i])
            globaltheta[i] = np.arctan(np.tan(globaltheta[i]))
            
        for i in range(len(globwaypoints.globwaypointsx)):
            distance[i] = math.sqrt(pow(self.x-globwaypoints.globwaypointsx[i],2)+pow(self.y-globwaypoints.globwaypointsy[i],2))
        
        if len(distance) > 0:
            closest_index = distance.index(min(distance))
        else:
            closest_index=2
                
        if closest_index<2:
            closest_index = 2
        ####################### BU KISIMDA DİK GELME DURUMU!!!!!########
        print("globaltheta in select_best_path",abs(abs((globaltheta[3]))-((math.pi)/(2))))
        for i in range(len(theta)):
            if abs(abs((globaltheta[i]))-((math.pi)/(2))) <0.001:
                LocalWaypointsX22[:,:,i] = (np.ones((1,NumberofWaypoints)))*(globwaypoints.globwaypointsx[goalIndex.goalindex])
                LocalWaypointsY22[:,:,i] = np.linspace(globwaypoints.globwaypointsy[closest_index],globwaypoints.globwaypointsy[goalIndex.goalindex],NumberofWaypoints)
                print("Dik gelme DURUMU")

        ####################### SELECT BEST PATH #############################3
        DistanceCenter = np.zeros((1,NumberofWaypoints,len(theta)))
        DistanceGoal = np.zeros((1,NumberofWaypoints,len(theta)))
        path_validity = np.ones((len(theta),1))
        Objective = [None]*len(theta)
        LocalWaypointsX2 = np.zeros((1,NumberofWaypoints))
        LocalWaypointsY2 = np.zeros((1,NumberofWaypoints))
        best_index = int(round((len(theta)-1)/(2)))
        Distancecheck = np.zeros((1,len(theta)))

        for j in range(len(theta)):
            for i in range(self.NumberofWaypoints):
                DistanceCenter[0,i,j] = math.sqrt(pow(LocalWaypointsX22[0,i,j]-self.CenterX,2)+pow(LocalWaypointsY22[0,i,j]-self.CenterY,2))
                DistanceGoal[0,i,j] = math.sqrt(pow(LocalWaypointsX22[0,i,j]-(transformed_goal_state.goal_state_vehicle_framex[int(round((len(theta)-1))/(2))]),2)+pow(LocalWaypointsY22[0,i,j]-(transformed_goal_state.goal_state_vehicle_framey[int(round((len(theta)-1))/(2))]),2))

        DistanceGoal[:,:,0] = (DistanceGoal[:,:,0])*(4)
        DistanceGoal[:,:,1] = (DistanceGoal[:,:,1])*(3)
        DistanceGoal[:,:,2] = (DistanceGoal[:,:,2])*(2)
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
        
        for j in range(len(theta)):
            I = Objective.index(min(Objective))
            Objective[I] = float("inf")
            if path_validity[I,0] == 1 :
                best_index = I
                break
        LocalWaypointsX2 = LocalWaypointsX22[0,:,best_index]
        LocalWaypointsY2 = LocalWaypointsY22[0,:,best_index]

        return LocalWaypointsX2, LocalWaypointsY2


        

if __name__ == "__main__":
    try:
        rospy.init_node('PlanPathv3',anonymous=True)
        planpath = PlanPath()
        planpath.loop()
    except rospy.ROSInterruptException:
        pass
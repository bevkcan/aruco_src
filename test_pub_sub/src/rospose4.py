#!/usr/bin/env python3
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt

import sys,time

from scipy.ndimage import filters

import roslib
import rospy

from sensor_msgs.msg import CompressedImage


VERBOSE = True

class arucoPubSub:

    def __init__(self):

        self.image_pub = rospy.Publisher("/processed_image/image_raw/compressed", CompressedImage, queue_size = 1)

        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback, queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/rgb/image_raw/compressed")

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        now = rospy.Time.now()

        color = (255, 0, 255)
        color2 = (0,0,255)
        color3 = (200,150,255)
        font = cv2.FONT_HERSHEY_SIMPLEX

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters =  cv2.aruco.DetectorParameters_create()
        msize = 0.15
        axlen = 0.10
        mtx =  np.array([[1.41621911e+03, 0.00000000e+00, 8.18188905e+02],
                [0.00000000e+00, 1.41621911e+03, 5.87359422e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        dist = np.array([[ 0.00126111],
                        [ 0.13755179],
                        [ 0.00276459],
                        [-0.0316146 ],
                        [-0.26607145],
                        [ 0.01957589],
                        [-0.10396296],
                        [ 0.05794116],
                        [ 0.        ],
                        [ 0.        ],
                        [ 0.        ],
                        [ 0.        ],
                        [ 0.        ],
                        [ 0.        ]])


        frame = image_np
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)
        if len(markerCorners)> 0:
            rvecs,tvecs, trash = aruco.estimatePoseSingleMarkers(markerCorners, msize, mtx, dist)
            imaxes = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)
            j = 0
            for i in range(len(tvecs)):
                imaxes = aruco.drawAxis(imaxes, mtx, dist, rvecs[i], tvecs[i], axlen)
                cv2.putText(imaxes,'ID', (250,660), font, 0.5, color3,2)
                cv2.putText(imaxes,'    RELATIVE POSITION VECTOR', (350,660), font, 0.5, color3,2)
                cv2.putText(imaxes,'    ROTATION VECTOR', (750,660), font, 0.5, color3,2)
                cv2.putText(imaxes,'ID:'+str(markerIds[i]), (250,700+j), font, 0.5, color,2)
                cv2.putText(imaxes,'  T:'+str(tvecs[i]), (350,700+j), font, 0.5, color,2)
                cv2.putText(imaxes,' R:'+str(rvecs[i]), (750,700+j), font, 0.5, color,2)
                cv2.putText(imaxes , "iterations: " + str(now),(50,50),font,1,(255,0,255),2)
                    #print(now)
            
                #cv2.imshow('cv_cameraImage', image_np)
                #cv2.waitKey(2)
                msg = CompressedImage()
                #msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg',imaxes)[1]).tostring()
                self.image_pub.publish(msg)
                j+=40





        else:
            cv2.putText(frame_markers,'ID', (250,660), font, 0.5, color3,2)
            cv2.putText(frame_markers,'    RELATIVE POSITION VECTOR', (350,660), font, 0.5, color3,2)
            cv2.putText(frame_markers,'    ROTATION VECTOR', (750,660), font, 0.5, color3,2)
            cv2.putText(frame_markers,'DISCON', (250,760), font, 4, color2,3)


            cv2.putText(frame_markers , "iterations: " + str(now),(50,50),font,1,(255,0,255),2)
            #print(now)
            
            #cv2.imshow('cv_cameraImage', image_np)
            #cv2.waitKey(2)
            msg = CompressedImage()
            #msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg',frame_markers)[1]).tostring()
            self.image_pub.publish(msg)







        

 

def main(args):
    
    rospy.init_node('visualGlobalEstimation', anonymous = False)
    ic = arucoPubSub()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down image processing module")
    cv2.destroyAllWindows()    

if __name__ == '__main__':
    main(sys.argv)



# cap = cv2.VideoCapture('testvid.avi')

# out = cv2.VideoWriter('axes.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 15, (1920,1080))
# dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
# parameters =  cv2.aruco.DetectorParameters_create()
# msize = 0.15
# axlen = 0.10
# mtx =  np.array([[1.41621911e+03, 0.00000000e+00, 8.18188905e+02],
#         [0.00000000e+00, 1.41621911e+03, 5.87359422e+02],
#         [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# dist = np.array([[ 0.00126111],
#                 [ 0.13755179],
#                 [ 0.00276459],
#                 [-0.0316146 ],
#                 [-0.26607145],
#                 [ 0.01957589],
#                 [-0.10396296],
#                 [ 0.05794116],
#                 [ 0.        ],
#                 [ 0.        ],
#                 [ 0.        ],
#                 [ 0.        ],
#                 [ 0.        ],
#                 [ 0.        ]])



# color = (255, 0, 255)
# color2 = (0,0,255)
# color3 = (200,150,255)
# font = cv2.FONT_HERSHEY_SIMPLEX

# while(cap.isOpened()):
#     start = time.time()
#     ret, frame = cap.read()

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
#     frame_markers = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)
#     if len(markerCorners)> 0:
#         rvecs,tvecs, trash = aruco.estimatePoseSingleMarkers(markerCorners, msize, mtx, dist)
#         imaxes = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)
#         j = 0
#         for i in range(len(tvecs)):
#             imaxes = aruco.drawAxis(imaxes, mtx, dist, rvecs[i], tvecs[i], axlen)
#             cv2.putText(imaxes,'ID', (250,660), font, 0.5, color3,2)
#             cv2.putText(imaxes,'    RELATIVE POSITION VECTOR', (350,660), font, 0.5, color3,2)
#             cv2.putText(imaxes,'    ROTATION VECTOR', (750,660), font, 0.5, color3,2)
#             cv2.putText(imaxes,'ID:'+str(markerIds[i]), (250,700+j), font, 0.5, color,2)
#             cv2.putText(imaxes,'  T:'+str(tvecs[i]), (350,700+j), font, 0.5, color,2)
#             cv2.putText(imaxes,' R:'+str(rvecs[i]), (750,700+j), font, 0.5, color,2)
#             end = time.time()
#             elapsed = int((end - start)*1000000)/1000
#             cv2.putText(imaxes,str(elapsed)+'ms', (100,700), font, 0.5, color3,2)
#             j+=40


#         cv2.imshow('frame',imaxes)
#         out.write(imaxes)
#     else:
#         cv2.putText(frame_markers,'ID', (250,660), font, 0.5, color3,2)
#         cv2.putText(frame_markers,'    RELATIVE POSITION VECTOR', (350,660), font, 0.5, color3,2)
#         cv2.putText(frame_markers,'    ROTATION VECTOR', (750,660), font, 0.5, color3,2)
#         cv2.putText(frame_markers,'DISCON', (250,760), font, 4, color2,3)
#         end = time.time()
#         elapsed = int((end - start)*1000000)/1000
#         cv2.putText(frame_markers,str(elapsed)+'ms', (100,700), font, 0.5, color3,2)

#         cv2.imshow('frame',frame_markers)
#         out.write(frame_markers)

#     end=time.time()






#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

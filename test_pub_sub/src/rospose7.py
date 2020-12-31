#!/usr/bin/env python3
import cv2
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import math


import sys,time

from scipy.ndimage import filters

import roslib
import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose

wdir = os.getcwd()
wdir=wdir+"/aruco_recognition/src/test_pub_sub/src"
os.chdir(wdir)
print(wdir)


VERBOSE = True

class arucoPubSub:

    np.set_printoptions(precision=4,suppress=True)
    markermap = pd.read_excel(r'markerworldmap.xlsx')
    rotM = np.zeros(shape=(3,3))

    maparray = [[0,0,0] for i in range(250)]
    mcount = 26

    idmap = markermap['ID'].tolist()

    xmap = markermap['x'].tolist()
    ymap = markermap['y'].tolist()
    zmap = markermap['z'].tolist()

    umap = markermap['rx'].tolist()
    vmap = markermap['ry'].tolist()
    wmap = markermap['rz'].tolist()

    for i in range(250-mcount):
        idmap.append(0)
        xmap.append(0)
        ymap.append(0)
        zmap.append(0)
        umap.append(0)
        vmap.append(0)
        wmap.append(0)

    mapTvecs = [[] for i in range(250)]
    mapRvecs = [[] for i in range(250)]

    for i in range(250):
        if(idmap[i]==0):
            mapTvecs[i]=[0,0,0]
            mapRvecs[i]=[0,0,0]
        else:
            mapTvecs[idmap[i]] = [xmap[i],ymap[i],zmap[i]]
            mapRvecs[idmap[i]] = [umap[i],vmap[i],wmap[i]]
            maparray[idmap[i]][0] = idmap[i]
            maparray[idmap[i]][1] = mapTvecs[idmap[i]]
            maparray[idmap[i]][2] = mapRvecs[idmap[i]]

    def getCoordArray(self,markerID):
        # coordArray = np.zeros((2,3))
        coordArray = [[0,0,0],[0,0,0]]
        coordArray[0] = self.maparray[int(markerID)][1]
        coordArray[1] = self.maparray[int(markerID)][2]
        return np.array(coordArray)

    def invT(self,tvec,rvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        R=-R
        invTvec = np.dot(R,tvec.T)
        invRvec, _ = cv2.Rodrigues(-R)#+-?
        invTvec = np.reshape(invTvec, (1,3))
        invTvec = np.array(invTvec)
        return invTvec

    def invR(self,tvec,rvec):
        R, _ = cv2.Rodrigues(rvec)
        R = np.matrix(R).T
        R=-R
        invTvec = np.dot(R,tvec.T)
        invRvec, _ = cv2.Rodrigues(-R)#+-?
        invRvec = np.reshape(invRvec, (1,3))
        invRvec = np.array(invRvec)
        return invRvec

    def vectorTransform(self,ID,tvec,rvec):
        rotM, _ = cv2.Rodrigues(np.array([self.getCoordArray(ID)[1]]))
        translate = np.array([self.getCoordArray(ID)[0]])
        rotM = np.matrix(rotM)
        # Trot = np.dot(rotM,(translate+tvec).T)#+np.dot(rotM,tvec.T)
        Trot = translate.T + np.dot(rotM,tvec.T)
        Trot = np.array(Trot).T
        Rrot = np.dot(rotM,rvec.T)
        Rrot = np.array(Rrot).T
        return np.array([Trot,Rrot])
    
    globx=[]
    globy=[]
    globz=[]

    globrX=[]
    globrY=[]
    globrZ=[]
    
    weights=[]


    def __init__(self):

        self.image_pub = rospy.Publisher("/processed_image/image_raw/compressed", CompressedImage, queue_size = 1,)
        self.pose_pub = rospy.Publisher("/visualPose", Pose, queue_size = 1)

        self.subscriber = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback, queue_size = 1, buff_size = 2**24)
        if VERBOSE :
            print("subscribed to /camera/rgb/image_raw/compressed")

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        imwidth, imheight = image_np.shape[:2]
        now = rospy.Time.now()

        color = (255, 0, 255)
        color2 = (0,0,255)
        color3 = (255,25,255)
        font = cv2.FONT_HERSHEY_SIMPLEX

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters =  cv2.aruco.DetectorParameters_create()
        #parameters.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR 


        msize = 0.16
        axlen = 0.10
        mtx =  np.array([[1.09985718e+03, 0.00000000e+00, 6.39960139e+02],                                                                                        
                        [0.00000000e+00, 1.09985718e+03 , 4.77231027e+02],                                                                                        
                        [0.00000000e+00, 0.00000000e+00 ,1.00000000e+00]]) 
        dist = np.array([[-3.20122590e+01],                                                                                                                     
                        [ 2.93055766e+02],                                                                                                                     
                        [-4.67823155e-03],                                                                                                                     
                        [ 1.52648785e-03],                                                                                                                     
                        [-3.03260701e+02],                                                                                                                     
                        [-3.20006459e+01],                                                                                                                     
                        [ 2.92680508e+02],                                                                                                                     
                        [-3.00351572e+02],                                                                                                                     
                        [ 0.00000000e+00],                                                                                                                     
                        [ 0.00000000e+00],                                                                                                                     
                        [ 0.00000000e+00],                                                                                                                     
                        [ 0.00000000e+00],                                                                                                                     
                        [ 0.00000000e+00],                                                                                                                     
                        [ 0.00000000e+00]])   

        
        frame = image_np
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)

            

        if len(markerCorners)> 0:
            rvecs,tvecs, trash = aruco.estimatePoseSingleMarkers(markerCorners, msize, mtx, dist)
            imaxes = aruco.drawDetectedMarkers(frame.copy(), markerCorners, markerIds)
            j = 0
            idatax=[]
            idatay=[]
            idataz=[]

            idatarX=[]
            idatarY=[]
            idatarZ=[]

            weights=[]


            for i in range(len(tvecs)):

                exceldata = pd.DataFrame(data = tvecs.reshape(len(tvecs),3), columns = ["tx", "ty", "tz"],
                index =markerIds.flatten())
                exceldata.index.name = "marker"
                exceldata.sort_index(inplace= True)

                invTvecs=[]
                invRvecs=[]

                mapTvectors=[]
                mapRvectors=[]

                for k in range(len(tvecs)):
                    invTvecs.append(self.invT(tvecs[k],rvecs[k]))
                    invRvecs.append(self.invR(tvecs[k],rvecs[k]))

                idatax.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[0][0][0])
                idatay.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[0][0][1])
                idataz.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[0][0][2])

                idatarX.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[1][0][0])
                idatarY.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[1][0][1])
                idatarZ.append(self.vectorTransform(markerIds[i],invTvecs[i],invRvecs[i])[1][0][2])


                imaxes = aruco.drawAxis(imaxes, mtx, dist, rvecs[i], tvecs[i], axlen)
                cv2.putText(imaxes,'ID', (50,200), font, 0.5, color3,2)
                cv2.putText(imaxes,'POS-1', (100,200), font, 0.5, color3,2)
                cv2.putText(imaxes,'ROT-1', (400,200), font, 0.5, color3,2)
                cv2.putText(imaxes,'ID:'+str(markerIds[i]), (40,220+j), font, 0.5, color,2)
                cv2.putText(imaxes,'  T:'+str(tvecs[i]), (100,220+j), font, 0.3, color,2)
                cv2.putText(imaxes,' R:'+str(rvecs[i]), (400,220+j), font, 0.3, color,2)
                cv2.putText(imaxes ,  str(imheight)+"x"+ str(imwidth),(10,20),font,0.5,(0,0,0),2)
            

                j+=20

                weights.append(1/(invTvecs[i][0][2]))

                idataXavg = np.average(idatax,weights=weights)
                idataYavg = np.average(idatay,weights=weights)
                idataZavg = np.average(idataz,weights=weights)

                idataRXavg = np.average(idatarX,weights=weights)
                idataRYavg = np.average(idatarY,weights=weights)
                idataRZavg = np.average(idatarZ,weights=weights)

            cv2.putText(imaxes,'GLOBAL POSITION VECTOR:', (10,40), font, 0.5, color3,2)
            cv2.putText(imaxes,'GLOBAL ROTATION VECTOR:', (10,90), font, 0.5, color3,2)
            cv2.putText(imaxes,str([idataXavg,idataYavg,idataZavg]), (10,60), font, 0.5, color,2)
            cv2.putText(imaxes,str([idataRXavg,idataRYavg,idataRZavg]), (10,110), font, 0.5, color,2)

            msg = CompressedImage()   
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg',imaxes)[1]).tostring()
            self.image_pub.publish(msg)

            p = Pose()

            p.position.x = idataXavg
            p.position.y = idataYavg    
            p.position.z = idataZavg

            angleMag = math.sqrt(idataRXavg**2+idataRYavg**2+idataRZavg**2)
            normRX = idataRXavg/angleMag
            normRY = idataRYavg/angleMag
            normRZ = idataRZavg/angleMag 
            quat = (normRX,normRY,normRZ,angleMag)

            normQuat = math.sqrt(normRX**2+normRY**2+normRZ**2+angleMag**2)
            QuatORX = normRX/normQuat   
            QuatORY = normRY/normQuat
            QuatORZ = normRZ/normQuat
            QuatORW = angleMag/normQuat

            p.orientation.x = QuatORX
            p.orientation.y = QuatORY
            p.orientation.z = QuatORZ
            p.orientation.w = QuatORW
            

            self.pose_pub.publish(p)








        else:
            cv2.putText(frame_markers,'DISCON', (50,150), font, 2, color2,3)


            cv2.putText(frame_markers , str(imheight)+"x"+ str(imwidth),(10,20),font,0.5,(0,0,0),2)
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

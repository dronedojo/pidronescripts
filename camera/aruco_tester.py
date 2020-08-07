import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

import time
import os
import platform
import sys
#############################
cap = WebcamVideoStream(src=0).start()

viewVideo=True
if len(sys.argv)>1:
    viewVideo=sys.argv[1]
    if viewVideo=='0' or viewVideo=='False' or viewVideo=='false':
        viewVideo=False
############ARUCO/CV2############
id_to_find=72
marker_size=19 #cm

width=640
height=480

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

calib_path="/home/pi/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
#############################

seconds=0
if viewVideo==True:
    seconds=1000000
    print("Showing video feed if X11 enabled.")
    print("Script will run until you exit.")
    print("-------------------------------")
    print("")
else:
    seconds=5
counter=0
start_time=time.time()
while time.time()-start_time<seconds:
    frame = cap.read() #for Threaded webcam
    
    frame = cv2.resize(frame,(width,height))
    
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)

    if ids is not None and ids[0] == id_to_find:
        ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
        rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]

        x="{:.2f}".format(tvec[0])
        y="{:.2f}".format(tvec[1])
        z="{:.2f}".format(tvec[2])
        #print("FOUND ARUCO!")
        marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
        print(marker_position)
        if viewVideo==True:
            aruco.drawDetectedMarkers(frame_np,corners)
            aruco.drawAxis(frame_np,cameraMatrix,cameraDistortion,rvec,tvec,10)
            cv2.imshow('frame',frame_np)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:
        print("ARUCO NOT FOUND.")
    counter=counter+1

if viewVideo==False:
    print("")
    print("")
    print("---------------------------")
    print("Loop iterations per second:")
    print(counter/seconds)
    print("---------------------------")

    print("Performance Diagnosis:")
    frequency=counter/seconds
    if frequency>10:
        print("Performance is more than enough for great precision landing.")
    elif frequency>5:
        print("Performance likely still good enough for precision landing, but pushing it.")
    else:
        print("Performance likely not good enough for precision landing.")
        print("MAKE SURE YOU HAVE A HEAT SINK ON YOUR PI!!!")
    print("---------------------------")
cv2.destroyAllWindows()

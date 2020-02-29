import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
from cv2 import aruco
import yaml
import glob
import rospy
from std_msgs.msg import String

# def convert2Dto3D(Point2D, R, T, cameraMatrix):
#
#     RotMatrix = cv2.Rodrigues(R)
#
#     RotMatrixInv = np.linalg.inv(RotMatrix)
#     cameraMatrixInv = np.linalg.inv(cameraMatrix)
#
#     tempMat = np.dot(RotMatrixInv,cameraMatrixInv)
#     tempMat = np.dot(tempMat, Point2D)
#     tempMat2 = np.dot(RotMatrixInv,T)
#     s = 0 + tempMat2[2, 0]
#     s /= tempMat[2, 0]
#     Point3D = np.dot(RotMatrixInv, np.dot(np.dot(s,cameraMatrixInv),Point2D) - T)
#
#     return Point3D


# pub = rospy.Publisher('chatter', String, queue_size=10)
# rospy.init_node('arucoDetector', anonymous=True)

frameRate = 100
vidCap = cv2.VideoCapture(0)

calibrationFile = "calibration.xml"
calibrationParams = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_READ)
cameraMatrix = calibrationParams.getNode("cameraMatrix").mat()
distCoeff = calibrationParams.getNode("distCoeff").mat()

axisLen = 5
markerLength = 6.9 #centimeters

nCornersCol = 9
nCornersROw = 6

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters_create()




while (True):
    ret, frame = vidCap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids != None:

        frame =  aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

        R, T, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix,  distCoeff)

        origin = np.float32([[0,0,0]]).reshape(-1,3)

        imgPoints2D, _ = cv2.projectPoints(origin, R, T, cameraMatrix, distCoeff)

        # convert2Dto3D(imgPoints2D, R, T, cameraMatrix)

        frame = aruco.drawAxis(frame, cameraMatrix, distCoeff, R, T, axisLen)

        #frame = cv2.line(frame, tuple(imgPoints2D[0][0]), tuple(imgPoints2D[0][0]), (255, 0, 255), 3)


        # cv2.imshow('frame', frame)
        # char = cv2.waitKey(0)

    cv2.imshow('frame', frame)

    #msg = str(imgPoints2D[0][0][0]) + ',' + str(imgPoints2D[0][0][1])
    # rospy.loginfo()
    #
    # pub.publish(msg)

    char = cv2.waitKey(1000 / frameRate)

    if (char == 27):  # esc char
        break;

# When everything done, release the capture
vidCap.release()
cv2.destroyAllWindows()
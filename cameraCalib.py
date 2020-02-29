import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import yaml
import cv2


calibrationFile = "calibration.xml"

nCornersCol = 6
nCornersROw = 9
frameRate = 4

objReal3DPoint = np.zeros((nCornersROw*nCornersCol,3), np.float32)
squareSideSize = 2.35 #centimeters

objReal3DPoint[:,:2] = np.mgrid[0:nCornersROw,0:nCornersCol].T.reshape(-1,2)*squareSideSize
realPoints3D = []
imgPoints2D = []

detectedFrames = 0
vidCap = cv2.VideoCapture(0)

while(detectedFrames < 20):  #long captures make calibrateCamera take too long
    ret, frameImg = vidCap.read()
    grayImg = cv2.cvtColor(frameImg, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(grayImg, (nCornersROw,nCornersCol), None)

    if ret == True:

        termCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cornersSubPix = cv2.cornerSubPix(grayImg,corners,(11,11),(-1,-1),termCriteria)
        imgPoints2D.append(cornersSubPix)
        realPoints3D.append(objReal3DPoint)
        frameImg = cv2.drawChessboardCorners(frameImg, (nCornersROw, nCornersCol), cornersSubPix, ret)
        detectedFrames += 1

    cv2.imshow('img', frameImg)
    char = cv2.waitKey(1000/frameRate)

    if (char == 27): #esc char
        break;

vidCap.release()
cv2.destroyAllWindows()

ret, cameraMatrix, distCoeff, R, T = cv2.calibrateCamera(realPoints3D, imgPoints2D, grayImg.shape[::-1], None, None)

fs = cv2.FileStorage(calibrationFile, cv2.FILE_STORAGE_WRITE)

fs.write("cameraMatrix", cameraMatrix)
fs.write("distCoeff", distCoeff)
fs.release()


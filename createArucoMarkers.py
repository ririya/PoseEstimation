import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from cv2 import aruco

arucoDictionary = aruco.getPredefinedDictionary( aruco.DICT_6X6_50)

arucoImg = aruco.drawMarker(arucoDictionary, 6, 200)

cv2.imshow('frame',arucoImg)
cv2.waitKey()

cv2.imwrite('arucoMarker.png', arucoImg)
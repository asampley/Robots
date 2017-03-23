#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
import glob
import sys
import math
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

mtx = np.array([[612.372615, 0.000000, 319.855223], [0.000000, 611.047376, 245.946766], [0.000000, 0.000000, 1.000000]], dtype=np.float64)
dist = np.array([-0.027223, 0.098406, 0.002497, -0.001910, 0.000000], dtype=np.float64)
# Load previously saved data
#with cv2.load(calibrationFileName) as X:
#    mtx, dist, _, _ = [X[i] for i in ('camera_matrix','distortion_coefficients')]

tvec=np.zeros((1,3))
rvec=np.zeros((1,3))

def draw(img, imgpts):
    corner = tuple(imgpts[0].ravel())
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[3].ravel()), (0,0,255), 5)
    return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#objp = np.zeros((6*8,3), np.float32)
#objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

axis = np.float32([[0,0,0], [.1,0,0], [0,.1,0], [0,0,-.1]]).reshape(-1,3)
cv2.namedWindow('img', 1)
cv2.waitKey(1)
bridge = cv_bridge.CvBridge()

def image_callback(msg):
  global tvec, rvec
#  print(msg)
  img = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
  # project 3D points to image plane
  imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
  print("Position: " + str(tvec))
  print("Rotation: " + str(rvec))
  print(imgpts)
  draw(img, imgpts)
  cv2.imshow('img',img)
  cv2.waitKey(1)
#  ret, corners = cv2.findChessboardCorners(gray, (8,6),None, cv2.CALIB_CB_FAST_CHECK)

#  if ret == True:
#    cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

#        print(criteria)        
#        print(corners)
#        print(corners2)

        # Find the rotation and translation vectors.
#    rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)

def marker_callback(msg):
	global tvec, rvec
	if len(msg.markers) > 0:
		"""
		x = msg.markers[0].pose.pose.position.x
		y = msg.markers[0].pose.pose.position.y
		z = msg.markers[0].pose.pose.position.z
		ox = msg.markers[0].pose.pose.orientation.x
		oy = msg.markers[0].pose.pose.orientation.y
		oz = msg.markers[0].pose.pose.orientation.z
		ow = msg.markers[0].pose.pose.orientation.w
		tvec = np.array([[x],[y],[z]])

		angle = 2 * math.acos(ow)
		x = ox / math.sqrt(1 - ow*ow)
		y = oy / math.sqrt(1 - ow*ow)
		z = oz / math.sqrt(1 - ow*ow)

		ratio = math.sqrt(x*x+y*y+z*z)    
		x = -x/(ratio * angle)
		y = -y/(ratio * angle)
		z = -z/(ratio * angle)

		rvec = np.array([[x],[y],[z]])
		"""
		#print("Position: " + str(tvec))
		#print("Rotation: " + str(rvec))

		tvec[0,0] = msg.markers[0].pose.pose.position.x
		tvec[0,1] = msg.markers[0].pose.pose.position.y
		tvec[0,2] = msg.markers[0].pose.pose.position.z
		rvec[0,0] = msg.markers[0].pose.pose.orientation.x
		rvec[0,1] = msg.markers[0].pose.pose.orientation.y
		rvec[0,2] = msg.markers[0].pose.pose.orientation.z

		#print("Position: " + str(tvec))
		#print("Rotation: " + str(rvec))
	else:
		print("No Markers Found")

#    img = draw(img,corners,imgpts)
#    print(img)
#        k = cv2.waitKey(0) & 0xff
#        if k == 's':
#        cv2.imwrite(fname[6:]+'_axes.png', img)

rospy.init_node('axis_drawer')
rospy.Subscriber('image', Image, image_callback)
rospy.Subscriber('marker', AlvarMarkers, marker_callback)
rospy.spin()

#!/usr/bin/env python

import cv2,rospy,cv_bridge,math
import numpy as np
import glob
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Load previously saved data
#with np.load('B.npz') as X:
#    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
mtx = np.array([
  [544.956062, 0.000000, 318.427876], 
  [0.000000, 549.845690, 235.690027], 
  [0.000000, 0.000000, 1.000000]])

dist = np.array([0.052190, -0.149998, 0.001184, -0.006267, 0.000000])
#dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    #img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    #img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    #img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    #return img

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

objp = objp * 0.026
print(objp)

axis = np.float32([[0.1,0,0], [0,0.1,0], [0,0,-0.1]]).reshape(-1,3)

#print("running")
#cv2.imshow("test",image)
cv2.namedWindow("axis",1)
#cv2.waitKey(3)

class Drawer:
  def __init__(self):
    #print("starting")
    cv2.namedWindow("axis",1)
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber ('image',Image,self.image_callback, queue_size=1)
    self.cmd_vel_pub = rospy.Publisher('twist_out',Twist, queue_size=1)
    self.twist = Twist()


  def image_callback(self,msg):
    #print("bleh")
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
#    cv2.imshow("axis", image)
    cv2.waitKey(1)

    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
#    ret, corners = cv2.findChessboardCorners(gray, (8,6), None, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

#    cv2.drawChessboardCorners(image, (8,6), corners, ret)

    if ret == True:
        #corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        #cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        #rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        #image = draw(image,corners,imgpts)
        corner = tuple(corners[0].ravel())
        cv2.line(image, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        cv2.line(image, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        cv2.line(image, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        

#        im00 = imgpts[0][0][0]
#        im01 = imgpts[0][0][1]
#        print("0")
#        print(imgpts[0])
#        print("1")
#        print(imgpts[1])
#        print("2")
#        print(imgpts[2])
        print("target" + str(tvecs))
        print("rotation" + str(rvecs))

        # if we are far from the checkerboard, move forward and turn
        if tvecs[2] > 0.3:
          self.twist.linear.x = 0.05
          if tvecs[0] > 0.1:
            self.twist.angular.z = -0.1
          elif tvecs[0] < -0.1:
            self.twist.angular.z = 0.1
          else:
            self.twist.angular.z = 0
        else:
          self.twist.linear.x = 0
          self.twist.angular.z = 0

        # if we are not directly facing the checkerboard, turn

        self.cmd_vel_pub.publish(self.twist)
        print("twist" + str(self.twist))

    else:
        self.twist.linear.x = 0
        self.twist.angular.z = 0.1
        self.cmd_vel_pub.publish(self.twist)

        #cv2.imshow('img',img)
    cv2.imshow("axis",image)
    cv2.waitKey(1)

rospy.init_node('axis_shower')
drawer = Drawer()
rospy.spin()



"""
for fname in glob.glob('left*.png'):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
        rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

        img = draw(img,corners2,imgpts)
        cv2.imshow('img',img)
        k = cv2.waitKey(0) & 0xff
        if k == 's':
            cv2.imwrite(fname[:6]+'.png', img)

cv2.destroyAllWindows()
"""

#!/usr/bin/env python

import cv2,rospy,cv_bridge,math
import numpy as np
import glob
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

go = False
# Load previously saved data
#with np.load('B.npz') as X:
#    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
mtx = np.array([
  [544.956062, 0.000000, 318.427876], 
  [0.000000, 549.845690, 235.690027], 
  [0.000000, 0.000000, 1.000000]])

dist = np.array([0.052190, -0.149998, 0.001184, -0.006267, 0.000000])
#dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
move_mode = False
twist_and_duration = []
target_to_be_reached = False
target_reached = False
target_found = False

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
    cv2.namedWindow("target", 1)
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber ('image',Image,self.image_callback, queue_size=1)

  def image_callback(self,msg):
    global target_found, twist_and_duration, go, target_to_be_reached
    #print("bleh")
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
#    print(str(target_found) + " " + str(go))
    if target_found or not go:
      cv2.imshow("axis", image)
      cv2.waitKey(1)
      return

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
        
        print(rvecs)
        print(tvecs)
        # move the target to be 0.25m in front of the grid
        rvec_target, tvec_target, _,_,_,_,_,_,_,_ = cv2.composeRT(np.array([0,0,0], dtype=np.float32), np.array([0,0,-0.25], dtype=np.float32), rvecs, tvecs)
        print("target" + str(tvec_target))
        print("rotation" + str(rvec_target))

        imgpt, jac = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvec_target, tvec_target, mtx, dist)
        cv2.circle(image, tuple(imgpt.ravel()), 10, (255, 127, 255), -1)

        # first, turn to face the checkerboard
        rot_radians  = -(np.arcsin(tvec_target[0] / tvec_target[2]))[0]
        rot_velocity = np.sign(rot_radians) * 0.8
        rot_period   = rot_radians / rot_velocity
        rot_twist = Twist()
        rot_twist.angular.z = rot_velocity
        # second, move towards the checkerboard for .25m or within .1m
        move_meters = min(0.25, np.linalg.norm(tvec_target[np.array([0,2])]) - 0.1)
        move_velocity = 0.2
        move_period   = move_meters / move_velocity
        move_twist = Twist()
        move_twist.linear.x = move_velocity
        # make the list of actions
        twist_and_duration = [(rot_twist, rot_period), (move_twist, move_period)]
        # say that we found the target
        target_found = True
        # say that we will reach the target
        if move_meters < 0.24:
          target_to_be_reached = True

        print("Directions: " + str(twist_and_duration))

        #cv2.imshow('img',img)
    cv2.imshow("target",image)
    cv2.waitKey(1)

def go_callback(msg):
  global go, target_found, target_reached, target_to_be_reached
  go = msg.data
  if not go:
    target_found = False
    target_reached = False
    target_to_be_reached = False

rospy.init_node('axis_shower')
drawer = Drawer()
go_subscriber = rospy.Subscriber('go', Bool, go_callback)
cmd_vel_pub = rospy.Publisher('twist_out',Twist, queue_size=1)
twist = Twist()


while True:
  if not target_found and not target_reached:
    twist.linear.x = 0
    twist.angular.z = 0.1
    cmd_vel_pub.publish(twist)
  elif not target_reached:
    while twist_and_duration:
      print("Moving towards target")
      twist, duration = twist_and_duration.pop(0)
      cmd_vel_pub.publish(twist)
      rospy.sleep(duration)
    target_found = False
    if target_to_be_reached:
      target_reached = True
    

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

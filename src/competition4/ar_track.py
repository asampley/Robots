#!/usr/bin/env python

import cv2,rospy,cv_bridge,math
import numpy as np
import math
import glob
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers,AlvarMarker
from geometry_msgs.msg import Twist, PoseStamped,Pose,Quaternion, Point


mtx = np.array([
[576.023,0.0,286.262],
[0.0,580.862,186.268],
[0.0, 0.0, 1.0]])

dist = np.array([-0.00633,-0.00643,-0.00164,-0.0110,0.0])


axis = np.float32([[.1,0,0], [0,.1,0], [0,0,.1],[0,0,0]]).reshape(-1,3)

cv2.namedWindow("axis",1)

class Drawer:
  def __init__(self):
    cv2.namedWindow("axis",1)
    self.bridge = cv_bridge.CvBridge()
    self.pose_sub = rospy.Subscriber ('/ar_pose_marker',AlvarMarkers, self.pose_callback)  
    self.image_sub = rospy.Subscriber ('camera/rgb/image_raw',Image,self.image_callback)

    self.ar_pose_pub = rospy.Publisher('ar_pose',Pose,queue_size=1)

    self.markers = AlvarMarkers()
    self.image_points = [] 
    self.origin_point = Point()
    self.found = False

  def pose_callback(self,msg):
    global tvecs, rvecs
    self.markers=msg.markers
    #ind = 0
    #print("length:" + str(len(msg.markers)))
    if len(msg.markers) > 0:
      self.found = True
      x = msg.markers[0].pose.pose.position.x
      y = msg.markers[0].pose.pose.position.y
      z = msg.markers[0].pose.pose.position.z
      tvecs = np.array([[x],[y],[z]])

      ox = msg.markers[0].pose.pose.orientation.x
      oy = msg.markers[0].pose.pose.orientation.y
      oz = msg.markers[0].pose.pose.orientation.z
      ow = msg.markers[0].pose.pose.orientation.w

      angle = 2 * math.acos(ow)
      x = ox / math.sqrt(1 - ow*ow)
      y = oy / math.sqrt(1 - ow*ow)
      z = oz / math.sqrt(1 - ow*ow)

      length = math.sqrt(x*x+y*y+z*z)    
      x = x/length * angle
      y = y/length * angle
      z = z/length * angle

      rvecs = np.array([[x],[y],[z]])

    else: 
      self.found = False

  def image_callback(self,msg):
    #print(str(self.marker))
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
   
    #gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    #print("gah")
    if self.found:
      #print("foo")
      imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
      origin = tuple(imgpts[3].ravel())
      cv2.line(image, origin, tuple(imgpts[0].ravel()), (255,0,0), 5)
      cv2.line(image, origin, tuple(imgpts[1].ravel()), (0,255,0), 5)
      cv2.line(image, origin, tuple(imgpts[2].ravel()), (0,0,255), 5)
 
      pose = Pose()
      pose.position.x = tvecs[0]
      pose.position.y = tvecs[1]
      pose.position.z = tvecs[2]
      self.ar_pose_pub.publish(pose)

    cv2.imshow("axis",image)
    cv2.waitKey(1)


rospy.init_node('axis_shower')
drawer = Drawer()
rospy.spin()

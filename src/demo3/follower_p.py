#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    cv2.namedWindow("raw_mask", 1)
    cv2.namedWindow("refined_mask", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()

    self.lower_hsv = numpy.array([\
      rospy.get_param("~lower_hsv/h", 110),\
      rospy.get_param("~lower_hsv/s", 100),\
      rospy.get_param("~lower_hsv/v", 100)])
    self.upper_hsv = numpy.array([\
      rospy.get_param("~upper_hsv/h", 130),\
      rospy.get_param("~upper_hsv/s", 255),\
      rospy.get_param("~upper_hsv/v", 255)])

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
    cv2.imshow("raw_mask", mask)

    e_kernel = numpy.ones((3,3), numpy.uint8)
    d_kernel = numpy.ones((3,3), numpy.uint8)
    mask = cv2.dilate(mask, d_kernel, iterations=1)
    mask = cv2.erode(mask, e_kernel, iterations=2)
    mask = cv2.dilate(mask, d_kernel, iterations=1)
    cv2.imshow("refined_mask", mask)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 60
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

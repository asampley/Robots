#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window1", 1)
    cv2.namedWindow("window2", 2)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.twist = Twist()
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_blue = numpy.array([65,  10, 10])
    upper_blue = numpy.array([105, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # BEGIN CROP
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    # END CROP
    # BEGIN FINDER
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
    # END FINDER
    # BEGIN CIRCLE
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    # END CIRCLE

    cv2.imshow("window1", image)
    cv2.imshow("window2", mask)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

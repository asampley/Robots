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
    cv2.namedWindow("light_mask", 1)
    cv2.namedWindow("trimmed_mask", 1)
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
    
    self.light1_lower_hsv = numpy.array([\
      rospy.get_param("~light1/lower_hsv/h", 0),\
      rospy.get_param("~light1/lower_hsv/s", 200),\
      rospy.get_param("~light1/lower_hsv/v", 200)])
    self.light1_upper_hsv = numpy.array([\
      rospy.get_param("~light1/upper_hsv/h", 10),\
      rospy.get_param("~light1/upper_hsv/s", 255),\
      rospy.get_param("~light1/upper_hsv/v", 255)])
    self.light2_lower_hsv = numpy.array([\
      rospy.get_param("~light2/lower_hsv/h", 160),\
      rospy.get_param("~light2/lower_hsv/s", 200),\
      rospy.get_param("~light2/lower_hsv/v", 200)])
    self.light2_upper_hsv = numpy.array([\
      rospy.get_param("~light2/upper_hsv/h", 179),\
      rospy.get_param("~light2/upper_hsv/s", 255),\
      rospy.get_param("~light2/upper_hsv/v", 255)])

    self.line_search_bot = rospy.get_param("~line/search_bot_height_fraction", 1.0)
    self.line_search_top = rospy.get_param("~line/search_top_height_fraction", 0.75)


    self.prev_err = 0
    self.red_light = False

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # find light
    light_mask = cv2.inRange(hsv, self.light1_lower_hsv, self.light1_upper_hsv)
    light_mask = numpy.maximum(light_mask, cv2.inRange(hsv, self.light2_lower_hsv, self.light2_upper_hsv))

#    light_mask = cv2.erode(light_mask, numpy.ones((3,3), numpy.uint8), iterations=1)
    light_mask = cv2.dilate(light_mask, numpy.ones((5,5), numpy.uint8), iterations=1)
    cv2.imshow("light_mask", light_mask)

    M = cv2.moments(light_mask)
    if M['m00'] > 100:
      if not self.red_light:
        print("Red Light!!!")
      self.red_light = True
    else:
      if self.red_light:
        print("Green Light!!!")
      self.red_light = False


    # find line
    mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
    cv2.imshow("raw_mask", mask)

    e_kernel = numpy.ones((5,5), numpy.uint8)
    d_kernel = numpy.ones((3,3), numpy.uint8)
    mask = cv2.dilate(mask, d_kernel, iterations=1)
    mask = cv2.erode(mask, e_kernel, iterations=1)
    cv2.imshow("refined_mask", mask)
    
    h, w, d = image.shape
    search_top = int(float(h) * self.line_search_top)
    search_bot = int(float(h) * self.line_search_bot)

    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    
    cv2.imshow("trimmed_mask", mask)
    
    M = cv2.moments(mask)
    if M['m00'] > 0 and not self.red_light:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      derr = err - self.prev_err
      kp = 0.01
      kd = 0.01
      self.twist.linear.x = 0.5
      self.twist.angular.z = -float(err) * kp + float(derr) * kd
      self.cmd_vel_pub.publish(self.twist)

      self.prev_err = err
      # END CONTROL
    else:
      self.cmd_vel_pub.publish(Twist())
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

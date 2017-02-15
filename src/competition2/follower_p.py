#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    #cv2.namedWindow("raw_mask", 1)
    cv2.namedWindow("white_refined_mask", 1)
    cv2.namedWindow("yellow_refined_mask", 1)
    cv2.namedWindow("light_mask", 1)
    cv2.namedWindow("white_trimmed_mask", 1)
    cv2.namedWindow("yellow_trimmed_mask", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()

    #white lines
    self.lower_hsv = numpy.array([\
      rospy.get_param("~lower_hsv/h", 110),\
      rospy.get_param("~lower_hsv/s", 50),\
      rospy.get_param("~lower_hsv/v", 50)])
    self.upper_hsv = numpy.array([\
      rospy.get_param("~upper_hsv/h", 130),\
      rospy.get_param("~upper_hsv/s", 255),\
      rospy.get_param("~upper_hsv/v", 255)])

    #yellow lines
    self.yellow_lower_hsv = numpy.array([\
      rospy.get_param("~yellow_lower_hsv/h", 10),\
      rospy.get_param("~yellow_lower_hsv/s", 50),\
      rospy.get_param("~yellow_lower_hsv/v", 100)])
    self.yellow_upper_hsv = numpy.array([\
      rospy.get_param("~yellow_upper_hsv/h", 30),\
      rospy.get_param("~yellow_upper_hsv/s", 255),\
      rospy.get_param("~yellow_upper_hsv/v", 255)])
    
    #Red lines
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


    self.white_prev_err = 0
    self.yellow_prev_err = 0
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
	
    #debug
    #self.red_light = False

    # find white line
    white_mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
    # cv2.imshow("raw_mask", mask)

    e_kernel = numpy.ones((5,5), numpy.uint8)
    d_kernel = numpy.ones((3,3), numpy.uint8)
    white_mask = cv2.dilate(white_mask, d_kernel, iterations=1)
    white_mask = cv2.erode(white_mask, e_kernel, iterations=1)
    cv2.imshow("white_refined_mask", white_mask)
    
    h, w, d = image.shape
    search_top = int(float(h) * self.line_search_top)
    search_bot = int(float(h) * self.line_search_bot)

    white_mask[0:search_top, 0:w] = 0
    white_mask[search_bot:h, 0:w] = 0
    cv2.imshow("white_trimmed_mask", white_mask)


    # find yellow line
    yellow_mask = cv2.inRange(hsv, self.yellow_lower_hsv, self.yellow_upper_hsv)
    # cv2.imshow("raw_mask", mask)

    e_kernel = numpy.ones((5,5), numpy.uint8)
    d_kernel = numpy.ones((3,3), numpy.uint8)
    yellow_mask = cv2.dilate(yellow_mask, d_kernel, iterations=1)
    yellow_mask = cv2.erode(yellow_mask, e_kernel, iterations=1)
    cv2.imshow("yellow_refined_mask", yellow_mask)
    
    h, w, d = image.shape
    search_top = int(float(h) * self.line_search_top)
    search_bot = int(float(h) * self.line_search_bot)

    yellow_mask[0:search_top, 0:w] = 0
    yellow_mask[search_bot:h, 0:w] = 0
    cv2.imshow("yellow_trimmed_mask", yellow_mask)


    whiteM = cv2.moments(white_mask)
    yellowM = cv2.moments(yellow_mask)
    if ((whiteM['m00'] > 0) or (yellowM['m00'] > 0) )  and not self.red_light:

      kp = 0.01
      kd = 0.01
      white_err = 0
      yellow_err = 0
	
      if(whiteM['m00'] != 0):
        white_cx = int(whiteM['m10']/whiteM['m00'])
        white_cy = int(whiteM['m01']/whiteM['m00'])
        cv2.circle(image, (white_cx, white_cy), 20, (0,0,180), -1)
        white_err = white_cx - float(w) * 0.75
        white_derr = white_err - self.white_prev_err
        self.twist.angular.z = -float(white_err) * kp + float(white_derr) * kd

      if(yellowM['m00'] != 0):
        yellow_cx = int(yellowM['m10']/yellowM['m00'])
        yellow_cy = int(yellowM['m01']/yellowM['m00'])
        cv2.circle(image, (yellow_cx, yellow_cy), 20, (0,90,180), -1)
        yellow_err = yellow_cx - float(w) * 0.25
        yellow_derr = yellow_err - self.yellow_prev_err
        self.twist.angular.z = -float(yellow_err) * kp + float(yellow_derr) * kd


      self.twist.linear.x = 0.1
      
      print(self.twist.angular.z)      

      self.cmd_vel_pub.publish(self.twist)

      self.white_prev_err = white_err
      self.yellow_prev_err = yellow_err
      # END CONTROL
    else:
      self.cmd_vel_pub.publish(Twist())
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

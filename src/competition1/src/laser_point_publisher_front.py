#!/usr/bin/env python

import rospy
import numpy
import math
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
    
scan_data = LaserScan()

point_list = []
clear_list = False
num_points = 10
point_decay = 0.5

class Point2D:
  def __init__(self, x=0, y=0):
    self.x = x
    self.y = y
  
  def dist(self, otherPoint):
    dx = self.x - otherPoint.x
    dy = self.y - otherPoint.y
    return math.sqrt(dx * dx + dy * dy)

def update_scan(data):
    global scan_data
    scan_data = data

def joy_callback(msg):
    global clear_list
    
    if msg.buttons[2] == 1:
      clear_list = True
#-----------------------------------------

def publish_state():

    global scan_data, prev_points, num_points, point_decay, point_list, clear_list

    rospy.Subscriber('/scan',LaserScan,update_scan)
    rospy.Subscriber('/joy', Joy, joy_callback)

    pub = rospy.Publisher('/point_follower/point',Point, queue_size=1)

    rospy.init_node('laser_point_pub', anonymous=True)

    rate = rospy.Rate(10)
    point = Point()

    while not rospy.is_shutdown():
        if clear_list:
          point_list = []
          clear_list = False

        #print scan_data.ranges[0]
        a_min = scan_data.angle_min
        a_inc = scan_data.angle_increment
        a_list = [a_min]
        a_max = a_min
        for i in range(len(scan_data.ranges)-1):
            a_max += a_inc
            a_list.append(a_max)

        # Find min and corresponding angle

        bestr = 100000
        besta = 100000
        besth = 100000
        dx = 100000
        dy = 100000

        point_valid = 0

        for i in range(len(scan_data.ranges)):
            r = scan_data.ranges[i]
            a = a_list[i]

            dx =  r * numpy.cos(a)
            dy =  r * numpy.sin(a)

            curr_point = Point2D(dx, dy)

            # heuristic for near to previous points
            if point_list:
              h = 0
              for li in range(0, len(point_list)):
                h += math.pow(point_decay, li) * point_list[li].dist(curr_point)
            else:
              h = 0
            
            if bestr + besth > r + h:
                bestr = r
                besth = h
                besta = a_list[i]
                point_valid = 1

        if point_valid:

            dx = bestr * numpy.cos(besta)
            dy = bestr * numpy.sin(besta)

            if len(point_list) == num_points:
              point_list.pop()
            point_list.insert(0, Point2D(dx, dy))

            print "Best (r, a, h) : (" + str(bestr) + ", " + str(besta) + ", " + str(besth) + ")"

            x_offset = 0.0
            y_offset = 0.0

            point.x = dx + x_offset
            point.y = dy + y_offset
            pub.publish(point)

            print "Target at point: (" + str(point.x) + ", " + str(point.y) + ")"
        rate.sleep()

if __name__ == "__main__": 
    publish_state()

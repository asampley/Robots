#!/usr/bin/env python

import rospy
import numpy
import math
from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
    
scan_data = LaserScan()
prev_x1 = None
prev_y1 = None
prev_x2 = None
prev_y2 = None

def update_scan(data):
    global scan_data
    scan_data = data
#-----------------------------------------

def publish_state():

    global scan_data, prev_x1, prev_x2, prev_y1, prev_y2

    rospy.Subscriber('/scan',LaserScan,update_scan)

    pub = rospy.Publisher('/point_follower/point',Point, queue_size=1)

    rospy.init_node('laser_point_pub', anonymous=True)

    rate = rospy.Rate(10)
    point = Point()

    while not rospy.is_shutdown():

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

            # heuristic for near to previous point
            if prev_x1 and prev_y1 and prev_x2 and prev_y2:
              curr_vel_x = dx - prev_x1
              curr_vel_y = dy - prev_y1
              prev_vel_x = prev_x1 - prev_x2
              prev_vel_y = prev_y1 - prev_y2

              h = math.pow(curr_vel_x - prev_vel_x, 2) + math.pow(curr_vel_y - prev_vel_y, 2)
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

            prev_x2 = prev_x1
            prev_y2 = prev_y1

            prev_x1 = dx
            prev_y1 = dy

            print "Best (r, a, h) : (" + str(bestr) + ", " + str(besta) + ", " + str(besth) + ")"

            x_offset = 0.0
            y_offset = -0.19 #0.19

            point.x = dx + x_offset
            point.y = dy + y_offset
            pub.publish(point)

            print "Target at point: (" + str(point.x) + ", " + str(point.y) + ")"
        rate.sleep()

if __name__ == "__main__": 
    publish_state()

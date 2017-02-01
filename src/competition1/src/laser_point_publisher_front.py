#!/usr/bin/env python

import rospy
import numpy

from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
    
scan_data = LaserScan()
prev_x = None
prev_y = None

def update_scan(data):
    global scan_data
    scan_data = data
#-----------------------------------------

def publish_state():

    global scan_data

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
            if prev_x and prev_y:
              h = math.pow(dx - prev_x, 2) + math.pow(dy - prev_y, 2)
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

            prev_x = dx
            prev_y = dy

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

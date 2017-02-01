#!/usr/bin/env python

import rospy
import numpy

from std_msgs.msg import UInt16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
    
scan_data = LaserScan()

def update_scan(data):
    global scan_data
    scan_data = data
#-----------------------------------------

def publish_state():

    global scan_data

    rospy.Subscriber('/scan',LaserScan,update_scan)

    pub = rospy.Publisher('/point_follower/point',Point)

    rospy.init_node('laser_point_pub', anonymous=True)

    r = rospy.Rate(10)
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

        d1 = 100000
        a1 = 100000
        dx = 100000
        dy = 100000

        point_valid = 0

        for i in range(len(scan_data.ranges)):
            if d1 > scan_data.ranges[i]:
                d1 = scan_data.ranges[i]
                a1 = a_list[i]
                point_valid = 1

        if point_valid:

            dx =  d1 * numpy.cos(a1)
            dy =  d1 * numpy.sin(a1)


            x_offset = 0.0
            y_offset = -0.19 #0.19

            point.x = dx + x_offset
            point.y = dy + y_offset
            pub.publish(point)

        r.sleep()

if __name__ == "__main__": 
    publish_state()

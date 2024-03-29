#!/usr/bin/env python

import rospy
import numpy

from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

point = Point()
time_last_point = 0
node_init_flag = 0

def update_point(data):
    global point
    global time_last_point
    global node_init_flag
    if node_init_flag:
        time_last_point = rospy.get_time()
        point = data
#-----------------------------------------

def publish_state():
    
    global time_last_point
    global node_init_flag

    global point

    rospy.Subscriber('/point_follower/point',Point,update_point)

    #pub = rospy.Publisher('/sys_states/state2',UInt16)
    pub_vel = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

    rospy.init_node('point_follower_front', anonymous=True)
    rate = rospy.Rate(20)

    cmd = Twist()
    node_init_flag = 1

    vr = 0
    wr = 0

    ed = 0
    ed_old = 0
    ea = 0
    ea_old = 0
    while not rospy.is_shutdown():

        time_now = rospy.get_time()
        point_valid = 0

        if abs(time_now - time_last_point) < 0.2:
            point_valid = 1

        if point_valid:

            d = numpy.sqrt(point.x**2 + point.y**2)
            a = numpy.arctan2(point.y,point.x)

            # check if point is in valid limits:

            dmax =   5.5
            dmin =   0.6
            amax =   1.0
            amin = - 1.0
            point_inside_limits = 0
            if d > dmin and d < dmax and a > amin and a < amax:
                point_inside_limits = 1

            else:
                vr = 0
                wr = 0

            if point_inside_limits:
                #print d, a
                dr = 0.8
                ar = 0.00

                # Linear velocity:

                KPd = - 2.0
                KDd = 0.0

                ed = float(dr - d)
                print ed
                ed_delta = (ed - ed_old)/float(0.05)
                ed_old = ed

                vr = 0
                if abs(ed) > 0.01:
                    vr = KPd * ed + KDd * ed_delta
                    #print ed, ed_delta ,KPd * ed ,  KDd * ed_delta

                if vr > 1.0:
                    vr = 1.0
                if vr < -1.0:
                    vr = -1.0

                # Angular Velocity
                KPa = 2.0
                KDa = 0.0

                ea = float(ar - a)
                ea_delta = (ea - ea_old)/float(0.05)
                ea_old = ed

                wr = 0
                if abs(ea) > 0.01 and abs(vr) > 0.1:
                    wr = KPa * ea + KDa * ea_delta

                wdir = 0

                if d > dr and a > ar:
                    wdir = 1
                if d > dr and a < ar:
                    wdir = -1
                if d < dr and a > ar:
                    wdir = 1 
                if d < dr and a < ar: 
                    wdir = - 1

                wr = wdir * abs(wr)


        str = "%s %s"%(vr,wr)
        #rospy.logwarn(str)
        #print vr, wr
        cmd.linear.x = vr 
        cmd.angular.z = wr
        pub_vel.publish(cmd)

        rate.sleep()

if __name__ == "__main__": 
    publish_state()



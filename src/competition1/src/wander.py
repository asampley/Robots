#!/usr/bin/env python
# BEGIN ALL
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

def scan_callback(msg):
  global g_range_ahead

  ranges = [x for x in msg.ranges if not math.isnan(x)]

  #print "Data: " + str(ranges)

  g_range_ahead = min(ranges)

def joy_callback(msg):
  global go

  button = msg.buttons[2] # x button
  
  if button == 1:
    go = not go

go = False
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  if go:
    if driving_forward:
      # BEGIN FORWARD
      if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
        driving_forward = False
        state_change_time = rospy.Time.now() + rospy.Duration(2)
      # END FORWARD
    else: # we're not driving_forward
      # BEGIN TURNING
      if rospy.Time.now() > state_change_time:
        driving_forward = True # we're done spinning, time to go forwards!
        state_change_time = rospy.Time.now() + rospy.Duration(30)
      # END TURNING
    twist = Twist()
    if driving_forward:
      twist.linear.x = 0.2
    else:
      twist.angular.z = 1
    cmd_vel_pub.publish(twist)
  else: # if not go
    cmd_vel_pub.publish(Twist())
  
  print "Range ahead: " + str(g_range_ahead)

  rate.sleep()
# END ALL

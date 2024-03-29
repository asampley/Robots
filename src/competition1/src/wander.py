#!/usr/bin/env python
# BEGIN ALL
import rospy
import math
from random import randint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

def scan_callback(msg):
  global g_range_ahead

  angle_capture = 0.80 # in radians

  range_min_i = (int)(math.floor((-(msg.angle_min) - (angle_capture / 2)) / msg.angle_increment))
  range_max_i = len(msg.ranges) - (int)(math.floor((msg.angle_max - (angle_capture / 2)) / msg.angle_increment))
  
  #print "Trim from " + str(range_min_i) + " to " + str(range_max_i)
  
  ranges = msg.ranges[range_min_i:range_max_i]
  nanCount = 0;
  for x in ranges:
    if math.isnan(x):
      nanCount = nanCount +1


  #print "Nans:" + str(nanCount);
  #print "Data: " + str(ranges)


  if(ranges):
    g_range_ahead = min(ranges)
    if(nanCount > 30):
      g_range_ahead = 0.0;

def joy_callback(msg):
  global go

  button = msg.buttons[2] # x button
  
  if button == 1:
    go = not go

go = False
#go = True
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)
turn_dir = 1

"""
while not rospy.is_shutdown():
  #print "looping"
  if go:
    print "going"
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
"""

while not rospy.is_shutdown():
  if go:

    if (g_range_ahead < 0.9):
      driving_forward = False
    else:
      driving_forward = True 
      if randint(0, 1) == 0:
        turn_dir = 1
      else:
        turn_dir = -1

    twist = Twist()
    #change liner.x to 0.8 for maximum speed
    if driving_forward:
      twist.linear.x = 1.3
      twist.angular.z = 0.0
    else:
      twist.angular.z = turn_dir * 3.5
      twist.linear.x = 0.0

    cmd_vel_pub.publish(twist)

  else: # if not go
    cmd_vel_pub.publish(Twist())

  #print "Range ahead: " + str(g_range_ahead)

  rate.sleep()
# END ALL

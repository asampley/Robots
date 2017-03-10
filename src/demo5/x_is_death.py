#!/usr/bin/env python
# BEGIN ALL
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joy_callback(msg):
  global go

  button = msg.buttons[2] # x button
  
  if button == 1:
    go = not go
    print "Go: " + str(go)

def twist_callback(msg):
  global twist_in

  twist_in = msg

go = False
twist_in = Twist()

joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
twist_sub = rospy.Subscriber('twist_in', Twist, twist_callback)
cmd_vel_pub = rospy.Publisher('twist_out', Twist, queue_size=1)
go_pub = rospy.Publisher('go', Bool, queue_size=1)
rospy.init_node('x_is_death')
rate = rospy.Rate(10)

while not rospy.is_shutdown():
  if go:
    cmd_vel_pub.publish(twist_in)
    go_pub.publish(Bool(go))
  else: # if not go
    cmd_vel_pub.publish(Twist())
    go_pub.publish(Bool(go))

  rate.sleep()
# END ALL

#!/usr/bin/env python

import rospy
import actionlib
import sys
from geometry_msgs.msg import Pose
from kobuki_msgs.msg import Sound

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [  
	[(3.6607125401, 7.31828998725, 0.0), (0.0, 0.0, -0.899002073698, 0.437944370311)],
	[(-0.481131523099, 2.63816824607, 0.0), (0.0, 0.0, -0.368556378916, 0.929605397768)],
	[(1.56175864561, 0.84328832173, 0.0), (0.0, 0.0, 0.484581645217, 0.874746036927)],
	[(5.8438633289, 5.37321792854, 0.0), (0.0, 0.0, 0.905211098608, 0.424962194739)]
]
gather_targets = False
logo_pose = None
logo_found = False
ar_pose = None
ar_found = False

def goal_pose(pose, frame_id):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = frame_id
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]

	return goal_pose

def logo_pose_callback(msg):
	global gather_targets, logo_found, logo_pose
	if gather_targets:
		logo_found = True
		logo_pose = msg

def ar_pose_callback(msg):
	global gather_targets, ar_found, ar_pose
	if gather_targets:
		ar_found = True
		ar_pose = msg


if __name__ == '__main__':
	NUM_LAPS = 1
	SEARCH_PERIOD = 1
	WAYPOINT_TF_FRAME = 'map'
	DOCKING_TF_FRAME = 'base_footprint'

	LOGO_SOUND = Sound()
	AR_SOUND = Sound()
	WAYPOINT_SOUND = Sound()
	LOGO_SOUND.value = 0
	AR_SOUND.value = 1
	WAYPOINT_SOUND.value = 5

	rospy.init_node('patrol')

	sound_pub = rospy.Publisher('kobuki_sound', Sound, queue_size=1)
	logo_sub = rospy.Subscriber('logo_point', Pose, logo_pose_callback)
	ar_sub = rospy.Subscriber('ar_point', Pose, ar_pose_callback)

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
	client.wait_for_server()
 
	lap_counter = 0
 
	while (lap_counter < NUM_LAPS):
		print("Beginning lap " + str(lap_counter))
		for pose in waypoints:
			print("Moving to next waypoint")
			# move to next waypoint
			gather_targets = False
			goal = goal_pose(pose, WAYPOINT_TF_FRAME)
			client.send_goal(goal)
			client.wait_for_result()
			
			# play sound to indicate reaching of a waypoint
			sound_pub.publish(WAYPOINT_SOUND)

			# Look for targets for a duration of time
			gather_targets = True
			rospy.sleep(SEARCH_PERIOD)
			
			if logo_found:
				print("Docking with logo")
				goal = goal_pose(logo_pose, DOCKING_TF_FRAME)
				sound_pub.publish(LOGO_SOUND)
			elif ar_found:
				print("Docking with AR code")
				goal = goal_pose(ar_pose, DOCKING_TF_FRAME)
				sound_pub.publish(AR_SOUND)
			logo_found = False
			ar_found = False

		lap_counter = lap_counter + 1
	print("Completed all " + str(NUM_LAPS) + " laps")

#!/usr/bin/env python

import rospy
import actionlib
import sys
import numpy as np
import math
from geometry_msgs.msg import Pose, Twist
from kobuki_msgs.msg import Sound, Led
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

rospy.init_node('patrol')

dock = rospy.get_param('~dock', False)

waypoints = [
[(-0.119950234018,-0.907901944962,0.0),(0.0,0.0,-0.384930863174,0.922945410399)],
[(-0.0599337622866,-1.01160645021,0.0),(0.0,0.0,-0.904601712743,0.426257834299)],
[(-0.742077092604,-0.580275540601,0.0),(0.0,0.0,-0.904443138298,0.381319923496)],
[(-1.34421700246,-0.158327525501,0.0),(0.0,0.0,-0.913949694425,0.405827495446)],
[(-1.91672861406,0.256386610932,0.0),(0.0,0.0,-0.87634205007,0.481689330667)],
[(-2.58617071639,0.76624820661,0.0),(0.0,0.0,-0.920137442443,0.391595565622)],
[(-1.27416391139,2.5829593758,0.0),(0.0,0.0,0.877247884483,0.480037653909)],
[(-0.820077128675,3.1543686682,0.0),(0.0,0.0,0.950986066213,0.309233733395)],
[(-0.361373536451,3.63026180198,0.0),(0.0,0.0,0.952104328607,0.305773359612)],
[(0.052931075014,4.11964141754,0.0),(0.0,0.0,0.905707760474,0.423902645212)],
[(0.731904631267,4.91704274085,0.0),(0.0,0.0,0.919704568788,0.392611138597)],
[(1.23920044856,5.49657367306,0.0),(0.0,0.0,0.91513685073,0.403143331155)],
[(1.64519961727,6.01622208067,0.0),(0.0,0.0,0.925300646371,0.379234378485)],
[(2.12391807627,6.60537586829,0.0),(0.0,0.0,0.922525279984,0.385936662926)],
[(2.60099654951,7.17368924683,0.0),(0.0,0.0,0.915951836811,0.401288216426)],
[(3.97422848316,8.67178957646,0.0),(0.0,0.0,0.40341438627,0.91501739489)],
[(4.70082732653,8.53146093341,0.0),(0.0,0.0,0.352884320317,0.935666958097)],
[(5.2958607171,7.98648175443,0.0),(0.0,0.0,0.328095173612,0.944644672378)],
[(5.82882917544,7.51351477618,0.0),(0.0,0.0,0.343175283013,0.939271379916)],
[(6.3863760458,7.05051300793,0.0),(0.0,0.0,0.420479456014,0.907302059443)],
[(6.82305635146,6.62854296252,0.0),(0.0,0.0,0.377746910528,0.925908889463)],
[(6.83796949567,6.64007977559,0.0),(0.0,0.0,-0.241589934588,0.970378433141)],
[(6.37049474453,6.07839454695,0.0),(0.0,0.0,-0.4126296528,0.91089889925)],
[(5.06797609782,4.80817749908,0.0),(0.0,0.0,-3.60056217465,0.932930608494)],
[(4.59565114414,4.33682222466,0.0),(0.0,0.0,-0.432249050867,0.901754266984)],
[(4.04112597261,3.8100783993,0.0),(0.0,0.0,-0.370100110207,0.928991877481)],
[(3.33647351165,3.02906368171,0.0),(0.0,0.0,-0.397563602617,0.917574619241)],
[(2.39813353871,2.38888917665,0.0),(0.0,0.0,-0.33222884791,0.943198808438)],
[(1.75380423421,1.75506863756,0.0),(0.0,0.0,-0.419478134598,0.907765440295)],
[(1.39538156006,0.984682087138,0.0),(0.0,0.0,-0.414802254225,0.909911583556)],
[(-0.00826481809515,-0.752769272729,0.0),(0.0,0.0,-0.457008500488,0.889462326623)],
[(-0.301846095219,-1.25765660547,0.0),(0.0,0.0,-0.377919074108,0.925838632498)]
]
gather_targets = False
logo_pose = None
logo_found = False
ar_pose = None
ar_found = False

def goal_pose_waypoint(wp, frame_id):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = frame_id
	goal_pose.target_pose.pose.position.x = wp[0][0]
	goal_pose.target_pose.pose.position.y = wp[0][1]
	goal_pose.target_pose.pose.position.z = wp[0][2]
	goal_pose.target_pose.pose.orientation.x = wp[1][0]
	goal_pose.target_pose.pose.orientation.y = wp[1][1]
	goal_pose.target_pose.pose.orientation.z = wp[1][2]
	goal_pose.target_pose.pose.orientation.w = wp[1][3]

	return goal_pose

def goal_pose(pose, frame_id):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = frame_id
	goal_pose.target_pose.pose.position.x = pose.position.x
	goal_pose.target_pose.pose.position.y = pose.position.y
	goal_pose.target_pose.pose.position.z = pose.position.z
	goal_pose.target_pose.pose.orientation.x = pose.orientation.x
	goal_pose.target_pose.pose.orientation.y = pose.orientation.y
	goal_pose.target_pose.pose.orientation.z = pose.orientation.z
	goal_pose.target_pose.pose.orientation.w = pose.orientation.w
	
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

def global_localize():
	rospy.wait_for_service('global_localization')
	localize_service = rospy.ServiceProxy('global_localization', Empty)

	try:
		res = localize_service()
	except:
		print("Service did not process request")

def move_direct(pose):
	tvec_target = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float32)
	#print("Moving to target")
	#print(tvec_target)

	rot_speed = 0.6
	move_speed = 0.2
	pause_period = 1

	# first, turn to face the target
        rot_radians  = -(np.arctan(tvec_target[0] / tvec_target[2]))
        rot_velocity = np.sign(rot_radians) * rot_speed
        rot_period   = rot_radians / rot_velocity
        rot_twist = Twist()
        rot_twist.angular.z = rot_velocity
        # second, move towards the checkerboard for .25m or within .1m
        move_meters = np.linalg.norm(tvec_target[np.array([0,2])])
        #print(np.linalg.norm(tvec_target[np.array([0,2])]))
        move_velocity = move_speed
        move_period   = move_meters / move_velocity
        move_twist = Twist()
        move_twist.linear.x = move_velocity

	_180_radians = math.pi*1.25
	#changed from rot_speed to 0.8
	_180_velocity = np.sign(_180_radians) * 2.4
	_180_period = _180_radians / _180_velocity
	_180_twist = Twist()
	_180_twist.angular.z = _180_velocity
	
	print("Rotation" + str(rot_twist))
	print("RDuration" + str(rot_period))
	print("Forward" + str(move_twist))
	print("PDuration" + str(move_period))

	twist_and_durations = [
		(rot_twist, rot_period),
		(move_twist, move_period),
		(Twist(), pause_period),
		(_180_twist, _180_period),
		(move_twist, move_period)]
	                       

	for t,d in twist_and_durations:
		start_time = rospy.get_rostime()
		duration = rospy.Duration.from_sec(d)
                #this is hacky light detection for pauses
		if(d == pause_period ):
			light_pub_1.publish(led.ORANGE)
		while start_time + duration > rospy.get_rostime():
			cmd_vel_pub.publish(t)

		light_pub_1.publish(led.BLACK)
		#######################################
		#back docking
		'''
		start_time = rospy.get_rostime()
		duration = rospy.Duration.from_sec(np.pi / rot_velocity)
		while start_time + duration > rospy.get_rostime():
			cmd_vel_pub.publish(rot_twist)
		move_twist.linear.x = - move_velocity
		'''
		#######################################
	
	cmd_vel_pub.publish(Twist())
	
	
def panic_mode():
        global alternator
	if(alternator == 1 ):
        	duration_seconds = 3.0
        	move_speed = -0.3
       		rot_speed = 0.6
		alternator = -alternator
	else:
        	duration_seconds = 3.0
        	move_speed = 0.3
       		rot_speed = -0.6
		alternator = -alternator

	start_time = rospy.get_rostime()
	duration = rospy.Duration.from_sec(duration_seconds)
	backup_twist = Twist()
	backup_twist.angular.z = rot_speed
        backup_twist.linear.x = move_speed
	while start_time + duration > rospy.get_rostime():
		cmd_vel_pub.publish(backup_twist)

if __name__ == '__main__':
	alternator = 1
	NUM_LAPS = 2
	SEARCH_PERIOD = 1
	#SEARCH_PERIOD = 2
	WAYPOINT_TF_FRAME = 'map'
	DOCKING_TF_FRAME = 'base_footprint'

	LOGO_SOUND = Sound()
	AR_SOUND = Sound()
	WAYPOINT_SOUND = Sound()
	LOGO_SOUND.value = 0
	AR_SOUND.value = 1
	WAYPOINT_SOUND.value = 3
        led = Led()

	sound_pub = rospy.Publisher('kobuki_sound', Sound, queue_size=1)
	light_pub_1 = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)
	light_pub_2 = rospy.Publisher('/mobile_base/commands/led2', Led, queue_size=1)
	logo_sub = rospy.Subscriber('logo_point', Pose, logo_pose_callback)
	ar_sub = rospy.Subscriber('ar_point', Pose, ar_pose_callback)
	cmd_vel_pub = rospy.Publisher('twist_out',Twist, queue_size=1)

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
	client.wait_for_server()
 
	lap_counter = 0
        fail_counter = 0

	light_pub_1.publish(led.RED)
	light_pub_2.publish(led.RED)
	global_localize()
	light_pub_1.publish(led.BLACK)
	light_pub_2.publish(led.BLACK)
	while (lap_counter < NUM_LAPS):
		print("Beginning lap " + str(lap_counter))
		i = 0
		while i < len(waypoints):
			wp = waypoints[i]

			#print("Moving to waypoint " + str(i))
			# move to next waypoint
			goal = goal_pose_waypoint(wp, WAYPOINT_TF_FRAME)
			client.send_goal(goal)
			#print("Sent waypoint")
			client.wait_for_result()

			#print("Processing result")
			if (client.get_state() == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
				#print("Success")
				light_pub_1.publish(led.BLACK)
				light_pub_2.publish(led.BLACK)
			else:
				light_pub_1.publish(led.RED)
				light_pub_2.publish(led.RED)
				panic_mode()
				#print("Failure, relocalizing")
				#global_localize()
				continue
			
			# play sound to indicate reaching of a waypoint
			light_pub_2.publish(led.GREEN)
			sound_pub.publish(WAYPOINT_SOUND)
			rospy.sleep(0.05)
			light_pub_2.publish(led.BLACK)

			# Look for targets for a duration of time
			gather_targets = True
			rospy.sleep(SEARCH_PERIOD)
			gather_targets = False
			if logo_found:
				sound_pub.publish(LOGO_SOUND)
				light_pub_1.publish(led.ORANGE)
				rospy.sleep(1)
				light_pub_1.publish(led.BLACK)
				if dock:
					print("Docking with logo")
					move_direct(logo_pose)
					#rospy.sleep(1)
				else:
					print("Found logo")
			elif ar_found:
				sound_pub.publish(AR_SOUND)
				light_pub_1.publish(led.ORANGE)
				rospy.sleep(1)
				light_pub_1.publish(led.BLACK)
				if dock:
					print("Docking with AR code")
					move_direct(ar_pose)
					#rospy.sleep(1)
				else:
					print("Found AR code")
			logo_found = False
			ar_found = False

			i = i + 1

		lap_counter = lap_counter + 1
	print("Completed all " + str(NUM_LAPS) + " laps")

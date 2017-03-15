#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


waypoints = [  
    [(3.6607125401, 7.31828998725, 0.0), (0.0, 0.0, -0.899002073698, 0.437944370311)],
    [(-0.481131523099, 2.63816824607, 0.0), (0.0, 0.0, -0.368556378916, 0.929605397768)],
    [(1.56175864561, 0.84328832173, 0.0), (0.0, 0.0, 0.484581645217, 0.874746036927)],
    [(5.8438633289, 5.37321792854, 0.0), (0.0, 0.0, 0.905211098608, 0.424962194739)]
]


def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


if __name__ == '__main__':
    rospy.init_node('patrol')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    client.wait_for_server()

    while True:
        for pose in waypoints:   
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()

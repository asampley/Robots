#!/usr/bin/env python

# import the necessary packages
import numpy as np
import argparse
import imutils
import glob
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

def RT2Pose(tvec, rvec):
        pose = Pose()
        pose.position.x = tvec[0]
        pose.position.y = tvec[1]
        pose.position.z = tvec[2]

        return pose


def image_callback(msg):
	global bridge

	# load the image, convert it to grayscale, and initialize the
	# bookkeeping variable to keep track of the matched region
	image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	found = None
 
	# loop over the scales of the image
	for scale in np.linspace(0.2, 1.0, 20)[::-1]:
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
		resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
		r = gray.shape[1] / float(resized.shape[1])

		# if the resized image is smaller than the template, then break
		# from the loop
		if resized.shape[0] < tH or resized.shape[1] < tW:
			#print("Image " + str(resized.shape) + " smaller than template " + str((tH, tW)))
			break
	
		# detect edges in the resized, grayscale image and apply template
		# matching to find the template in the image
		edged = cv2.Canny(resized, 50, 200)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF_NORMED)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
		print(maxVal)
	 
#		# draw a bounding box around the detected region
#		clone = np.dstack([edged, edged, edged])
#		cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
#			(maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
#		cv2.imshow("Visualize", clone)
#		cv2.waitKey(0)
	
		# if we have found a new maximum correlation value, then ipdate
		# the bookkeeping variable
		if found is None or maxVal > found[0]:
			found = (maxVal, maxLoc, r)
	
	# unpack the bookkeeping varaible and compute the (x, y) coordinates
	# of the bounding box based on the resized ratio
	#print(found)
	(maxVal, maxLoc, r) = found
	(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
	(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
	
	imgp = np.array([[startX,startY],[startX,endY],[endX,startY],[endX,endY]], dtype=np.float32)

	# draw a bounding box around the detected result and display the image
	if maxVal > 0.125:
		cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
		rvec, tvec, inliers = cv2.solvePnPRansac(objp, imgp, mtx, dist)
		# project target in front of logo
		rvec_target, tvec_target, _,_,_,_,_,_,_,_ = cv2.composeRT(np.array([0,0,0], dtype=np.float32), np.array([0,0,-0.3], dtype=np.float32), rvec, tvec)
		print("target" + str(tvec_target))
		print("rotation" + str(rvec_target))

		imgpt, jac = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvec_target, tvec_target, mtx, dist)
		cv2.circle(image, tuple(imgpt.ravel()), 10, (255, 0, 255), -1)
		
		logo_pose = RT2Pose(tvec_target, rvec_target)
		logo_pose_pub.publish(logo_pose)
	cv2.imshow("Image", image)
	cv2.waitKey(1)

mtx = np.array([[540.408778, 0.000000, 324.541383], [0.000000, 544.021453, 250.415376], [0.000000, 0.000000, 1.000000]], dtype=np.float64)
dist = np.array([0.047539, -0.151056, 0.008007, 0.002056, 0.000000], dtype=np.float64)

axis = np.float32([[0,0,0], [.1,0,0], [0,.1,0], [0,0,.1]]).reshape(-1,3)
objp = 0.2 * np.array([[-0.5,-0.647,0],[-0.5,0.647,0],[0.5,-0.647,0],[0.5,0.647,0]], dtype=np.float32)

bridge = cv_bridge.CvBridge()

# load the image image, convert it to grayscale, and detect edges
template = cv2.imread('uofa.png')
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 50, 200)
(tH, tW) = template.shape[:2]
cv2.imshow("Template", template)

rospy.init_node('logo_template_match')

rospy.Subscriber('image', Image, image_callback)
logo_pose_pub = rospy.Publisher('logo_pose', Pose, queue_size = 1)
rospy.spin()


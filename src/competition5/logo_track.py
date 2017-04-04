#!/usr/bin/env python

import numpy as np
import cv2
import cv_bridge
import rospy
import math
from draw_matches import draw_matches
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

mtx = np.array([[540.408778, 0.000000, 324.541383], [0.000000, 544.021453, 250.415376], [0.000000, 0.000000, 1.000000]], dtype=np.float64)
dist = np.array([0.047539, -0.151056, 0.008007, 0.002056, 0.000000], dtype=np.float64)

def draw(img, imgpts):
    corner = tuple(imgpts[0].ravel())
    cv2.line(img, corner, tuple(imgpts[1].ravel()), (255,0,0), 5)
    cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,255,0), 5)
    cv2.line(img, corner, tuple(imgpts[3].ravel()), (0,0,255), 5)
    return img

axis = np.float32([[0,0,0], [.1,0,0], [0,.1,0], [0,0,.1]]).reshape(-1,3)
cv2.namedWindow('img', 1)
cv2.waitKey(1)
bridge = cv_bridge.CvBridge()

img1 = cv2.imread('uofa.png', 0)          # queryImage
print(img1.shape)
cv2.waitKey(1000)
orb = cv2.ORB()
kp1 = orb.detect(img1,None)
kp1, des1 = orb.compute(img1,kp1)

def RT2Pose(tvec, rvec):
	pose = Pose()
	pose.position.x = tvec[0]
	pose.position.y = tvec[1]
	pose.position.z = tvec[2]

	return pose

def image_callback(msg):
  global img1, orb, kp1, des1, mtx, dist, bridge, axis, logo_pose_pub

  found=True
  tvec=np.zeros((1,3))
  rvec=np.zeros((1,3))
#  print(msg)

  img2 = bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')
  img2 = img2.squeeze(axis=2)
#  print(img1.shape)
#  print(img2.shape)

  # find the keypoints and descriptors with ORB
  kp2 = orb.detect(img2,None)
  kp2, des2 = orb.compute(img2,kp2)

#  FLANN_INDEX_LSH = 6
#  index_params = {"algorithm" : FLANN_INDEX_LSH,\
#                  "table_number" : 30,\
#                  "key_size" : 20,\
#                  "multi_probe_level" : 2}
#  search_params = {"checks" : 1000}
#  flann = cv2.FlannBasedMatcher(index_params, search_params) 
#  matches = flann.knnMatch(des1,des2,k=2)

  bf = cv2.BFMatcher(cv2.NORM_HAMMING)
  matches = bf.knnMatch(des1, des2, 2)

  # store all the good matches as per Lowe's ratio test.
  good = []
  for i in range(len(matches)):
    match = matches[i]
    if len(match) == 2:
      m,n = match
      if m.distance < 0.5*n.distance:
        good.append(m)

#  # use threshold distance and cross check instead of ratio test
#  bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#  matches = bf.match(des1, des2)
#  
#  distances = [m.distance for m in matches]
#  thres_dist = (sum(distances) / len(distances)) * 0.7
#  good = [m for m in matches if m.distance < thres_dist]

#  # sort and trim to at most 20 good keypoints
#  good.sort(key=lambda match: match.distance)
#  good = good[:min(len(good)-1, 20)]

  if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    # make 3D definition of object centered and scaled based on 20cm wide logo
    objp = np.float32(np.append(0.2 / img1.shape[1] * (src_pts - img1.shape[1] / 2), np.zeros((src_pts.shape[0], src_pts.shape[1], 1)), axis=2)) 
    
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    
    matchesMask = mask.ravel().tolist()

    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    cv2.polylines(img2,[np.int32(dst)],True,255,3)

    rvec, tvec, inliers = cv2.solvePnPRansac(objp, dst_pts, mtx, dist)
#    print("Position: " + str(tvec))
#    print("Rotation: " + str(rvec))

    # project target in front of logo
    rvec_target, tvec_target, _,_,_,_,_,_,_,_ = cv2.composeRT(np.array([0,0,0], dtype=np.float32), np.array([0,0,-0.3], dtype=np.float32), rvec, tvec)
    print("target" + str(tvec_target))
    print("rotation" + str(rvec_target))

    # reduce false positives by removing those that are far from straight at the camera
    target_good = np.linalg.norm(rvec_target) < math.pi / 4

    if target_good:
      
      # publish target
      logo_pose = RT2Pose(tvec_target, rvec_target)
      logo_pose_pub.publish(logo_pose)

      imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
      img4 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
      draw(img4,imgpts)
    
      imgpt, jac = cv2.projectPoints(np.array([[0,0,0]], dtype=np.float32), rvec_target, tvec_target, mtx, dist)
      cv2.circle(img4, tuple(imgpt.ravel()), 10, (255, 0, 255), -1)
    
      cv2.imshow('axes', img4)
    maskedMatches = [good[i] for i in range(len(good)) if matchesMask[i] == 1]
    
    #print(matchesMask)
    #print(maskedMatches)
    img3 = draw_matches(img1,kp1,img2,kp2,maskedMatches,color=255)
  else:
    #print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None
    img3 = draw_matches(img1,kp1,img2,kp2,good,color=127)

  #draw_params = dict(matchColor = (0,255,0), # draw matches in green color
  #                 singlePointColor = None,
  #                 matchesMask = matchesMask, # draw only inliers
  #                 flags = 2)

MIN_MATCH_COUNT = 10

rospy.init_node('axis_drawer')
rospy.Subscriber('image', Image, image_callback)
logo_pose_pub = rospy.Publisher('logo_pose', Pose, queue_size = 1)
rospy.spin()

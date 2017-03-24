#!/usr/bin/env python

import numpy as np
import cv2
import cv_bridge
import rospy
from draw_matches import draw_matches
from matplotlib import pyplot as plt
from sensor_msgs.msg import Image

mtx = np.array([[612.372615, 0.000000, 319.855223], [0.000000, 611.047376, 245.946766], [0.000000, 0.000000, 1.000000]], dtype=np.float64)
dist = np.array([-0.027223, 0.098406, 0.002497, -0.001910, 0.000000], dtype=np.float64)

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
print(img1)
cv2.waitKey(1000)
orb = cv2.ORB()
kp1 = orb.detect(img1,None)
kp1, des1 = orb.compute(img1,kp1)

def image_callback(msg):
  global img1, orb, kp1, des1, mtx, dist, bridge, axis

  found=True
  tvec=np.zeros((1,3))
  rvec=np.zeros((1,3))
#  print(msg)

  img2 = bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')
  img2 = img2.squeeze(axis=2)
  print(img1.shape)
  print(img2.shape)

  # find the keypoints and descriptors with SIFT
  kp2 = orb.detect(img2,None)
  kp2, des2 = orb.compute(img2,kp2)

  #FLANN_INDEX_KDTREE = 0
  #index_params = {"algorithm" : FLANN_INDEX_KDTREE, "trees" : 5}
  #search_params = {"checks" : 50}
  
  #flann = FlannBasedMatcher(index_params, search_params)
  #
  #matches = flann.knnMatch(des1,des2,k=2)

  bf = cv2.BFMatcher(cv2.NORM_HAMMING)
  matches = bf.knnMatch(des1, des2, 2)

  # store all the good matches as per Lowe's ratio test.
  good = []
  for m,n in matches:
    if m.distance < 0.7*n.distance:
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

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()

    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    cv2.polylines(img2,[np.int32(dst)],True,255,3)

    maskedMatches = [good[i] for i in range(len(good)) if matchesMask == 1]

    img3 = draw_matches(img1,kp1,img2,kp2,maskedMatches,color=255)
  else:
    #print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None
    img3 = draw_matches(img1,kp1,img2,kp2,good,color=127)

  #draw_params = dict(matchColor = (0,255,0), # draw matches in green color
  #                 singlePointColor = None,
  #                 matchesMask = matchesMask, # draw only inliers
  #                 flags = 2)

  #print(img1)
  #print(img2)
  #plt.imshow(img3, 'gray'),plt.show()
  
  if found:
  # project 3D points to image plane
    imgpts, jac = cv2.projectPoints(axis, rvec, tvec, mtx, dist)
    print("Position: " + str(tvec))
    print("Rotation: " + str(rvec))
    print(imgpts)
    draw(img2, imgpts)

MIN_MATCH_COUNT = 10

rospy.init_node('axis_drawer')
rospy.Subscriber('image', Image, image_callback)
rospy.spin()
# Initiate SIFT detector

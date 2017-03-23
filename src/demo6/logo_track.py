#!/usr/bin/env python

import numpy as np
import cv2
from draw_matches import draw_matches
from matplotlib import pyplot as plt

MIN_MATCH_COUNT = 10

img1 = cv2.imread('uofa.png',0)          # queryImage
img2 = cv2.imread('uofa2.png',0)         # trainImage

print(type(img1))
print(type(img2))
# Initiate SIFT detector
orb = cv2.ORB()

# find the keypoints and descriptors with SIFT
kp1 = orb.detect(img1,None)
kp1, des1 = orb.compute(img1,kp1)
kp2 = orb.detect(img2,None)
kp2, des2 = orb.compute(img2,kp2)

#FLANN_INDEX_KDTREE = np.array([0], dtype=np.float32)
#index_params = {"algorithm" : FLANN_INDEX_KDTREE, "trees" : 5}
#search_params = {"checks" : 50}
#
#flann = cv2.FlannBasedMatcher(index_params, search_params)
#
#matches = flann.knnMatch(des1,des2,k=2)

bf = cv2.BFMatcher(cv2.NORM_HAMMING)
matches = bf.knnMatch(des1, des2, 2)

# store all the good matches as per Lowe's ratio test.
good = []
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

if len(good)>MIN_MATCH_COUNT:
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    matchesMask = mask.ravel().tolist()

    h,w = img1.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)

    cv2.polylines(img2,[np.int32(dst)],True,255,3)

else:
    print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    matchesMask = None

draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

img3 = draw_matches(img1,kp1,img2,kp2,good)
#plt.imshow(img3, 'gray'),plt.show()

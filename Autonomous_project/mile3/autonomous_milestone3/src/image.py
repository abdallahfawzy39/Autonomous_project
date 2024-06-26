#!/usr/bin/env python3

# import the required libraries:
import rospy
import numpy as np 
import cv2 #import openCV to python


# Save image in set directory 
img = cv2.imread('/home/dell/catkin_ws/src/autonomous_milestone3/map.png')  #Read image 
grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #Convert RGB image to grayscale
ret, bw_img = cv2.threshold(grayImage,0,1,cv2.THRESH_BINARY) #Convert grayscale image to binary
print(type(bw_img))
bw_img = bw_img.astype(np.uint8)
cv2.imshow("Window", bw_img)

h, w= bw_img.shape #get image dimenssions
print('height:', h)
print('width:', w)
print(bw_img)


cv2.waitKey(0) #Maintain output window until user presses a key 
cv2.destroyAllWindows() #Destroying present windows on screen


# -*- coding: utf-8 -*-
"""
Created on Thu Sep 19 16:50:04 2019

@author: BOBY SINGH
"""

#video using webcam and communication between pc and arduino using pyserial lib.
import numpy as np
import cv2
cap=cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FPS,72)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(w,h)
while(True):
    ret,frame=cap.read()
    img = np.array(frame)
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
 
    # Range for lower red
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
 
    # Range for upper range
    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask2 = cv2.inRange(hsv,lower_red,upper_red)
    
    # Generating the final mask to detect red color
    mask1 = mask1+mask2
    
    image, contours, hierarchy = cv2.findContours(mask1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   
    cv2.drawContours(frame,contours, -1, (0,255,0), 2)    
        
    cv2.imshow('Original',frame)
    cv2.imshow('Thresh',mask1)
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
        break
    
cap.release()
cv2.destroyAllWindows()

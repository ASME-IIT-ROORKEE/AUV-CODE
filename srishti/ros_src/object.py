#!/usr/bin/python
 
import rospy
import math
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
 
def cameraTest():
    rospy.init_node('position',anonymous=True)
    pub = rospy.Publisher('value',Int32, queue_size=1000)
    pub1 = rospy.Publisher('image', Image,queue_size=1000)
    pub2 = rospy.Publisher('orig_image', Image,queue_size=1000)
    rate = rospy.Rate(10)
    x = 0
    y = 0
    #cap = cv2.VideoCapture('/home/asmeiitr/Downloads/Underwater_ball_detection-master/outputk.avi')
    cap = cv2.VideoCapture(0)
    br = CvBridge()
    #a = time.time()
    # Read until video is completed
    while (cap.isOpened() and not(rospy.is_shutdown())):
      # Capture frame-by-frame
      ret, img = cap.read()
      if ret == True:
 
        im = np.zeros((img.shape[0],img.shape[1]),dtype = int)
        im = 0.6*img[:,:,0] + 0.2*img[:,:,1] + 0.2*img[:,:,2]
        im = cv2.normalize(im.astype(int), None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)
        #applying sharpnening
        kernel_sharpening = np.array([[-1,-1,-1], 
                                  [-1, 9,-1],
                                  [-1,-1,-1]])
 
        sharpened = cv2.filter2D(im, -1, kernel_sharpening)
        #applying blur
        img3 = cv2.medianBlur(sharpened,5)
        img3 = cv2.medianBlur(img3,15,3)
 
        img3 = cv2.Canny(img3,70,70)
        #detecting circles
        circles = cv2.HoughCircles(img3, cv2.HOUGH_GRADIENT, 2,1800)
 
        if circles is None:
            continue
        c = circles[0]
        x=circles[0][0][0]
        y=circles[0][0][1]
 
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            #draw the outer circle
            cv2.circle(img3,(i[0],i[1]),i[2],(255,255,255),5)
            #draw the center of the circle
            cv2.circle(img3,(i[0],i[1]),2,(255,255,255),3)
            #cv2.imshow('result',img3) 
          #if time.time()-a > 10 :
#             break
	pub1.publish(br.cv2_to_imgmsg(img3))
	pub2.publish(br.cv2_to_imgmsg(im))	
	rospy.loginfo("Coordinate: %s", x)
        pub.publish(x)
        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
	try:
	    cameraTest()
	except rospy.ROSInterruptException:
	    pass

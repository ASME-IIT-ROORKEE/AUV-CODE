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
 
def linefollow():
    rospy.init_node('Tub_with-line',anonymous=True)
    pub = rospy.Publisher('Line_following_error_tub',Int32, queue_size=1000)
    pub1 = rospy.Publisher('tub-status',Int32, queue_size=1000)
    pub2 = rospy.Publisher('line_image', Image,queue_size=1000)
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(0)
    br = CvBridge()

    while(cap.isOpened()):
      # Capture frame-by-frame
        ret, img = cap.read()
        if ret == True:
            im = np.zeros((img.shape[0],img.shape[1]),dtype = int)
            im = 0.6*img[:,:,0] + 0.2*img[:,:,1] + 0.2*img[:,:,2]
            im = cv2.normalize(im.astype(int), None, 255,0, cv2.NORM_MINMAX, cv2.CV_8UC1)
            #applying sharpnening
            kernel_sharpening =  np.array([[-1,-1,-1], 
                                           [-1, 9,-1],
                                           [-1,-1,-1]])
        
            sharpened = cv2.filter2D(im, -1, kernel_sharpening)
            #applying blur
            img3 = cv2.medianBlur(sharpened,5)
            ret, threshed_img = cv2.threshold(img3,40, 255, cv2.THRESH_BINARY)
            width = threshed_img.shape[0]
            height = threshed_img.shape[1]

            img5 = threshed_img[0:height/4,0:width]
            img5 = cv2.dilate(img5,(9,9),iterations=2)
            img5 = 255-img5
	   

            #cv2.imshow("contours",img5)

            contours, hierarchy = cv2.findContours(img5,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            areas = [cv2.contourArea(c) for c in contours]
            if len(areas) > 0 :
		#for area in areas:
		#	print(area)                
		max_index= np.argmax(areas)
		#print (areas[max_index])
                contours1 = contours[max_index]
		x1,y1,w1,h1 = cv2.boundingRect(contours1)
		print("x1 ")
		print(x1)
		print("y1 ")
		print(y1)
		print("w1 ")
		print(w1)
		print("h1 ")
		print(h1)    
    
           # if len(contours1) > 0 :
              
             #print(x1,y1,w1,h1)
                if (x1 >= 0 and y1>=0 and w1 >=0 and h1>=0 ):
             #frame =cv2.line(frame, (x1+int(w1/2), 200), (x1+int(w1/2), 250),(255,0,0),3)
                    frame =cv2.rectangle(img,(int(x1),int(y1)),(int(x1+w1), int(y1+h1)),(0,255,255),5)
            # compute the center of the contour
            M = cv2.moments(contours1)
	    if(M["m00"]):
            	    cX = int(M["m10"] / M["m00"])           
		    cY = int(M["m01"] / M["m00"])
		    cv2.drawContours(frame,contours1, -1, (0,255,0), 2)
		    frame = cv2.circle(frame, (cX,cY), 10, (255,255,255),3) 
		    cv2.imshow("contours1",frame)
		    err=cX-(width/2)
		    print(err)
		    imgb=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		    blue_lower=np.array([100,150,0],np.uint8)
		    blue_upper=np.array([140,255,255],np.uint8)
		    blue=cv2.inRange(imgb,blue_lower,blue_upper)
		    white=cv2.countNonZero(blue)
		    #black=area-cv2.countNonZero(blue)

		    #print(white)
		    #print(black)
		    total=blue.size
		    #print(blue.size)
		    status = 0
		    if(white>=0.3*total):
		        status = 1
		    print(status)
		    pub2.publish(br.cv2_to_imgmsg(frame))
		    rospy.loginfo("Coordinate: %s", err)
		    pub.publish(err)
		    pub1.publish(status)
		    rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
	try:
	    linefollow()
	except rospy.ROSInterruptException:
	    pass


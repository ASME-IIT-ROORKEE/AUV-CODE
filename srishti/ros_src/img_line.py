#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
def callback1(data):
	br1 = CvBridge()
	rospy.loginfo('receiving image1')
	cv2.imshow("camera1",br1.imgmsg_to_cv2(data))
	cv2.waitKey(1)
 
def callback2(data):
	br2 = CvBridge()
	rospy.loginfo('receiving image2')
	cv2.imshow("camera2",br2.imgmsg_to_cv2(data))
	cv2.waitKey(1)
 
def listener():
	rospy.init_node('listener',anonymous=True)
	rospy.Subscriber('line_image',Image, callback1)
	#rospy.Subscriber('orig_image',Image, callback2)
	rospy.spin()
	cv2.destroyAllWindows()
if __name__ == '__main__':
	listener()

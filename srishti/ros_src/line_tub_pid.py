#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32

prev_err =0 
#lthrust=1500
#rthrust=1500
x=0
y=0
err=0

def callback1(data1):
    global x
    x = data1.data
def callback2(data2):
    global y
    y = data2.data
def callback3(data3):
    global z
    err = data3.data
    print(err)
    operation()    

def operation():
	global prev_err
	global x
	global y
	global err

	derr = err - prev_err
	prev_err = err
	corr = int((x * err) + (y * derr))
	print(corr)


	rate = rospy.Rate(10)
	pub_pwm1=rospy.Publisher('PWM1i', Int32, queue_size=10)
	pub_pwm2=rospy.Publisher('PWM2i', Int32, queue_size=10)
	pub_pwm3=rospy.Publisher('PWM3i', Int32, queue_size=10)
	pub_pwm4=rospy.Publisher('PWM4i', Int32, queue_size=10)
	pub_pwm5=rospy.Publisher('PWM5i', Int32, queue_size=10)
	pub_pwm6=rospy.Publisher('PWM6i', Int32, queue_size=10)


	pwm1=1386+corr    #right
	pwm2=1608+corr
	pwm3=1500
	pwm4=1500
	pwm5=1500
	pwm6=1500
	pub_pwm1.publish(pwm1)
	pub_pwm2.publish(pwm2)
	pub_pwm3.publish(pwm3)
	pub_pwm4.publish(pwm4)
	pub_pwm5.publish(pwm5)
	pub_pwm6.publish(pwm6)
	
def initialize():
    rospy.init_node("line_pid", anonymous = True)
    kp = rospy.Subscriber("Kp", Float32, callback1) 
    kd = rospy.Subscriber("Kd", Float32, callback2) 
    error = rospy.Subscriber("Line_following_error", Int32, callback3)
    rospy.spin()
   # rate.sleep()
if __name__ == '__main__':
    initialize()

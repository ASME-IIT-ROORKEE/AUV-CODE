#!/usr/bin/python
from Tkinter import *
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64

if __name__ == '__main__':
    try:
		
	def forward():
   		x = 1
		motion.publish(x)
	def backward():
   		x = 2
		motion.publish(x)
	def left():
   		x = 3
		motion.publish(x)
	def right():
   		x = 4
		motion.publish(x)
	def stop():
   		x = 0
		motion.publish(x)	
	def move_pwm():
   		x = 5
		motion.publish(x)
	def callback(msg):
		print msg.data
	
	master = Tk() 
	master.wm_title("AUV GUI")
	motion = rospy.Publisher('/motion', Int32, queue_size=1000)
	rospy.init_node('remote_gui', anonymous=True)

	label = Label(master, bd=5,fg="blue", anchor=N, text="AUV")
	label.pack()
	l1 = Label(master, fg="green", text="Remote Controller").pack()
	
	B1 = Button(master, text ="Front", command = forward).pack()
	B2 = Button(master, text ="Back", command = backward).pack()
	B3 = Button(master, text ="Left", command = left).pack()
	B4 = Button(master, text ="Right", command = right).pack()
	B5 = Button(master, text ="Stop", fg="yellow", bg="red",command = stop).pack()
	B6 = Button(master, text ="MOVE", command = move_pwm).pack()

########################################################
	
	
	master.mainloop()
	
    except rospy.ROSInterruptException:
        pass

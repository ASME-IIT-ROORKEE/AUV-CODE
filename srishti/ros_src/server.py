#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server
from rov_gui.cfg import TutorialsConfig

def callback(config, level):
    #rospy.loginfo("""{Depth_SetPoint},{Kp},{Kd},{Ki}""".format(**config))
    Depth_SP = """{Depth_SetPoint}""".format(**config)
    Yaw_SP = """{Yaw_SetPoint}""".format(**config)
    KP = """{Kp}""".format(**config)
    KD = """{Kd}""".format(**config)
    KI = """{Ki}""".format(**config)
    PWM1 = """{PWM1}""".format(**config)
    PWM2 = """{PWM2}""".format(**config)
    PWM3 = """{PWM3}""".format(**config)
    PWM4 = """{PWM4}""".format(**config)
    PWM5 = """{PWM5}""".format(**config)
    PWM6 = """{PWM6}""".format(**config)
    int_dep = int(Depth_SP)
    int_yaw = int(Yaw_SP) 
    flo_kp = float(KP)
    flo_kd = float(KD)
    flo_ki = float(KI)
    int_PWM1 = int(PWM1)
    int_PWM2 = int(PWM2)
    int_PWM3 = int(PWM3)
    int_PWM4 = int(PWM4)
    int_PWM5 = int(PWM5)
    int_PWM6 = int(PWM6) 
    pub_depth.publish(int_dep)
    pub_yaw.publish(int_yaw)
    pub_kp.publish(flo_kp)
    pub_kd.publish(flo_kd)
    pub_ki.publish(flo_ki)
    pub_pwm1.publish(int_PWM1)
    pub_pwm2.publish(int_PWM2)
    pub_pwm3.publish(int_PWM3)
    pub_pwm4.publish(int_PWM4)
    pub_pwm5.publish(int_PWM5)
    pub_pwm6.publish(int_PWM6)
    print("Yaw,Depth,Kp,Kd,Ki Published")
    return config

if __name__ == "__main__":
    rospy.init_node("server", anonymous = False)
    pub_depth=rospy.Publisher('SetPoint_depth', Int32, queue_size=10)
    pub_yaw=rospy.Publisher('SetPoint_yaw', Int32, queue_size=10)
    pub_kp=rospy.Publisher('Kp', Float32, queue_size=10)
    pub_kd=rospy.Publisher('Kd', Float32, queue_size=10)
    pub_ki=rospy.Publisher('Ki', Float32, queue_size=10)
    pub_pwm1=rospy.Publisher('PWM1', Int32, queue_size=10)
    pub_pwm2=rospy.Publisher('PWM2', Int32, queue_size=10)
    pub_pwm3=rospy.Publisher('PWM3', Int32, queue_size=10)
    pub_pwm4=rospy.Publisher('PWM4', Int32, queue_size=10)
    pub_pwm5=rospy.Publisher('PWM5', Int32, queue_size=10)
    pub_pwm6=rospy.Publisher('PWM6', Int32, queue_size=10)
    #rate= rospy.Rate(5)
    srv = Server(TutorialsConfig, callback)
    rospy.spin()

/*
  Code written by Ashish 3rd Feb 2020
 */
#include <Servo.h>


#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
byte servoPinRight = 2;
byte servoPinLeft = 3;
byte servoPinTopRight = 4;
byte servoPinTopLeft = 5;
byte servoPinTopBottom = 6;
byte servoPinSide = 7;
Servo servoRight;
Servo servoLeft;
Servo servoTopRight;
Servo servoTopLeft;
Servo servoTopBottom;
Servo servoSide;
 
ros::NodeHandle nh;

int drxn;     //subscribe
int pwm1,pwm2,pwm3,pwm4,pwm5,pwm6;
void message(const std_msgs::Int32 &msg)
{
  drxn=msg.data;
}
void message1(const std_msgs::Int32 &msg1)
{
  pwm1=msg1.data;
}
void message2(const std_msgs::Int32 &msg2)
{
  pwm2=msg2.data;
}
void message3(const std_msgs::Int32 &msg3)
{
  pwm3=msg3.data;
}
void message4(const std_msgs::Int32 &msg4)
{
  pwm4=msg4.data;
}
void message5(const std_msgs::Int32 &msg5)
{
  pwm5=msg5.data;
}
void message6(const std_msgs::Int32 &msg6)
{
  pwm6=msg6.data;
}


ros::Subscriber<std_msgs::Int32> sub1("motion", &message);

ros::Subscriber<std_msgs::Int32> pwm_s1("PWM1", &message1);
ros::Subscriber<std_msgs::Int32> pwm_s2("PWM2", &message2);
ros::Subscriber<std_msgs::Int32> pwm_s3("PWM3", &message3);
ros::Subscriber<std_msgs::Int32> pwm_s4("PWM4", &message4);
ros::Subscriber<std_msgs::Int32> pwm_s5("PWM5", &message5);
ros::Subscriber<std_msgs::Int32> pwm_s6("PWM6", &message6);



void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);  
  nh.subscribe(pwm_s1);  
  nh.subscribe(pwm_s2);  
  nh.subscribe(pwm_s3);  
  nh.subscribe(pwm_s4);  
  nh.subscribe(pwm_s5);  
  nh.subscribe(pwm_s6);
  servoRight.attach(servoPinRight);
  servoLeft.attach(servoPinLeft);
  servoTopRight.attach(servoPinTopRight);
  servoTopLeft.attach(servoPinTopLeft);
  servoTopBottom.attach(servoPinTopBottom);
  servoSide.attach(servoPinSide);
  servoRight.writeMicroseconds(1500); // send "stop" signal to ESC.</p>
  servoLeft.writeMicroseconds(1500);
  servoTopRight.writeMicroseconds(1500);
  servoTopLeft.writeMicroseconds(1500);
  servoTopBottom.writeMicroseconds(1500);
  servoSide.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal<br />
}
void acc_pwm()
{servoRight.writeMicroseconds(pwm1); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(pwm2);
 servoTopRight.writeMicroseconds(pwm3);
 servoTopLeft.writeMicroseconds(pwm4);
 servoTopBottom.writeMicroseconds(pwm5);
 servoSide.writeMicroseconds(pwm6);
  
}
void forward()
{
 servoRight.writeMicroseconds(1600); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(1600);
 servoTopRight.writeMicroseconds(1550);
 servoTopLeft.writeMicroseconds(1550);
 servoTopBottom.writeMicroseconds(1550);
 servoSide.writeMicroseconds(1500);
}
void backward(){
 servoRight.writeMicroseconds(1400); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(1400);
 servoTopRight.writeMicroseconds(1550);
 servoTopLeft.writeMicroseconds(1550);
 servoTopBottom.writeMicroseconds(1550);
 servoSide.writeMicroseconds(1500);
  }
   void left(){
 servoSide.writeMicroseconds(1550); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(1500);
 servoTopRight.writeMicroseconds(1550);
 servoTopLeft.writeMicroseconds(1550);
 servoTopBottom.writeMicroseconds(1550);
 servoRight.writeMicroseconds(1500);
    }
 void right(){
      servoSide.writeMicroseconds(1450); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(1500);
 servoTopRight.writeMicroseconds(1550);
 servoTopLeft.writeMicroseconds(1550);
 servoTopBottom.writeMicroseconds(1550);
 servoRight.writeMicroseconds(1500);
      
      }
 void stop_it(){
  servoSide.writeMicroseconds(1500); // Send signal to ESC.<br />
 servoLeft.writeMicroseconds(1500);
 servoTopRight.writeMicroseconds(1500);
 servoTopLeft.writeMicroseconds(1500);
 servoTopBottom.writeMicroseconds(1500);
 servoRight.writeMicroseconds(1500);
  }
 
 
void loop()
{if(drxn==0)
{stop_it();}
else if(drxn==1)
{forward();}
else if(drxn==2)
{backward();
}
else if(drxn==3)
{left();}
else if(drxn==4)
{right();}
else if(drxn == 5)
   {
    acc_pwm();
     //move acc to pwm value
     
   }
   nh.spinOnce();

}

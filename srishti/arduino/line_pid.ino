#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Servo.h>

ros::NodeHandle nh;

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
 
float kp=10;
float kd=8;
int err=0;
int prev_err =0;
int derr=0 ;
int pwm2=1600;
int pwm1=1400; 
int pwm3=1500;
int pwm4=1500;
int pwm5=1500;
int pwm6=1500;
int corr =0;


void messageCb1(const std_msgs::Float32 &msg)
{
  kp=msg.data;
}
 
void messageCb2(const std_msgs::Float32 &msg)
{
  kd=msg.data;
}
 
 void messageCb3(const std_msgs::Int32 &msg)
{
  err=msg.data;
}
 
ros::Subscriber<std_msgs::Float32> sub1("Kp", &messageCb1);
ros::Subscriber<std_msgs::Float32> sub2("Kd", &messageCb2);
ros::Subscriber<std_msgs::Int32> sub3("Line_following_error_tub", &messageCb3);

std_msgs::Int32 correction;
ros::Publisher pub1("/correction", &correction);

void setup()
{//Serial.begin(9600);
  
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(pub1);
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
  delay(7000);
   
}


void loop()
{    
derr = err - prev_err;
prev_err = err;
corr = ((kp * err) + (kd * derr));
corr = map(corr, -800,800,-85,85);
pwm1=1386-corr; //right 
pwm2=1608-corr;  //left
pwm3=1500;
pwm4=1500;
pwm5=1500;
pwm6=1500;
correction.data = corr;
servoRight.writeMicroseconds(pwm1);
  servoLeft.writeMicroseconds(pwm2);
  servoTopRight.writeMicroseconds(pwm3);
  servoTopLeft.writeMicroseconds(pwm4);
  servoTopBottom.writeMicroseconds(pwm5);
  servoSide.writeMicroseconds(pwm6);
//Serial.print(pwm1); 
 pub1.publish(&correction);
  nh.spinOnce();
}


//HOLI
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

#include <Wire.h>
#include "MS5837.h"
MS5837 sensor;


int depth,signalstable_tb,signalstable_tl,signalstable_tr;
float error,prev_error=0,correction,derrivative;

int value,gati_tb,gati_tr,gati_tl;
float kp,kd,ki;

byte servoPinRight = 9;
byte servoPinLeft = 10;
byte servoPinTopRight = 11;
byte servoPinTopLeft = 12;
byte servoPinTopBottom = 13;
byte servoPinSide = 14;

Servo servoRight;
Servo servoLeft;
Servo servoTopRight;
Servo servoTopLeft;
Servo servoTopBottom;
Servo servoSide;
 
ros::NodeHandle nh;

void message(const std_msgs::Int32 &msg)
{
  value=msg.data;
}
void message1(const std_msgs::Int32 &msg1)
{
  kp=msg1.data;
}
void message2(const std_msgs::Int32 &msg2)
{
  kd=msg2.data;
}
void message3(const std_msgs::Int32 &msg3)
{
  ki=msg3.data;
}

ros::Subscriber<std_msgs::Int32> sub1("SetPoint_depth", &message);
ros::Subscriber<std_msgs::Float32> sub2("Kp", &message1);
ros::Subscriber<std_msgs::Float32> sub3("Kd", &message2);
ros::Subscriber<std_msgs::Float32> sub4("Ki", &message3);

void setup() {
  nh.initNode();
  nh.subscribe(sub1);  
  nh.subscribe(sub2);  
  nh.subscribe(sub3);  
  nh.subscribe(sub4);  
  
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
  
  
  Wire.begin();
  
  while (!sensor.init()) {
    //Serial.println("Init failed!");
    //Serial.println("Are SDA/SCL connected correctly?");
    //Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    //Serial.println("\n\n\n");
    delay(2000);
  }
  
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);
  
}



void loop() {

 
  sensor.read();

  //Serial.print("Depth: "); 
   depth=(sensor.depth())*100;
  //Serial.print(depth); 
  //Serial.println(" m ");
  
  calculate_pid();
  balance();
  
//  Serial.print("Pressure: "); 
//  Serial.print(sensor.pressure()); 
//  Serial.print(" mbar  ");
  
//  Serial.print("Temperature: "); 
//  Serial.print(sensor.temperature()); 
//  Serial.println(" deg C");
  
//  Serial.print("Altitude: "); 
//  Serial.print(sensor.altitude()); 
//  Serial.println(" m above mean sea level");
 nh.spinOnce();
}

void balance()
{
  if(correction<0){
  gati_tb = map(correction, -1,0,1200,signalstable);
  gati_tl = map(correction, -1,0,1800,signalstable);
  gati_tr = map(correction, -1,0,1800,signalstable);
  }
 else{
   gati_tb = signalstable_tb;
   gati_tl = signalstable_tl;
   gati_tr = signalstable_tr;
  }
 servoTopRight.writeMicroseconds(gati_tr);
 servoTopLeft.writeMicroseconds(gati_tl);
 servoTopBottom.writeMicroseconds(gati_tb);
}

void calculate_pid()
{
error = (sensor.depth()*100) - value; 
derrivative = error - prev_error;
correction = (kp*error)+(kd*derrivative);
prev_error = error;
}

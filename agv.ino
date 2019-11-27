/*     Arduino Rotary Encoder Tutorial
 *      
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.04;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;


 #define outputA 8
 #define outputB 9
 #define outputC 22
 #define outputD 23
 int mr1 = 3;
 int mr2 = 4;
 int ml1 = 5;
 int ml2 = 6;
 int mre = 2; 
 int mle = 7;
 int counter1 = 0,counter2 = 0;; 
 int a1State,a2State;
 int aLastState1,aLastState2;  
 void setup() { 

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
   pinMode (outputA,INPUT);
   pinMode (outputB,INPUT);
   
   pinMode (outputC,INPUT);
   pinMode (outputD,INPUT);
   
   Serial.begin (115200);
   // Reads the initial state of the outputA
   aLastState1 = digitalRead(outputA);
   aLastState2 = digitalRead(outputC);   
 } 
 void loop() { 
  imu();
  encoder1();
  encoder2();
  run();
 }

 void imu(){
       timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();


  yaw = yaw + norm.ZAxis * timeStep;

  // Output  
  Serial.print(" Yaw = ");
  Serial.println(yaw);

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));

  }

 void encoder1(){

   a1State = digitalRead(outputA);
   
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (a1State != aLastState1){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != a1State) { 
       counter1 ++;
     } else {
       counter1 --;
     }
   } 
   aLastState1 = a1State; // Updates the previous state of the outputA with the current state
 
   
     Serial.print("Position1: ");
     Serial.print(counter1);
     

 
  }
 void encoder2(){
a2State = digitalRead(outputC);// Reads the "current" state of the outputA

   if (a2State != aLastState2){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputD) != a2State) { 
       counter2 ++;
     } else {
       counter2 --;
     }
   } 
   aLastState2 = a2State;

 Serial.print("Position2: ");
 Serial.println(counter2);
  
  }


  void run(){
     if(counter1 > -100 && counter2 > -100){
     forward();
    }
        
   if(counter1 <= -100 || counter2 <= -100){
    
        rotate();
        
   if(yaw>45.0){
   counter1 =0;
   counter2 =0;
        }
   }
    }

 void forward()
 {
  digitalWrite(mr1, HIGH);
  digitalWrite(mr2, LOW);
  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);
  analogWrite(mre,255);
  analogWrite(mle,255);
 }

 void rukja(){
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, LOW);
  digitalWrite(ml1, LOW);
  digitalWrite(ml2, LOW);
 }

 
 void rotate(){
  digitalWrite(mr1, LOW);
  digitalWrite(mr2, HIGH);
  digitalWrite(ml1, HIGH);
  digitalWrite(ml2, LOW);
  analogWrite(mre,125);
  analogWrite(mle,125);
 }

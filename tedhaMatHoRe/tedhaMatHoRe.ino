#include <Servo.h>
Servo esc1,esc2,esc3;
bool relay1, relay 2;
//when relay1 is off , motor is rotating clockwise, same for relay2
int depth;// actual depth
int reqd;// required depth
int errorAngle;
int multD;
int ikp,ikd,iki;//for imu
int dkp,dkd,dki;// for depth
bool dir1, dir2;
int s1,s2,s3;
///// for depth
int errord;
int prevd;
int derrord;
int sumd;
int terrord;
//////for angle
int errora;
int preva;
int derrora;
int suma;
int terrora;
// dir1 is false, motor is clockwise
int depthCall(){
   return depth;
}
void imuerror(){
  return imuerrorx;
  return imuerrory;
}
void setup() {
  // put your setup code here, to run once:
esc1.write(2,1500,2000);
esc2.write(3,1500,2000);
esc3.write(4,1500,2000);
Serial.begin(9600);
}
void  depthError(){
 depth =  depthCall();
 errord = reqd - depth;
 derrord = errord - prevd;
 prevd = errord;
 sumd += errord;
 terrord = dkp * errord + dkd * derrord + dki*sumd;
}
void angleErrorx(){
 errorx =  imuerrorx();
 derrorx = errorx - prevx;
 prevx = errorx;
 sumx += errorx;
 terrorx = ikp * errorx + ikd * derrorx + iki*sumx;
 //return terrorx;
}
void angleErrory(){
 errory =  imuerrory();
 derrory = errory - prevy;
 prevy = errory;
 sumy += errory;
 terrory = ikp * errory + ikd * derrory + iki*sumy;
 //return terrory;
}
void motorSpeed(){
  depthError();
  angleError();
  s1 = terrod + terrorx+ terrory;
    if(s1 >= 0){
      s1 = map(s1,0,500,0,180);
      relay1 = false;
      esc1.write(s1);
    }
    else {
      s1 = map(s1,-500,0,180,0);
      relay1 = true;
      esc1.write(s1);
    }
   s2 = terrod - terrorx+terrory;
    if(s2 >= 0){
      s2 = map(s2,0,500,0,180);
      relay2 = false;
      esc2.write(s2);
    }
    else {
      s2 = map(s2,-500,0,180,0);
      relay2 = true;
      esc2.write(s2);
    }
      s3 = terrod-terreoy;
      s3 = map(s3,0,500,0,180);
      //relay1 = false;
      esc3.write(s3);   
}
void loop() {
  // put your main code here, to run repeatedly:

}

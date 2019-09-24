int prev_err = 0, err, derr;
float kp = 0.36;
float kd = 0.25;
float corr,err1,err2;
int l1=130, r1=130;
int l,r;
int mrf=24, mrb=25, mlf=22,mlb=23;//mrf = dirction,mrb = break;
int pwml=11,pwmr=12;

            /**************************/

void setup() {
Serial.begin(9600);
Serial.println("Hi!, I am Arduino");

pinMode(mrf,OUTPUT);
pinMode(mrb,OUTPUT);
pinMode(mlf,OUTPUT);
pinMode(mlb,OUTPUT);

}

            /*************************/

void forward(){
  digitalWrite(mrf,HIGH);
  digitalWrite(mrb,LOW);
  digitalWrite(mlf,HIGH);
  digitalWrite(mlb,LOW);
  analogWrite(pwmr,r);
  analogWrite(pwml,l);
  }
  
/*void backward(){
  digitalWrite(mrf,LOW);
  digitalWrite(mrb,LOW);
  digitalWrite(mlf,LOW);
  digitalWrite(mlb,LOW);
  analogWrite(pwml,l);
  analogWrite(pwmr,r);
  }
  
void left(){
  digitalWrite(mrf,HIGH);
  digitalWrite(mrb,LOW);
  digitalWrite(mlf,LOW);
  digitalWrite(mlb,LOW);
  l=l1;
  r=r1;
  analogWrite(pwmr,r);
  analogWrite(pwml,l);
  }
void right(){
  digitalWrite(mrf,LOW);
  digitalWrite(mrb,LOW);
  digitalWrite(mlf,HIGH);
  digitalWrite(mlb,LOW);
  l=l1;
  r=r1;
  analogWrite(pwmr,r);
  analogWrite(pwml,l);
  }
*/  
void stop(){
  digitalWrite(mrf,HIGH);
  digitalWrite(mrb,HIGH);
  digitalWrite(mlf,HIGH);
  digitalWrite(mlb,HIGH);
  analogWrite(pwmr,0);
  analogWrite(pwml,0);
  }  
  
void calc_pwm(){
  derr = err - prev_err;
  prev_err = err;
  corr = kp*err + kd * derr;
  corr = int(corr);
  //Serial.println(err);
  //Serial.println(corr);
  }

void drive(){
  l = l1 + corr;
  r = r1 - corr;
  if(r<0) r=0;
  if(r>254) r=254;
  if(l<0) l=0;
  if(l>254) l=254;
  forward();
  Serial.print("corr");
  Serial.println(corr);
  Serial.print("l=");
  Serial.println(l);
  Serial.print("r=");
  
  
  Serial.println(r);
  }


void linefollow(){
  calc_pwm();
  drive();
  }

               /****************************/

void loop() {
  if (Serial.available()){
  String str1;
  //String str2;
  str1 = Serial.readString();
  //str2 = Serial.readString();
  err = str1.toInt();
  //err2 = str2.toInt();
  //err = (err2 * 256)+err1;
  err = map(err, -640, 0, -100, 100);
  Serial.print("ERROR=");
  Serial.println(err);
  linefollow();
  Serial.flush();
  //delay(100);
  }
}

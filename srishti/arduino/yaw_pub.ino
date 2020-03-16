#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <Servo.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//Variables
int error_yaw,setpoint,derrivative_yaw,prev_error_yaw,correction_yaw,value;
float kp,kd,yaw,pitch,roll;

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

 
ros::Subscriber<std_msgs::Int32> sub1("SetPoint_yaw", &message);
ros::Subscriber<std_msgs::Float32> sub2("Kp", &message1);
ros::Subscriber<std_msgs::Float32> sub3("Kd", &message2);

std_msgs::Int32 pwm_right;
std_msgs::Int32 pwm_left;
ros::Publisher pub1("/pwm1", &pwm_right);
ros::Publisher pub2("/pwm2", &pwm_left);
 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  nh.initNode();
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.subscribe(sub1);  
  nh.subscribe(sub2);  
  nh.subscribe(sub3); 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
thrusters_setup();
    
    }

void thrusters_setup(){
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

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
        /*    Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z); */
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
         /*   Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);/*/
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           // Serial.print("ypr\t");
          //  Serial.print(ypr[0] * 180/M_PI);
            yaw = (ypr[0] * 180/M_PI);
           // Serial.print("\t");
           // Serial.print(ypr[1] * 180/M_PI);
            pitch = (ypr[1] * 180/M_PI);
           // Serial.print("\t");
           // Serial.println(ypr[2] * 180/M_PI);
            roll = (ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
           // Serial.print("areal\t");
           // Serial.print(aaReal.x);
            //Serial.print("\t");
            //Serial.print(aaReal.y);
           // Serial.print("\t");
            //Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          /*  Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z); */
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
        
    }
    calculate_pid_yaw();
    balance_yaw();

    nh.spinOnce();
}

void calculate_pid_yaw()
{
error_yaw = yaw - setpoint; 
derrivative_yaw = error_yaw - prev_error_yaw;
correction_yaw = (kp*error_yaw)+(kd*derrivative_yaw);
prev_error_yaw = error_yaw;
}
void balance_yaw()
{
 value = map(correction_yaw, -200,200,-300,300);
 //servoRight.writeMicroseconds(1400+value);
 //servoLeft.writeMicroseconds(1600-value);
 pwm_right.data = 1500+value;
 pwm_left.data = 1500-value;
 pub1.publish( &pwm_right );
 pub2.publish( &pwm_left );
}

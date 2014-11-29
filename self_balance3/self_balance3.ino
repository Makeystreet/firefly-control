
//integrating encoders 

#include "I2Cdev.h"
#include <elapsedMillis.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define MOTOR_L_1 12
#define MOTOR_L_2 13
#define MOTOR_L_ENABLE 11
#define MOTOR_R_1 4
#define MOTOR_R_2 5
#define MOTOR_R_ENABLE 6
MPU6050 mpu;

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector


static int motorBottomOffset = 80;
int incomingByte = 0;	// for incoming serial data
int Kpid[]={30,0,150};
float Kp = 0;
float Ki = 0;
float Kd = 0;
float Kp_dis = 0;
float Ki_dis = 0;
float Kd_dis = 0;
float targetAngle = -3;
float currAngle = 0;
float pTerm = 0;
float iTerm = 0;
float dTerm = 0;
float integrated_error = 0;
float last_error = 0;
float K = 1;
int torque=0;



// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

int dis_l=0;
int dis_r=0;
int prev_dis_l=0;
int prev_dis_r=0;
void dis_int_l()
{
//  if (digitalRead(SPD_PUL_L))
//  Serial.println("left");
  if(torque<0)
    dis_l-=1;
  else
    dis_l+=1;
  delay(3);
}

void dis_int_r()
{
//  Serial.println("right");
  if (torque<0)
    dis_r-=1;
  else
    dis_r+=1;
  delay(3);
}


// ================================================================
// === INITIAL SETUP ===
// ================================================================
elapsedMillis timeElapsed;
unsigned int interval = 1000;
unsigned int count = 0;
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    while (!Serial); 
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(82);
    mpu.setYGyroOffset(-59);
    mpu.setZGyroOffset(30);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    mpu.setXAccelOffset(1470);
    mpu.setYAccelOffset(-4830);
    mpu.setZAccelOffset(1200);
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
    pinMode(MOTOR_L_ENABLE, OUTPUT);
    pinMode(MOTOR_R_1,OUTPUT);
    pinMode(MOTOR_R_2,OUTPUT);
    pinMode(MOTOR_R_ENABLE, OUTPUT);
    attachInterrupt(0, dis_int_l, RISING);
    attachInterrupt(1, dis_int_r, RISING);
  }

// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

void loop() {
    if (!dmpReady) return;
//    while (!mpuInterrupt && fifoCount < packetSize) {
//    }
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        //getting MMPU reading
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            

        updatePIDconstant();
        Kp=Kpid[0];
        Ki=Kpid[1];
        Kd=Kpid[2];
        currAngle = (ypr[2] * 180/M_PI);
        torque=updateSpeed();
        Drive_Motor(torque);
    }
    statusUpdate();
    count++;
    if (timeElapsed > interval) 
    {			
      Serial.print("frequency:");
      Serial.println(count);
      timeElapsed = 0;
      count = 0;
    }
}

float updateSpeed() {
  float error = targetAngle - currAngle;
  float error_dis = - (dis_l + dis_r)/2;
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -50, 50);
  dTerm = Kd * (error - last_error) 
  last_error = error;
  int speed = -constrain(K*(pTerm + iTerm + dTerm), -255+motorBottomOffset, 255-motorBottomOffset);
  if (speed > 0)
    speed+=motorBottomOffset;
  else
    speed-=motorBottomOffset;
  return speed;
  
}

float Drive_Motor(float torque)  {
//  Serial.print("torque: ");
//  Serial.println(torque);
  if (torque >= 0)  {                                        // drive motors forward
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
  }  else {        // drive motors backward
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
    
    
    torque = abs(torque);
  }
  analogWrite(MOTOR_L_ENABLE,torque);
  analogWrite(MOTOR_R_ENABLE,torque);    
}

//function that reads the serial port and updates the PID constants
void updatePIDconstant()
{
  int val=0;
  int i;
  for (i=0;i<3;i++)
  {
    val = 0;
    if (Serial.available() > 0) {
      while(1)
      {
        incomingByte = Serial.read();
        if (incomingByte>=48 && incomingByte<=57)
        {
          val=val*10 + incomingByte-48;
         Kpid[i]=val;
        }
        if (incomingByte==32)
          break; 
        if (incomingByte==13 || incomingByte==10)
          goto bailout;
      }
    }
  }
  bailout:
  delay(1);
      
  // say what you got:
//  Serial.print("I received: ");
//  Serial.println(incomingByte, DEC);
}
void statusUpdate()
{
  Serial.print("roll\t");
//  Serial.print(ypr[0] * 180/M_PI);
//  Serial.print("\t");
//  Serial.print(ypr[1] * 180/M_PI);
//  Serial.print("\t");
  Serial.print(ypr[2] * 180/M_PI);
  Serial.print("\tOutput: ");
  Serial.print(torque);
  Serial.print("\t");
  Serial.print(dis_l);
  Serial.print("\t");
  Serial.println(dis_r);
  
//  Serial.print("Kpid\t");
//  Serial.print(Kp);
//  Serial.print("\t");
//  Serial.print(Ki);
//  Serial.print("\t");
//  Serial.println(Kd);
  
  
}


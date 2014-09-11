#include <Arduino.h>
#include <PID_v1.h>
#include <I2Cdev.h>
#include <KalmanFilter.h>
#include <SelfHardwareMake.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif


#define ACCEL 0x3B
#define GYRO  0x43
#define KP 200
#define KI 300
#define KD 150

uint8_t ADR = 0x68;

uint8_t MAG_ADR = 0x0c;

uint8_t buffer[6];  
int16_t ax,ay,az;
int16_t p,q,r;

SelfHardwareMake motion;

// set time is 10ms
KalmanFilter kf(0.01);

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID MotorPID(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);



void sendFloat(float val);

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 2; // 24: 400kHz I2C clock (200kHz if CPU is 8MHz) //2014.01.10変えてみた．
    //TWBR = 12; // 12;400kHz I2C clock (400kHz if CPU is 16MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  // Serial.println("Initializing I2C devices...");

  uint8_t who_am_i;
  I2Cdev::readByte(ADR, 0x75, &who_am_i);
  // if(who_am_i == 0x71){
  //   Serial.println("Successfully connected to MPU9250");
  // }
  // else{
  //   Serial.println("Failed to Connect to MPU9250");
  // }

  I2Cdev::writeByte(ADR, 0x1b, 0x00);
  delay(1);

  I2Cdev::writeByte(ADR, 0x1c, 0x00);
  delay(1);

  I2Cdev::writeByte(ADR, 0x6b, 0x00);
  delay(1);

  I2Cdev::writeByte(ADR, 0x37, 0x02);

  I2Cdev::readByte(MAG_ADR,0x00,&who_am_i);
  if(who_am_i == 0x48){
    // Serial.println("Successfully connected to COMPASS(AK8963)");
  }
  else{
    // Serial.println("Failed to Connect to COMPASS(AK8963)");
  }
  delay(5);

  I2Cdev::writeByte(MAG_ADR, 0x0a, 0x00);
  delay(10);


  I2Cdev::writeByte(MAG_ADR, 0x0a, 0x16);
  delay(10); 


  MotorPID.SetOutputLimits(-255.0, 255.0);
  MotorPID.SetMode(AUTOMATIC);
  MotorPID.SetSampleTime(10);
  MotorPID.SetTunings(KP, KI, KD);
  Setpoint = 0;
}

unsigned long time = 0;
float a;
float b;

void sendFloat(float val)
{
    byte * p;
    p = (byte *) &val;
    Serial.write(p, sizeof(float));
}

void loop()
{
  while(millis() < time + 10);
  
  I2Cdev::readBytes(ADR,ACCEL,6,buffer);
  ax = (((int16_t)buffer[0]) << 8) | buffer[1];
  ay = (((int16_t)buffer[2]) << 8) | buffer[3];
  az = (((int16_t)buffer[4]) << 8) | buffer[5];

  I2Cdev::readBytes(ADR,GYRO,6,buffer);
  p = (((int16_t)buffer[0]) << 8) | buffer[1];
  q = (((int16_t)buffer[2]) << 8) | buffer[2];
  r = (((int16_t)buffer[4]) << 8) | buffer[5];  

  kf.kalman_update(az, ay);
  kf.state_update((float) p * DEG_TO_RAD / 131);

  Input = (double) (kf.getAngle());

  MotorPID.Compute();
  
  if (Output > 0) {
    motion.front((int)Output, (int)Output);
  } else if (Output < 0){
    motion.back((int)-Output, (int)-Output);
  } else {
    motion.stop();
  }

  a = kf.getAngle();
  b = p * DEG_TO_RAD / 14.375;

  Serial.write(0xA5);
  Serial.write(0x5A);
  sendFloat(a);
  sendFloat(b);
  time = millis();
}
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <AHRS.h>

MPU6050 MPU;
 
long LastTime,NowTime,TimeSpan;

IMU imu;
 
void setup()//MPU6050的设置都采用了默认值，请参看库文件
{
    Wire.begin();

    Serial.begin(115200);

    Serial.println("Initializing I2C device.....");
    MPU.initialize();

    Serial.println("Testing device connections...");
    Serial.println(MPU.testConnection() ? "MPU6050 connection successful":"MPU6050 connection failure");
}

void loop()
{
    MPU.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
}



#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#include <math.h>
#include "CommunicationUtils.h"

#define ACCEL 0x3B
#define GYRO  0x43

#define GYRO_CONSTANT PI / (131 * 180)
#define ACC_CONSTANT 2 / 32768
#define PI 3.141592653
// #define ACC_CONSTANT 2 / 32768
// #define GYRO_CONSTANT 1 / 131


uint8_t ADR = 0x68;

uint8_t buffer[6];

//陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq  200.0f      // sample frequency in Hz
#define betaDef     0.1f        // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions


// 2 * proportional gain (Kp)
volatile float beta = betaDef;           
// quaternion of sensor frame relative to auxiliary frame                   
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  
float q[4];


// Quaternions to Rotation matrix
// 

class Quat {
public:
    Quat():w(q[0]), x(q[1]), y(q[2]), z(q[3]){

    }
    float q[4];
    float &w;
    float &x;
    float &y;
    float &z;
};

class RotaMat{
public:
    RotaMat(){}
    float mat[3][3];
};



class Vector3{
public:
    Vector3() : x(v[0]), y(v[1]), z(v[2]){
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 0.0f;
        
    }
    float v[3];

    float &x;
    float &y;
    float &z;
};


class IMU{
public:
    IMU();
    ~IMU();
    Vector3 acc;
    Vector3 gyro;
    Vector3 magn;
};

Vector3 GYRO_OFFSET;
Vector3 rawAcc;
Vector3 rawGyro;
Vector3 gravity;

void showXYZ(Vector3 a) {
    Serial.print("x = ");
    Serial.print(a.x);
    Serial.print(",");
    Serial.print("y = ");
    Serial.print(a.y);
    Serial.print(",");
    Serial.print("z = ");
    Serial.println(a.z);
}

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void QuaternionsToRotationMatrix(RotaMat &rotaMat, Quat &quat) {
    rotaMat.mat[0][0] = quat.w * quat.w + quat.x * quat.x 
                      - quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[0][1] = 2.0 * (quat.x * quat.y - quat.w * quat.z);
    rotaMat.mat[0][2] = 2.0 * (quat.x * quat.z + quat.w * quat.y);

    rotaMat.mat[1][0] = 2.0 * (quat.x * quat.y + quat.w * quat.z);
    rotaMat.mat[1][1] = quat.w * quat.w - quat.x * quat.x 
                      + quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[1][2] = 2.0 * (quat.y * quat.z - quat.w * quat.x);

    rotaMat.mat[2][0] = 2.0 * (quat.x * quat.z - quat.w * quat.y);
    rotaMat.mat[2][1] = 2.0 * (quat.y * quat.z + quat.w * quat.x);
    rotaMat.mat[2][2] = quat.w * quat.w - quat.x * quat.x 
                      - quat.y * quat.y + quat.z * quat.z;
}
// void QuaternionsToRotationMatrix(RotaMat &rotaMat, Quat &quat) {
//     rotaMat.mat[0][0] = quat.w * quat.w + quat.x * quat.x 
//                       - quat.y * quat.y - quat.z * quat.z;
//     rotaMat.mat[0][1] = 2.0 * (quat.x * quat.y - quat.w * quat.z);
//     rotaMat.mat[0][2] = 2.0 * (quat.x * quat.z + quat.w * quat.y);

//     rotaMat.mat[1][0] = 2.0 * (quat.x * quat.y + quat.w * quat.z);
//     rotaMat.mat[1][1] = quat.w * quat.w - quat.x * quat.x 
//                       + quat.y * quat.y - quat.z * quat.z;
//     rotaMat.mat[1][2] = 2.0 * (quat.y * quat.z + quat.w * quat.x);

//     rotaMat.mat[2][0] = 2.0 * (quat.x * quat.z - quat.w * quat.y);
//     rotaMat.mat[2][1] = 2.0 * (quat.y * quat.z - quat.w * quat.x);
//     rotaMat.mat[2][2] = quat.w * quat.w - quat.x * quat.x 
//                       - quat.y * quat.y + quat.z * quat.z;
// }

void convertXYZ(Vector3 &XYZ, Vector3 &Acc,RotaMat &rotaMat) {
    for (int i = 0; i < 3; ++i) {
        XYZ.v[i] = 0;
        for (int j = 0; j < 3; ++j) {
            XYZ.v[i] += rawAcc.v[j] * rotaMat.mat[j][i];
        }
    }
}

void gravityCompensateAcc(Vector3 &acc, Vector3 &rawAcc, float * q) {
  float g[3];
  
  // get expected direction of gravity in the sensor frame
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  // compensate accelerometer readings with the expected direction of gravity
  acc.x = rawAcc.x - g[0];
  acc.y = rawAcc.y - g[1];
  acc.z = rawAcc.z - g[2];
}

void getEulerRad(float * angles) {
  float q[4]; // quaternion
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}



void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12;    // 24: 400kHz I2C clock (200kHz if CPU is 8MHz)
        // TWBR = 12;  // 12; 400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    I2Cdev::writeByte(ADR, 0x1b, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x1c, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x6b, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x37, 0x02);

    Vector3 tmpOffset;
    #define TOTAL_SAMPLE 50
    for (int i = 0; i < TOTAL_SAMPLE; ++i) {
        I2Cdev::readBytes(ADR,GYRO,6,buffer);
        rawGyro.x = (((int16_t)buffer[0]) << 8) | buffer[1];
        rawGyro.y = (((int16_t)buffer[2]) << 8) | buffer[2];
        rawGyro.z = (((int16_t)buffer[4]) << 8) | buffer[5];

        rawGyro.x = float(rawGyro.x) * GYRO_CONSTANT;
        rawGyro.y = float(rawGyro.y) * GYRO_CONSTANT;
        rawGyro.z = float(rawGyro.z) * GYRO_CONSTANT;

        tmpOffset.x += rawGyro.x;
        tmpOffset.y += rawGyro.y;
        tmpOffset.z += rawGyro.z;
    }

    GYRO_OFFSET.x = tmpOffset.x / TOTAL_SAMPLE;
    GYRO_OFFSET.y = tmpOffset.y / TOTAL_SAMPLE;
    GYRO_OFFSET.z = tmpOffset.z / TOTAL_SAMPLE;
}

long time = 0;
long nowTime;


RotaMat rotaMat;
Quat quat;


void loop() {
    nowTime = millis();
    if (nowTime >= time + 5) {
        time = nowTime;
        I2Cdev::readBytes(ADR,ACCEL,6,buffer);
        rawAcc.x = (((int16_t)buffer[0]) << 8) | buffer[1];
        rawAcc.y = (((int16_t)buffer[2]) << 8) | buffer[3];
        rawAcc.z = (((int16_t)buffer[4]) << 8) | buffer[5];

        I2Cdev::readBytes(ADR,GYRO,6,buffer);
        rawGyro.x = (((int16_t)buffer[0]) << 8) | buffer[1];
        rawGyro.y = (((int16_t)buffer[2]) << 8) | buffer[2];
        rawGyro.z = (((int16_t)buffer[4]) << 8) | buffer[5];

        rawAcc.x = float(rawAcc.x) * ACC_CONSTANT;
        rawAcc.y = float(rawAcc.y) * ACC_CONSTANT;
        rawAcc.z = float(rawAcc.z) * ACC_CONSTANT;

        rawGyro.x = float(rawGyro.x) * GYRO_CONSTANT;
        rawGyro.y = float(rawGyro.y) * GYRO_CONSTANT;
        rawGyro.z = float(rawGyro.z) * GYRO_CONSTANT;

        rawGyro.x -= GYRO_OFFSET.x;
        rawGyro.y -= GYRO_OFFSET.y;
        rawGyro.z -= GYRO_OFFSET.z;

        MadgwickAHRSupdateIMU(rawGyro.x, rawGyro.y, rawGyro.z, rawAcc.x, rawAcc.y, rawAcc.z);
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;

        // quat.w = q0;
        // quat.x = q1;
        // quat.y = q2;
        // quat.z = q3;

        // showXYZ(rawAcc);

        // Vector3 acc;
        // gravityCompensateAcc(acc, rawAcc, q);
        // showXYZ(acc);

        // QuaternionsToRotationMatrix(rotaMat, quat);

        // Vector3 XYZ;
        // convertXYZ(XYZ, acc, rotaMat);
        // showXYZ(XYZ);

        // Serial.println("");
        // 
        serialPrintFloatArr(q, 4);
        Serial.println("");
    }
}


void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


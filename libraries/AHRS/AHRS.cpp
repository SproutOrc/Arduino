//
//  AHRS.cpp
//  AHRS
//
//  Created by Atropos on 15/1/3.
//  Copyright (c) 2015骞� Atropos. All rights reserved.
//

#include "AHRS.h"
#include "math.h"

AHRS::AHRS(){
    sampleFreq = 100.0f;
    beta = 0.1f;
}

AHRS::AHRS(float sampleFreq, float beta) {
    this->sampleFreq = sampleFreq;
    this->beta = beta;
}

AHRS::~AHRS(){}
//void AHRS::getQuat(Quat &quat) {
//    quat.w = this->quat.w;
//    quat.x = this->quat.x;
//    quat.y = this->quat.y;
//    quat.z = this->quat.z;
//}

void AHRS::UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-quat.x * gx - quat.y * gy - quat.z * gz);
    qDot2 = 0.5f * (quat.w * gx + quat.y * gz - quat.z * gy);
    qDot3 = 0.5f * (quat.w * gy - quat.x * gz + quat.z * gx);
    qDot4 = 0.5f * (quat.w * gz + quat.x * gy - quat.y * gx);
    
    // Compute feedback only if accelerometer measurement
    // valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * quat.w;
        _2q1 = 2.0f * quat.x;
        _2q2 = 2.0f * quat.y;
        _2q3 = 2.0f * quat.z;
        _4q0 = 4.0f * quat.w;
        _4q1 = 4.0f * quat.x;
        _4q2 = 4.0f * quat.y;
        _8q1 = 8.0f * quat.x;
        _8q2 = 8.0f * quat.y;
        q0q0 = quat.w * quat.w;
        q1q1 = quat.x * quat.x;
        q2q2 = quat.y * quat.y;
        q3q3 = quat.z * quat.z;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * quat.x - _2q0 * ay
           - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * quat.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * quat.z - _2q1 * ax + 4.0f * q2q2 * quat.z - _2q2 * ay;
        // normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
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
    quat.w += qDot1 * (1.0f / sampleFreq);
    quat.x += qDot2 * (1.0f / sampleFreq);
    quat.y += qDot3 * (1.0f / sampleFreq);
    quat.z += qDot4 * (1.0f / sampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z);
    quat.w *= recipNorm;
    quat.x *= recipNorm;
    quat.y *= recipNorm;
    quat.z *= recipNorm;
}

void AHRS::UpdateIMU(const IMU &imu) {
    UpdateIMU(imu.gyro.x, imu.gyro.y, imu.gyro.z, imu.acc.x, imu.acc.y, imu.acc.z);
}

void AHRS::QuaternionsToRotationMatrix(RotaMat &rotaMat) const{
    rotaMat.mat[0][0] = quat.w * quat.w + quat.x * quat.x
                      - quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[0][1] = 2.0f * (quat.x * quat.y - quat.w * quat.z);
    rotaMat.mat[0][2] = 2.0f * (quat.x * quat.z + quat.w * quat.y);
    
    rotaMat.mat[1][0] = 2.0f * (quat.x * quat.y + quat.w * quat.z);
    rotaMat.mat[1][1] = quat.w * quat.w - quat.x * quat.x
                      + quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[1][2] = 2.0f * (quat.y * quat.z - quat.w * quat.x);
    
    rotaMat.mat[2][0] = 2.0f * (quat.x * quat.z - quat.w * quat.y);
    rotaMat.mat[2][1] = 2.0f * (quat.y * quat.z + quat.w * quat.x);
    rotaMat.mat[2][2] = quat.w * quat.w - quat.x * quat.x
                      - quat.y * quat.y + quat.z * quat.z;
}

void AHRS::GravityCompensateAcc(float * acc, float * q) {
    float g[3];
    
    // get expected direction of gravity in the sensor frame
    g[0] = 2.0f * (quat.x * quat.z - quat.w * quat.y);
    g[1] = 2.0f * (quat.y * quat.z + quat.w * quat.x);
    g[2] = quat.w * quat.w - quat.x * quat.x
         - quat.y * quat.y + quat.z * quat.z;
    
    // compensate accelerometer readings with the expected direction of gravity
    acc[0] = acc[0] - g[0];
    acc[1] = acc[1] - g[1];
    acc[2] = acc[2] - g[2];
}

void AHRS::GravityCompensateAcc(Vector3 &acc) {
    GravityCompensateAcc(acc.v, quat.q);
}

void AHRS::EulerRad(float * angles) const {
    angles[0] = atan2(2 * quat.x * quat.y - 2 * quat.w * quat.z, 2 * quat.w * quat.w + 2 * quat.x * quat.x - 1); // psi
    angles[1] = -asin(2 * quat.x * quat.z + 2 * quat.w * quat.y); // theta
    angles[2] = atan2(2 * quat.y * quat.z - 2 * quat.w * quat.x, 2 * quat.w * quat.w + 2 * quat.z * quat.z - 1); // phi
}

void AHRS::EulerRad(Vector3 &v3) const{
    EulerRad(v3.v);
}

/**
 * Fast inverse square-root
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */

float AHRS::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

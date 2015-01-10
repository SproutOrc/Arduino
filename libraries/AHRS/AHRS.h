//
//  AHRS.h
//  AHRS
//
//  Created by Atropos on 15/1/3.
//  Copyright (c) 2015骞� Atropos. All rights reserved.
//

#ifndef __AHRS__AHRS__
#define __AHRS__AHRS__

class Quat{
public:
    Quat():w(q[0]), x(q[1]), y(q[2]), z(q[3]){
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
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

class IMU {
public:
    IMU(){};
    ~IMU(){};
    
    Vector3 acc;
    Vector3 gyro;
    Vector3 magn;
};

class AHRS{
public:
    AHRS();
    AHRS(float sampleFreq, float beta);
    ~AHRS();
    
    void UpdateIMU(const IMU &imu);
    void GravityCompensateAcc(Vector3 &acc);
    void QuaternionsToRotationMatrix(RotaMat &rotaMat) const;
    void EulerRad(Vector3 &v3) const;
//    void getQuat(Quat &quat);
    Quat quat;
    
private:
    
    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void GravityCompensateAcc(float * acc, float * q);
    void EulerRad(float * angles) const;
    
    float invSqrt(float x);
    
    // 2 * proportional gain (Kp)
    float beta;
    
    float sampleFreq;
};

#endif /* defined(__AHRS__AHRS__) */

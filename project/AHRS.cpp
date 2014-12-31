//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq  200.0f      // sample frequency in Hz
#define betaDef     0.1f        // 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions
               

float q[4];

class Quat {
public:
    Quat():w(q[0]), x(q[1]), y(q[2]), z(q[3]){
        // quaternion of sensor frame relative to auxiliary frame    
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

class Vector3{
public:
    Vector3() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    float x;
    float y;
    float z;
};

class AHRS{
public:
    AHRS(float sampleFreq, float beta);
    AHRS() : sampleFreq(200.0f), beta(0.1f){};
    ~AHRS();

    void QuaternionsToRotationMatrix(RotaMat &rotaMat, Quat &quat);
    void GravityCompensateAcc(float * acc, float * q);
    void GetEulerRad(float * angles);
    void UpdateIMU(Quat q, float gx, float gy, float gz, float ax, float ay, float az);

    Vector3 acc;
    Vector3 gyro;
    Vector3 magn;

private:
    float invSqrt(float x);

    // 2 * proportional gain (Kp)
    float beta;

    float sampleFreq;

    Quat quat;
};

AHRS::AHRS(float sampleFreq, float beta) {
    this->sampleFreq = sampleFreq;
    this->beta = beta;
}

void AHRS::QuaternionsToRotationMatrix(RotaMat &rotaMat, Quat &quat) {
    rotaMat.mat[0][0] = quat.w * quat.w + quat.x * quat.x 
                      - quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[0][1] = 2.0 * (quat.x * quat.y - quat.w * quat.z);
    rotaMat.mat[0][2] = 2.0*(quat.x*quat.z+quat.w*quat.y);

    rotaMat.mat[1][0] = 2.0 * (quat.x * quat.y + quat.w * quat.z);
    rotaMat.mat[1][1] = quat.w * quat.w - quat.x * quat.x 
                      + quat.y * quat.y - quat.z * quat.z;
    rotaMat.mat[1][2] = 2.0 * (quat.y * quat.z - quat.w * quat.x);

    rotaMat.mat[2][0] = 2.0 * (quat.x * quat.z - quat.w * quat.y);
    rotaMat.mat[2][1] = 2.0 * (quat.y * quat.z + quat.w * quat.x);
    rotaMat.mat[2][2] = quat.w * quat.w - quat.x * quat.x 
                      - quat.y * quat.y + quat.z * quat.z;
}

void AHRS::GravityCompensateAcc(float * acc, float * q) {
  float g[3];
  
  // get expected direction of gravity in the sensor frame
  g[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
  g[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
  g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  // compensate accelerometer readings with the expected direction of gravity
  acc[0] = acc[0] - g[0];
  acc[1] = acc[1] - g[1];
  acc[2] = acc[2] - g[2];
}

void AHRS::GetEulerRad(float * angles) {
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

void AHRS::UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
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


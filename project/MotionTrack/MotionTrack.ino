#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "AHRS.h"

// 单位转换；rad/s
#define GYRO_CONST PI / (131.072 * 180)
// 单位转换；g * m / s^2
#define ACC_CONST 2 / 32768

Vector3 GYRO_BIAS;

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

IMU imu;
AHRS ahrs(100.0, 0.1);

short int buffer[6] = {0};

void getGyroBias(Vector3 &gyroBias) {
    float g[3] = {0};
    for (int i = 0; i < 20; ++i) {
        mpu.getRotation(&gx, &gy, &gz);
        g[0] += float(gx) * GYRO_CONST;
        g[1] += float(gy) * GYRO_CONST;
        g[2] += float(gz) * GYRO_CONST;
    }
    gyroBias.x = g[0] / 20;
    gyroBias.y = g[1] / 20;
    gyroBias.z = g[2] / 20;
}

void setup()//MPU6050的设置都采用了默认值，请参看库文件
{
    Wire.begin();

    Serial.begin(115200);

//    Serial.println("Initializing I2C device.....");
    mpu.initialize();

//    Serial.println("Testing device connections...");
//    Serial.println(mpu.testConnection() ? "MPU6050 connection successful":"MPU6050 connection failure");

    getGyroBias(GYRO_BIAS);
}

Vector3 acc;
Vector3 oxyz;
RotaMat rotaMat;

unsigned long int nowTime, lastTime = 0;

void loop()
{
    nowTime = millis();
    if (nowTime > (lastTime + 10)) {
        lastTime = nowTime;

        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        imu.acc.x = float(ax) * ACC_CONST;
        imu.acc.y = float(ay) * ACC_CONST;
        imu.acc.z = float(az) * ACC_CONST;

        imu.gyro.x = float(gx) * GYRO_CONST - GYRO_BIAS.x;
        imu.gyro.y = float(gy) * GYRO_CONST - GYRO_BIAS.y;
        imu.gyro.z = float(gz) * GYRO_CONST - GYRO_BIAS.z;

        ahrs.UpdateIMU(imu);

        acc.x = imu.acc.x;
        acc.y = imu.acc.y;
        acc.z = imu.acc.z;

        ahrs.GravityCompensateAcc(acc);

        ahrs.QuaternionsToRotationMatrix(rotaMat);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                oxyz.v[i] = acc.v[j] * rotaMat.mat[j][i];
            }
        }

        Serial.print("x = ");
        Serial.print(oxyz.x, 5);
        Serial.print(",");
        Serial.print("y = ");
        Serial.print(oxyz.y, 5);
        Serial.print(",");
        Serial.print("z = ");
        Serial.println(oxyz.z, 5);
    }

}

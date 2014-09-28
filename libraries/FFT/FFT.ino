#include "Wire.h"
#include "I2Cdev.h"
#include "PID_v1.h"

#include "KalmanFilter.h"
#include "SelfHardwareMake.h"
//时间操作系统头文件  
//本程序用作timeChange时间采集并处理一次数据
#include "Timer.h"
#include "MPU6050.h"

MPU6050 accelgyro;//陀螺仪类

double KP = 50.0;
double KI = 20.0;
double KD = 0.4;

#define SET_POINT -1.5
#define SAMPLE_TIME 50
#define SPEED 240.0
#define OFFSET_LEFT 0
#define OFFSET_RIGHT 0
#define SET_ERROR 0.5

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID MotorPID(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);

SelfHardwareMake motion;

KalmanFilter kf(SAMPLE_TIME * 0.001);
KalmanFilter turnKf(50 * 0.001);
//时间类
Timer t;


//滤波法采样时间间隔毫秒
float timeChange=SAMPLE_TIME;
// //计算后的角度（与x轴夹角）和角速度
float angleAx,gyroGy;
//陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;

float angle;


void setup() {

    Serial1.begin(57600);
    accelgyro.initialize();//初始化

    //本语句执行以后timeChange毫秒执行回调函数getangle
    int tickEvent1=t.every(timeChange, getangle);
    //本语句执行以后50毫秒执行回调函数printout，串口输出
    int tickEvent2=t.every(50, printout);

    MotorPID.SetOutputLimits(-SPEED, SPEED);
    MotorPID.SetMode(AUTOMATIC);
    MotorPID.SetSampleTime(SAMPLE_TIME);
    MotorPID.SetTunings(KP, KI, KD);
    Setpoint = SET_POINT;
}   

void loop() {
    //时间操作系统运行
    t.update();
}

void printout()
{
    // Serial1.print(Setpoint);Serial1.print(',');
    // Serial1.print(angleAx);Serial1.print(',');
    // Serial1.print(kf.getAngle());Serial1.print(',');
    // Serial1.println(40);
    Serial1.print("Setpoint = ");
    Serial1.print(Setpoint);
    Serial1.print(',');
    Serial1.print("angleAx = ");
    Serial1.print(angleAx);
    Serial1.print(',');
    Serial1.print("kfAngle = ");
    Serial1.println(kf.getAngle());
}

int flag = 0;

void getangle() 
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);//读取原始6个数据
    // 计算与x轴夹角
    angleAx = atan2(-ax, -az) * 180 / PI;
    // 计算角速度
    gyroGy = -gy / 131.00;

    kf.state_update(gyroGy);
    kf.kalman_update(angleAx);
    angle = kf.getAngle();

    if (abs(angle) > 40) {
        flag = 1;
        MotorPID.SetMode(MANUAL);
        Output = 0;
        motion.stop();
        
        return;
    }

    // if (angle <= Setpoint + SET_ERROR 
    //  && angle >= Setpoint - SET_ERROR) {
    //     flag = 1;
    //     MotorPID.SetMode(MANUAL);
    //     Output = 0;
    //     motion.stop();
    //     return;
    // } else 
    if (flag == 1) {
        flag = 0;
        MotorPID.SetMode(AUTOMATIC);
    }

    Input = angle;

    MotorPID.Compute();

    if (Output > 0) {
        motion.back(OFFSET_LEFT + (int)Output, OFFSET_RIGHT + (int)Output);
    } else{
        motion.front (OFFSET_LEFT - (int)Output, OFFSET_RIGHT - (int)Output);
    } 
}
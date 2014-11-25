
/**
 * 角度环控制
 */

#define ANGLE_OFFSET 0
#define GYRO_OFFSET 0
#define ANGLE_P 45
#define ANGLE_D 0.35
void AngleSabilityControl(
            float &angleControl, 
      const float &angle, 
      const float &gyro
) {
    float value;

    value = (ANGLE_OFFSET - angle) * ANGLE_P +
            (GYRO_OFFSET - gyro) *  ANGLE_D;

    angleControl = value;
}

/**
 * 速度环控制
 */

#define SPEED_CONSTANT 1
#define SPEED_P 0.09
#define SPEED_I 1.1

void SpeedSabilityControl(
            float &nowSpeedControl, 
            float &lastSpeedControl, 
      const int &setSpeed,
            int &leftSpeed,
            int &rightSpeed
) {
    float error;
    float pValue, iValue;
    static float position = 0;

    float realSpeed = (leftSpeed + rightSpeed) / 2.0;
    realSpeed *= SPEED_CONSTANT;

    error = setSpeed - realSpeed;
    pValue = error * SPEED_P;
    iValue = error * SPEED_I;

    position += iValue;

    lastSpeedControl = nowSpeedControl;
    nowSpeedControl  = pValue + position;
}

/**
 * 速度平滑控制
 */
#define SPEED_SCALE_MAX 10
void SpeedSmoothControl(
            float &speedControl,
      const float &nowSpeedControl,
      const float &lastSpeedControl
) {
    static int count = 1;
    float value;

    value = nowSpeedControl - lastSpeedControl;

    value = value * count / SPEED_SCALE_MAX + lastSpeedControl;
    count ++;
    if (count == SPEED_SCALE_MAX + 1) count = 1;
    speedControl = value;
}

/**
 * 方向控制
 */
#define TURN_SPEED_CONSTANT 1
#define TURN_SPEED_P 1
#define TURN_SPEED_D 1

void TurnSpeedSabilityControl(
            float &nowTurnSpeedControl,
            float &lastTurnSpeedControl,
      const float &gyro,
      const float &setTurnSpeed, 
      const int &leftSpeed, peed
) {
    float value;
    float error;
    float twoWheelSpeedDiff;

    // 计算两个轮子的速度差 向左转为正
    twoWheelSpeedDiff = rightSpeed - leftSpeed;
    twoWheelSpeedDiff *= TURN_SPEED_CONSTANT;
    error = setTurnSpeed - twoWheelSpeedDiff;
    value = error * TURN_SPEED_P;
    value += gyro * TURN_SPEED_D;
    lastTurnSpeedControl = nowTurnSpeedControl;
    nowTurnSpeedControl = value;
}

/**
 * 方向平滑控制
 */
#define TURN_SPEED_SCALE_MAX 2

void TurnSpeedSmoothControl(
            float &turnSpeedControl,
      const float &nowTurnSpeedControl,
      const float &lastTurnSpeedControl,
) {
    static int count = 1;
    float value;

    value = nowTurnSpeedControl - lastTurnSpeedControl;

    value = value * count / TURN_SPEED_SCALE_MAX + lastTurnSpeedControl;
    count ++;
    if (count == TURN_SPEED_SCALE_MAX + 1) count = 1;
    turnSpeedControl = value;
}

/**
 * 左右电机输出控制
 */

void MotorSpeedCreate(
            
) {

}
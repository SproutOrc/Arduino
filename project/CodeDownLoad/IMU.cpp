//********************************************************************
// Function:四元数更新算法
//********************************************************************
 
void IMUupdate(double gx, double gy, double gz, double ax, double ay, double az)
{
    double norm;
    double vx, vy, vz;
    double ex, ey, ez; 

    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;  // acc数据归一化

    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;  // estimated direction of gravity

    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);   // error is sum of cross product between reference direction of field and direction measured by sensor

    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;   // integral error scaled integral gain

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;   // adjusted gyroscope measurements

    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;   // integrate quaternion rate and normalise

    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;  // normalise quaternion

    //更新方向余弦矩阵

    double t11 = q0*q0+q1*q1-q2*q2-q3*q3;
    double t12 = 2.0*(q1*q2-q0*q3);
    double t13 = 2.0*(q1*q3+q0*q2);
    double t21 = 2.0*(q1*q2+q0*q3);
    double t22 = q0*q0-q1*q1+q2*q2-q3*q3;
    double t23 = 2.0*(q2*q3-q0*q1);
    double t31 = 2.0*(q1*q3-q0*q2);
    double t32 = 2.0*(q2*q3+q0*q1);
    double t33 = q0*q0-q1*q1-q2*q2+q3*q3;

    //求出欧拉角
    roll = asin(t32) * RAD_TO_DEG;//俯仰角，绕x轴转动
    pitch = -atan2(t31,t33) * RAD_TO_DEG;//横滚角，绕y轴转动
    yaw = atan2(t12,t22) * RAD_TO_DEG;//偏航角，绕z轴转动
 
}
 
//********************************************************************
//  Function: 消除重力加速度影响
//********************************************************************
void Sub_g(double aa, double bb, double cc )
{
    int i, j, k;
    double s[3][1] = {0,0,0};
    double a[3][3] = {t11, t21, t31, t12, t22, t32, t13, t23, t33};
    double b[3][1] = {0, 0, g};

    for (i = 0; i < 3; i++)
    {
      for (j = 0; j < 1; j++)
      {
        for (k  = 0; k < 3; k++)
        {
          s[i][j] += a[i][k] * b[k][j];  
        }
      }
    }
    for ( int l = 0; l < 3; l++)
    {
      c[l][0] = s[l][0];
    }        
    aa -= c[0][0];
    bb -= c[1][0];
    cc -= c[2][0];
}
 
//********************************************************************
//  Function: 坐标变换，acc数值由b系变换至n系
//********************************************************************
void acc_convert(double x, double y, double z)
{
    double w[3][1] = {0,0,0};
    double f[3][1];
    double d[3][3] = {t11, t12, t13, t21, t22, t23, t31, t32, t33};
    double e[3][1] = {x, y, z};

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 1; j++)
      {
        for (int k = 0 ; k < 3; k++)
        {
          w[i][j] += d[i][k] * e[k][j]; 
        }
      }
    }
    for ( int l = 0; l < 3; l++)
    {
      f[l][0] = w[l][0];
    }              
    x = f[0][0];
    y = f[1][0];
    z = f[2][0];
}
//********************************************************************
//  Function: 加速度一重积分
//  V(n) = V(n-1) + 0.5 * (a(n) + a(n-1)) * dt
//********************************************************************
void acc_to_vel(int n)
{
    double result[3][1] = {vel_x, vel_y, vel_z};  //计算结果Vn 
    double previous[3][1];  //前一结果Vn-1
    double al[3][1] = {init_ax/16384.0, init_ay/16384.0, init_az/16384.0};  //当前结果a(n)
    double ap[3][1] = {0,0,0};  //前一结果a(n-1)  
 
    while (n >= 1)
    {
      n--;
      for (int i = 0; i < 3; i++)
      {
        previous[i][0] = result[i][0];
      }
      for (int j = 0; j < 3; j++)  
      {
        result[j][0] = previous[j][0] + 0.5 * (al[j][0] + ap[j][0]) * dt;  //V(n)=V(n-1)+0.5*(a(n)+a(n-1))*dt
      }
      for (int k = 0; k < 3; k++)
      {
        ap[k][0] = al[k][0];
      }
    }
    vel_x = result[0][0];
    vel_y = result[1][0];
    vel_z = result[2][0];
    velocity = sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);
 
Serial.print("vx =");
Serial.print(vel_x);Serial.print("\t");
Serial.print("vy =");
Serial.print(vel_y);Serial.print("\t");
Serial.print("vz =");
Serial.print(vel_z);Serial.print("\t");
Serial.print("velocity =");
Serial.print(velocity);Serial.print("\t");
Serial.print("\t");
}

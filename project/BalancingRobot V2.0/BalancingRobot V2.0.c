/*
 * BalancingRobot_V3.c
 *
 * Created: 2014.06.25. 12:55:23
 *  Author: F
 */ 

#ifndef F_CPU
#define F_CPU 8000000UL
#endif




#define MYUBRR 12	//8MHz 38400baud 0.2% error UBRR=12

#define RAD_to_DEG		57.295779513082320876798154814105
#define DEG_to_RAD		0.0174532925

#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/atomic.h> 
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "header/checkBetween.h"
#include "header/balance.h"
#include "header/i2cmaster.h"
#include "header/MPU6050.h"
#include "header/IRencoder.h"
#include "header/odometry.h"
#include "header/PID.h"
#include "header/motorController.h"
#include "header/ASCIIprotocol.h"
#include "header/commandList.h"
#include "header/behavior.h"

//----------USART------------------------------------
#define MYUBRR 12	//8MHz 38400baud 0.2% error UBRR=12

#define MANUAL_STOP		0	//If the robotStart.startType set to this then we force the robot to start balancing
#define MANUAL_START	1	//If the robotStart.startType set to this then the robot stops balancing
#define AUTO			2	//If the robotStart.startType set to this then the robot automatically start and stops balancing 
#define TURN_OFF_ANGLE	50	//Above this angle the robot stops balancing and the motors turn off
#define TURN_ON_ANGLE	2	//Below this angle the robot starts balancing

int main(void)
{
	typedef struct				//This struct used for starting the robot's balancing loop.
	{
		bool started;			//If this == true then the requirements met to start the balancing loop.
		uint8_t startType;		//If this == AUTO then the robot will start balancing when it standed up. (COMP.roll <2 >-2)
								//If this == MANUAL_START then the robot will start balancing (because a command is sent to the robot)
								//If this == MANUAL_STOP  then the robot will stop balancing (because a command is sent to the robot)
	}robotStartBalanceSTRUCT;
	
	typedef struct
	{
		int8_t counter;
		float RPM;
	}motorSTRUCT;
	
	typedef struct
	{
		uint8_t on;
		uint8_t compRoll;
		uint8_t compPitch;
		uint8_t reqAngle;
		uint8_t reqSpeed;
		uint8_t reqDir;
		uint8_t accRoll;
		uint8_t accPitch;
		uint8_t globalDir;
		uint8_t globalPOS;
		uint8_t localVel;
		uint8_t encoder;
		uint8_t M1PWM;
		uint8_t M2PWM;
		uint8_t M1RPM;
		uint8_t M2RPM;
	}csvSTRUCT;
	
	float PWM_A_filt[2] = {0,0};
	float PWM_B_filt[2] = {0,0};
	float PWMduty =			0;	//PWM duty cycle
	float PWMoffset =		0;	//difference between the two motors PWM for turning
	float PWMgain =			PWM_LPF_GAIN;	

	csvSTRUCT CSV = {false,false,false,false,false,false,false,false,false,false,false,false,false};
	
	motorSTRUCT motor1 = {0,0};
	motorSTRUCT motor2 = {0,0};
		
	robotStartBalanceSTRUCT	robotStart		= {false,AUTO}; 	
	odometrySTRUCT			robotOdometry	= {{0,0},0,0,{0,0},{0,0},DIRECTION_LPF_GAIN,VELOCITY_LPF_GAIN,0,0,0};
				
	accRawSTRUCT	accRaw		=	{0,0,0};
	accAngleSTRUCT	accAngle	=	{0,0};
	gyroRawSTRUCT	gyroRaw		=	{0,0,0};
	eulerSTRUCT		eulerAngVel	=	{0,0,0};
	compSTRUCT		compAngle	=	{0,0,0,COMP_ANGLE_GAIN};
				
	float dt = 0;				//delta time for PID calculations
	

	//If the robot tilt PID's derivative tag (robotTiltPID.D) calculated from the input(the measured angle) then we get rid of the derivative spike
	//We can also use the FROM_EXTERNAL_SOURCE macro, if the input is too noisy.
	//In that case derivative will calculated from the gyroscope's angular velocity which is the derivative angle	
	pidSTRUCT		robotVelocityPID	= {
										VELOCITY_KP,
										VELOCITY_KP,
										VELOCITY_KI,
										VELOCITY_KI,
										VELOCITY_KD,
										VELOCITY_KD,
										VELOCITY_D_MES_TYPE,
										{
											VELOCITY_WIND_UMIN,
											VELOCITY_WIND_UMAX,
											VELOCITY_WIND_MAX_I_CHANGE,
											VELOCITY_WIND_IMAX,
											VELOCITY_WIND_FASTRESET,
											VELOCITY_WIND_KC,
											VELOCITY_WIND_KC,
											VELOCITY_WIND_ERROR_LIMIT_KC,
										},0,0,0,0,0
	};
	
	pidSTRUCT		robotTiltPID		= {
										TILT_KP,
										TILT_KP,
										TILT_KI,
										TILT_KI,
										TILT_KD,
										TILT_KD,
										TILT_D_MES_TYPE,
										{
											TILT_WIND_UMIN,
											TILT_WIND_UMAX,
											TILT_WIND_MAX_I_CHANGE,
											TILT_WIND_IMAX,
											TILT_WIND_FASTRESET,
											TILT_WIND_KC,
											TILT_WIND_KC,
											TILT_WIND_ERROR_LIMIT_KC,
										},0,0,0,0,0
	};
	
	pidSTRUCT		robotOrientationPID = {
										ORIENTATION_KP,
										ORIENTATION_KP,
										ORIENTATION_KI,
										ORIENTATION_KI,
										ORIENTATION_KD,
										ORIENTATION_KD,
										ORIENTATION_D_MES_TYPE,
										{
											ORIENTATION_WIND_UMIN,
											ORIENTATION_WIND_UMAX,
											ORIENTATION_WIND_MAX_I_CHANGE,
											ORIENTATION_WIND_IMAX,
											ORIENTATION_WIND_FASTRESET,
											ORIENTATION_WIND_KC,
											ORIENTATION_WIND_KC,
											ORIENTATION_WIND_ERROR_LIMIT_KC,
										},0,0,0,0,0
	};
	
	pidSTRUCT		robotTurningRatePID = {
										TURNING_KP,
										TURNING_KP,
										TURNING_KI,
										TURNING_KI,
										TURNING_KD,
										TURNING_KD,
										TURNING_D_MES_TYPE,
										{
											TURNING_WIND_UMIN,
											TURNING_WIND_UMAX,
											TURNING_WIND_MAX_I_CHANGE,
											TURNING_WIND_IMAX,
											TURNING_WIND_FASTRESET,
											TURNING_WIND_KC,
											TURNING_WIND_KC,
											TURNING_WIND_ERROR_LIMIT_KC,
										},0,0,0,0,0
									};
	
	
	DDRB |= 1<<PB5;
	PORTB &= ~(1<<PB5);
		
	//Disable JTag	
	MCUCSR |=1<<JTD;
	MCUCSR |=1<<JTD;
	
	//------TIMER for main loop timing--------------
	//with prescale 8
	//At 8MHz 0.01s = 10 000 unit
	TCCR1A=0X00;
	TCCR1B |=1<<CS11;
	
	//--------------TIMER for PWM-------------------
	//PIN 0C0-OC2 inverted FAST PWM with prescale 1. This means a ~30kHz signal to the motor controller.
	TCCR2 |=1<<WGM21 | 1<<WGM20 | 1<<COM21 | 1<<CS20;
	TCCR0 |=1<<WGM01 | 1<<WGM00 | 1<<COM01 | 1<<CS00;
	OCR2 = 0;
	OCR0 = 0;
	
	uart_init(UART_BAUD_SELECT(38400,8000000L));
	
	//--------------ADC for Encoder-----------------
	//IR LED encoder the signal is noisy so I can't use an external interrupt.
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1;	
	ADMUX |= 1<<REFS0;				
	ADCSRA |= 1<<ADIE;				
	ADCSRA |= 1<<ADEN;				
		
	sei();							//Enable global interrupts
	
	ADCSRA |= 1<<ADSC;				//Start ADC	
		
	//----------------Init MPU6050------------------
 	i2c_init();
	MPU6050_DDR |= 1<<MPU6050_AD0;
	MPU6050_PORT &= ~(1<<MPU6050_AD0);
	MPU6050_write(MPU6050_PWR_MGMT_1,0);
	MPU6050_write(MPU6050_GYRO_CONFIG,0b00001000);
	
	INIT_MotorController();
	
	_delay_ms(2000); //wait ford the MPU6050 to turn on.
	accRaw.x = MPU6050_read_ACC(MPU6050_ACC_X)-MPU6050_ACC_X_AVG;
	accRaw.y = MPU6050_read_ACC(MPU6050_ACC_Y)-MPU6050_ACC_Y_AVG;
	accRaw.z = -MPU6050_read_ACC(MPU6050_ACC_Z)-MPU6050_ACC_Z_AVG;
	compAngle.roll	= atan2(accRaw.z,accRaw.y)*RAD_to_DEG;
	compAngle.pitch	= atan2(accRaw.x,hypotf(accRaw.y,accRaw.z))*RAD_to_DEG;
	
	//----------------Main loop---------------------
	
	//float euyaw = 0;
    while(1)
    {
		if (TCNT1>=loopTIME)
		{			
			dt = (float)TCNT1/1000000;
			TCNT1 = 0;
					
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				motor1.counter = encoder1.counter;
				motor2.counter = encoder2.counter;
				encoder1.counter = 0;
				encoder2.counter = 0;
			}

			//Note that the accRaw.z's sign is changed
			accRaw.x = MPU6050_read_ACC(MPU6050_ACC_X)-MPU6050_ACC_X_AVG;
			accRaw.y = MPU6050_read_ACC(MPU6050_ACC_Y)-MPU6050_ACC_Y_AVG;
			accRaw.z = -MPU6050_read_ACC(MPU6050_ACC_Z)-MPU6050_ACC_Z_AVG;

			//I needed to swap gyroRaw.y and gyroRaw.z because of the gyro placement
			//with this modification I was able to use the usual Euler angle frame setup in the calcEulerAngles() like in this document:
			//http://www.chrobotics.com/library/understanding-euler-angles
			//I also change the sign of the gyroRaw.z and the accRaw.z to match with the example in the document
			gyroRaw.x = (MPU6050_read_GYRO(MPU6050_GYRO_ROLL)-MPU6050_GYRO_ROLL_AVG)/MPU6050_ScaleFactor*dt;
			gyroRaw.y = (MPU6050_read_GYRO(MPU6050_GYRO_PITCH)-MPU6050_GYRO_PITCH_AVG)/MPU6050_ScaleFactor*dt;
			gyroRaw.z = -(MPU6050_read_GYRO(MPU6050_GYRO_YAW)-MPU6050_GYRO_YAW_AVG)/MPU6050_ScaleFactor*dt;
			
			
			
			accAngle.roll	= atan2(accRaw.z,accRaw.y)*RAD_to_DEG;
			accAngle.pitch	= atan2(accRaw.x,hypotf(accRaw.y,accRaw.z))*RAD_to_DEG;	//-ACC_PITCH_OFFSET
			
			eulerAngVel		= calcEulerAngles(&gyroRaw,&compAngle);
			compAngle.pitch = compFilter(eulerAngVel.derivatedPitch,accAngle.pitch,compAngle.pitch,compAngle.gain);
			compAngle.roll	= compFilter(eulerAngVel.derivatedRoll,accAngle.roll,compAngle.roll,compAngle.gain);
			
			
			requested.direction += odometryCalc(motor1.counter,motor2.counter,&robotOdometry);
			motor1.RPM = getRPM(motor1.counter,dt);
			motor2.RPM = getRPM(motor2.counter,dt);
			
			
			
			//--------------Emergency STOP---------------
			if ((robotStart.started == true && (!Between(compAngle.pitch,-TURN_OFF_ANGLE,TURN_OFF_ANGLE,0) && robotStart.startType != MANUAL_START)) || robotStart.startType == MANUAL_STOP )
			{				
				robotStart.started = false;
				Motor_controll_PORT &= ~(1<<STBY);
				resetPID(&robotVelocityPID);
				resetPID(&robotTiltPID);
				resetPID(&robotOrientationPID);
//				resetPID(&robotTurningRatePID);
			}
		
			//--------------Prepare to START-------------
			if ((robotStart.started == false && Between(compAngle.pitch,-TURN_ON_ANGLE,TURN_ON_ANGLE,0) && robotStart.startType != MANUAL_STOP) || robotStart.startType == MANUAL_START)
			{
				if (robotStart.startType == MANUAL_START) robotStart.startType = AUTO;
				robotStart.started = true;				
				//zeroing starting parameters (odometry etc..)
				odometryReset(&robotOdometry);
				requested.turningRate = 0;
				requested.velocity = 0;
				requested.filtVelocity[0] = 0;
				requested.filtVelocity[1] = 0;
				requested.direction = robotOdometry.direction;
				requested.position = robotOdometry.position;
				Motor_controll_PORT |= (1<<STBY);
				
			}
		
			//--------------Start balancing--------------
			//This is the actual balancing note that the odometry calculations are already made.
			if (robotStart.started == true && robotStart.startType !=MANUAL_STOP )
			{
				manageBehavior(&robotOdometry,&robotVelocityPID,&robotTiltPID,&robotTurningRatePID);
				
				//PID functions and motor controller. The actual balancing.
				LPfilter(requested.velocity,requested.filtVelocity,requested.velLPFgain);
				LPfilter(requested.turningRate,requested.filtTurningRate,requested.turnLPFgain);
				requested.angle = -updatePID(&robotVelocityPID,robotOdometry.filtVelocity[0],requested.filtVelocity[0],dt);
				PWMduty	= updatePID(&robotTiltPID,compAngle.pitch,requested.angle,dt);
				
				//robotOrientationPID.D = robotOrientationPID.Kd * robotOdometry.filtDirectionChange[0]/dt;
 				if (requested.turningRate == 0)PWMoffset = updatePID(&robotOrientationPID,robotOdometry.direction,requested.direction,dt);
 				else PWMoffset = updatePID(&robotTurningRatePID,robotOdometry.filtDirectionChange[0],requested.filtTurningRate[0],dt);
				
				LPfilter((PWMduty+PWMoffset),PWM_A_filt,PWMgain);
				LPfilter((PWMduty-PWMoffset),PWM_B_filt,PWMgain);
				driveMotor(PWM_A_filt[0],PWM_B_filt[0]);
			}
			if (CSV.on)
			{	
				if (CSV.compPitch)
				{
					USART_Transmit_int(compAngle.pitch);
					uart_putc(',');
				}
				if (CSV.compRoll)
				{
					USART_Transmit_int(compAngle.roll);
					uart_putc(',');
				}
				if (CSV.accPitch)
				{
					USART_Transmit_int(accAngle.pitch);
					uart_putc(',');
				}				
				if (CSV.accRoll)
				{
					USART_Transmit_int(accAngle.roll);
					uart_putc(',');
				}
				if (CSV.reqAngle)
				{
					USART_Transmit_int(requested.angle);
					uart_putc(',');
				}
				if (CSV.reqSpeed)
				{
					USART_Transmit_int(requested.velocity);
					uart_putc(',');
				}		
				if (CSV.reqDir)
				{
					USART_Transmit_int(requested.direction);
					uart_putc(',');
				}
				if (CSV.globalDir)
				{
					USART_Transmit_int(robotOdometry.direction);
					uart_putc(',');
				}	
				if (CSV.globalPOS)
				{
					USART_Transmit_int(robotOdometry.position.x);
					uart_putc(',');
					USART_Transmit_int(robotOdometry.position.y);
					uart_putc(',');
				}	
				if (CSV.localVel)
				{
					USART_Transmit_float(robotOdometry.filtVelocity[0]);
					uart_putc(',');
				}	
				if (CSV.encoder)
				{
					USART_Transmit_int(robotOdometry.enc1);
					uart_putc(',');
					USART_Transmit_int(robotOdometry.enc2);
					uart_putc(',');
				}
				if (CSV.M1PWM)
				{
					USART_Transmit_int(PWM_A_filt[0]);
					uart_putc(',');
				}
				if (CSV.M2PWM)
				{
					USART_Transmit_int(PWM_A_filt[0]);
					uart_putc(',');
				}					
				if (CSV.M1RPM)
				{
					USART_Transmit_int(motor1.RPM);
					uart_putc(',');
				}
				if (CSV.M2RPM)
				{
					USART_Transmit_int(motor2.RPM);
					uart_putc(',');
				}										
				uart_putc(0x0A);
			}			
			else if (wp.patrolStatus == ONTARGET)
			{
				uart_putc(0x31);
				uart_putc(0x0A);
				wp.patrolStatus = MESSAGE_SENT;
			}
			else if (robotStart.started == 0)
			{
				uart_putc(0x32);
				uart_putc(0x0A);
			}
		}
 		
 		//-----------------------Finalize ASCII commands---------------------------
 		uint16_t BufferData = uart_getc();
		if (BufferData >> 8 == 0) commandHandler(&lastCommand,BufferData & 0xff);
		if (lastCommand.state == ENDED)
		{
			//---------------------------requests----------------------------------			
			if (strcmp((const char*)lastCommand.name,Command_RequestedSpeed_FromRC)==0)				requested.velocity = atof((const char*)lastCommand.value1)/10;
			else if (strcmp((const char*)lastCommand.name,Command_RequestedDirection)==0)			requested.turningRate = atof((const char*)lastCommand.value1)*11/100;
			else if (strcmp((const char*)lastCommand.name,Command_RequestedGlobalDirection)==0)		requested.direction = atoi((const char*)lastCommand.value1);			
			//else if (strcmp((const char*)lastCommand.name,Command_RequestedTurningRate)==0)		requested.turningRate=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_BalanceStart)==0)					robotStart.startType=atoi((const char*)lastCommand.value1);			
			else if (strcmp((const char*)lastCommand.name,Command_RequestedMaximumSpeed)==0)		robotBehavior.maxSpeed=atof((const char*)lastCommand.value1);			
			else if (strcmp((const char*)lastCommand.name,Command_SetBehavior)==0)					requested.behavior=atoi((const char*)lastCommand.value1);			
			else if (strcmp((const char*)lastCommand.name,Command_Ready)==0)
			{
				if (requested.controllMode == PC) requested.controllMode = REMOTE;
				robotBehavior.loopCounter = 0;
			}
			else if (strcmp((const char*)lastCommand.name,Command_SET_GOTOgxy)==0)
			{
				requested.behavior = BEHAVIOR_GOTOXY;
				destinationForGOTOxy.x=atoi((const char*)lastCommand.value1);
				destinationForGOTOxy.y=atoi((const char*)lastCommand.value2);
				wp.patrolStatus = UNDERWAY;
			}
			else if (strcmp((const char*)lastCommand.name,Command_SET_GOTOlxy)==0)
			{
				positionSTRUCT reqLocalPos;
				requested.behavior = BEHAVIOR_GOTOXY;
				reqLocalPos.x=atoi((const char*)lastCommand.value1);
				reqLocalPos.y=atoi((const char*)lastCommand.value2);
				destinationForGOTOxy = calcLocalXY(reqLocalPos,robotOdometry);				
				wp.patrolStatus = UNDERWAY;	
			}
				//********************WP*******************************
			else if (strcmp((const char*)lastCommand.name,Command_PatrolLoop)==0)					wp.patrolType=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_PatrolSpeed)==0)
			{
				wp.patrolVelocity = fabs(atof((const char*)lastCommand.value1)/10);
				if (wp.patrolVelocity == 0)
				{
					ResetWP();
					requested.behavior = BEHAVIOR_STAY;
				}
				else
				{
					if (wp.nextWP > wp.stopWP)
					{
						wp.nextWP = wp.stopWP;
						wp.lastWp = wp.stopWP;
					}
				}
			}
			else if (strcmp((const char*)lastCommand.name,Command_SET_wp)==0)
			{
				if (requested.behavior != BEHAVIOR_PATROL && wp.stopWP < MAX_WP-1 && wp.nextWP<=MAX_WP-1)
				{
					wp.wpSET = true;
					wp.positionList[wp.nextWP].x = robotOdometry.position.x;
					wp.positionList[wp.nextWP].y = robotOdometry.position.y;
					wp.patrolStatus = UNDERWAY;
					wp.stopWP = wp.nextWP;
					wp.nextWP = constrain(wp.nextWP+1,0,MAX_WP-1);
				}
			}
			//-------------------------------CSV-----------------------------------
			else if (strcmp((const char*)lastCommand.name,Command_CSV_ON)==0)						CSV.on=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Comp_pitch)==0)			CSV.compPitch=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Comp_roll)==0)			CSV.compRoll=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Acc_pitch)==0)			CSV.accPitch=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Acc_roll)==0)				CSV.accRoll=atoi((const char*)lastCommand.value1);

							
			else if (strcmp((const char*)lastCommand.name,Command_Display_Req_angle)==0)			CSV.reqAngle=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Req_speed)==0)			CSV.reqSpeed=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Req_dir)==0)				CSV.reqDir=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Global_Dir)==0)			CSV.globalDir=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Global_POS)==0)			CSV.globalPOS=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Local_vel)==0)			CSV.localVel=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_Encoder)==0)				CSV.encoder=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_M1_PWM)==0)				CSV.M1PWM=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_M2_PWM)==0)				CSV.M2PWM=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_M1_RPM)==0)				CSV.M1RPM=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Display_M2_RPM)==0)				CSV.M2RPM=atoi((const char*)lastCommand.value1);
			//---------------------complementary filter gain-----------------------					
			else if (strcmp((const char*)lastCommand.name,Command_Set_CompGain)==0)					compAngle.gain=atof((const char*)lastCommand.value1);
			//-----------------------------PID-------------------------------------
			//-----robotVelocityPID
			else if (strcmp((const char*)lastCommand.name,Command_Set_WheelvelKP)==0)				robotVelocityPID.KpBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_WheelvelKI)==0)				robotVelocityPID.KiBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_Umin)==0)		robotVelocityPID.windup.Umin=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_Umax)==0)		robotVelocityPID.windup.Umax=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_maxIchange)==0)	robotVelocityPID.windup.maxIchange=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_Imax)==0)		robotVelocityPID.windup.Imax=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_fastReset)==0)	robotVelocityPID.windup.fastReset=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Wheelvel_wind_KC)==0)			robotVelocityPID.windup.KcBase=atof((const char*)lastCommand.value1);
			//-----robotTiltPID
			else if (strcmp((const char*)lastCommand.name,Command_Set_AngleKP)==0)					robotTiltPID.KpBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_AngleKI)==0)					robotTiltPID.KiBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_AngleKD)==0)					robotTiltPID.KdBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_Umin)==0)			robotTiltPID.windup.Umin=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_Umax)==0)			robotTiltPID.windup.Umax=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_maxIchange)==0)	robotTiltPID.windup.maxIchange=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_Imax)==0)			robotTiltPID.windup.Imax=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_fastReset)==0)		robotTiltPID.windup.fastReset=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Angle_wind_KC)==0)			robotTiltPID.windup.KcBase=atof((const char*)lastCommand.value1);
			//-------robotOrientationPID
			else if (strcmp((const char*)lastCommand.name,Command_SET_OrientKP)==0)					robotOrientationPID.KpBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_OrientKI)==0)					robotOrientationPID.KiBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_OrientKD)==0)					robotOrientationPID.KdBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_Umin)==0)			robotOrientationPID.windup.Umin=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_Umax)==0)			robotOrientationPID.windup.Umax=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_maxIchange)==0)	robotOrientationPID.windup.maxIchange=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_Imax)==0)			robotOrientationPID.windup.Imax=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_fastReset)==0)	robotOrientationPID.windup.fastReset=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Orient_wind_KC)==0)			robotOrientationPID.windup.KcBase=atof((const char*)lastCommand.value1);
			//-------robotTurningRatePID
			else if (strcmp((const char*)lastCommand.name,Command_SET_TurningKP)==0)				robotTurningRatePID.KpBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_TurningKI)==0)				robotTurningRatePID.KiBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_TurningKD)==0)				robotTurningRatePID.KdBase=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_Umin)==0)		robotTurningRatePID.windup.Umin=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_Umax)==0)		robotTurningRatePID.windup.Umax=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_maxIchange)==0)	robotTurningRatePID.windup.maxIchange=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_Imax)==0)		robotTurningRatePID.windup.Imax=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_fastReset)==0)	robotTurningRatePID.windup.fastReset=atoi((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_Set_Turning_wind_KC)==0)			robotTurningRatePID.windup.KcBase=atof((const char*)lastCommand.value1);
			//--------Position		
			else if (strcmp((const char*)lastCommand.name,Command_SET_PositionKP)==0)				robotBehavior.posKP=atof((const char*)lastCommand.value1);
			//----------------------------LPF---------------------------------------------			
			else if (strcmp((const char*)lastCommand.name,Command_Set_vel_LPF_gain)==0)				robotOdometry.velocityLPFgain=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_Turning_LPFgain)==0)			robotOdometry.directionLPFgain=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_PWM_LPFgain)==0)				PWMgain=atof((const char*)lastCommand.value1);
			else if (strcmp((const char*)lastCommand.name,Command_SET_reqvel_LPFgain)==0)			requested.velLPFgain=atof((const char*)lastCommand.value1);
			
			else if (strcmp((const char*)lastCommand.name,Command_Set_Payload)==0)
			{
				robotBehavior.payload=atoi((const char*)lastCommand.value1);
				if (robotBehavior.payload>0)
				{
					robotVelocityPID.KpBase = VELOCITY_KP*linearBetween(robotBehavior.payload,10,0,0.8,1);
					robotVelocityPID.KiBase = VELOCITY_KI * linearBetween(robotBehavior.payload,10,0,0.6,1);
					robotVelocityPID.windup.KcBase = linearBetween(robotBehavior.payload,10,0,1,VELOCITY_WIND_MAX_I_CHANGE);					
					robotVelocityPID.windup.Umax = VELOCITY_WIND_UMAX * linearBetween(robotBehavior.payload,10,0,0.5,1);
					robotVelocityPID.windup.Umin = -robotVelocityPID.windup.Umax;
					robotTiltPID.KdBase = TILT_KD * linearBetween(robotBehavior.payload,10,0,2,1);
					robotTiltPID.KpBase = TILT_KP * linearBetween(robotBehavior.payload,0,10,1,1.5);
					robotBehavior.posKP = WHEELPOS_KP * linearBetween(robotBehavior.payload,10,0,0.6,1);										
				}
			}			
									
			else ;//send NACK
			lastCommand.state = PROCESSED;
		}
    }
}

ISR(ADC_vect)
{
	uint8_t LMB = ADCL;
	uint16_t ADCvalue = ADCH <<8 | LMB;
	switch(ADMUX)
	{
		case 0b01000000:
			encoder_checkSignal(ADCvalue,&encoder2.A);
			encoder_read(&encoder2);
			ADMUX = 0b01000001;
			break;
		
		case 0b01000001:
			encoder_checkSignal(ADCvalue,&encoder2.B);
			encoder_read(&encoder2);
			ADMUX = 0b01000010;
			break;
		
		case 0b01000010:
			encoder_checkSignal(ADCvalue,&encoder1.A);
			encoder_read(&encoder1);
			ADMUX = 0b01000011;
			break;
			
		case 0b01000011:
			encoder_checkSignal(ADCvalue,&encoder1.B);
			encoder_read(&encoder1);
			ADMUX = 0b01000000;			
			break;
	}
	ADCSRA |= 1<<ADSC;	//Start next conversation
}

// ISR(USART_RXC_vect)
// {
// 	uint8_t ReceiveByte = UDR;
// 	commandHandler(&lastCommand, ReceiveByte);
// }
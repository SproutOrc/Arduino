/*
 * balance.h
 *
 * Created: 2014.07.03. 12:01:16
 *  Author: F
 */ 


#ifndef BALANCE_H_
#define BALANCE_H_

#define loopTIME						10000	//10ms
#define Hz								100

#define VELOCITY_KP						2
#define VELOCITY_KI						1.64
#define VELOCITY_KD						0
#define VELOCITY_D_MES_TYPE				FROM_ERROR
#define VELOCITY_WIND_UMIN				-20
#define VELOCITY_WIND_UMAX				20
#define VELOCITY_WIND_MAX_I_CHANGE		6.6
#define VELOCITY_WIND_IMAX				0
#define VELOCITY_WIND_FASTRESET			DISABLE
#define VELOCITY_WIND_KC				7
#define VELOCITY_WIND_ERROR_LIMIT_KC	1.5

#define TILT_KP							40
#define TILT_KI							0
#define TILT_KD							-4.5		//-3
#define TILT_D_MES_TYPE					FROM_MEASURED_VALUE
#define TILT_WIND_UMIN					-255
#define TILT_WIND_UMAX					255
#define TILT_WIND_MAX_I_CHANGE			0
#define TILT_WIND_IMAX					0
#define TILT_WIND_FASTRESET				DISABLE
#define TILT_WIND_KC					0
#define TILT_WIND_ERROR_LIMIT_KC		0

#define ORIENTATION_KP					5
#define ORIENTATION_KI					0
#define ORIENTATION_KD					0.1
#define ORIENTATION_D_MES_TYPE			FROM_ERROR
#define ORIENTATION_WIND_UMIN			-135
#define ORIENTATION_WIND_UMAX			135
#define ORIENTATION_WIND_MAX_I_CHANGE	0
#define ORIENTATION_WIND_IMAX			0
#define ORIENTATION_WIND_FASTRESET		DISABLE
#define ORIENTATION_WIND_KC				0
#define ORIENTATION_WIND_ERROR_LIMIT_KC 0

#define TURNING_KP						120
#define TURNING_KI						10
#define TURNING_KD						1.2
#define TURNING_D_MES_TYPE				FROM_ERROR
#define TURNING_WIND_UMIN				-255
#define TURNING_WIND_UMAX				255
#define TURNING_WIND_MAX_I_CHANGE		0
#define TURNING_WIND_IMAX				0
#define TURNING_WIND_FASTRESET			ON
#define TURNING_WIND_KC					0
#define TURNING_WIND_ERROR_LIMIT_KC		0

#define WHEELPOS_KP						0.008
#define MAXIMUM_SPEED					3

#define WVelocity_Fastreset_deadspace	50
#define WVelocity_Fastreset_speedlim	2.5
#define PositionLargeCircle				200
#define PositionSmallCircle				30
#define CoopDirectionDeadSpace			8
#define CoopPositionDeadSpace			100

#define MAX_VELOCITY_FROM_RC			7.5
#define CoopMinimumSpeed				0.2
#define maxAUTOturn						15
#define TurningWithMaxSpeed				2.5		//above this speed turning rate is reduced
#define TurningWithMinSpeed				5.5		//above this speed turning rate is cap to a minimum of maxAUTOturn.
#define FastTurn						10		//The robot's velocity depend on the requested direction. Between FastTurn and SlowTurn change from max to 0
#define SlowTurn						60		//The robot's velocity depend on the requested direction. Between FastTurn and SlowTurn change from max to 0
#define LineLostAngle					75		//if the new direction is more then LineLostAngle then the robot will stop and only start again is the new direction is less then FastTurn 

//#define SPEED_TRANSITION_TIME			50		//Hundredth of a second		

#define	DIRECTION_LPF_GAIN				0.5		
#define	WHEEL_VELOCITY_LPF_GAIN			0.35		//0.45
#define	MOTOR_PWM_LPF_GAIN				0.2		//0.45
#define REQ_VEL_LPF_GAIN				0.05
#define REQ_TURN_LPF_GAIN				0.2

#define COMP_ANGLE_GAIN					0.996

#define ACC_PITCH_OFFSET				-1
	
#endif /* BALANCE_H_ */


/*
 * balance.h
 *
 * Created: 2014.07.03. 12:01:16
 *  Author: F
 */ 


// #define loopTIME						10000	//10ms
// #define Hz								100
// 
// #define VELOCITY_KP						2
// #define VELOCITY_KI						1.64
// #define VELOCITY_KD						0
// #define VELOCITY_D_MES_TYPE				FROM_ERROR
// #define VELOCITY_WIND_UMIN				-20
// #define VELOCITY_WIND_UMAX				20
// #define VELOCITY_WIND_MAX_I_CHANGE		4
// #define VELOCITY_WIND_IMAX				1.5
// #define VELOCITY_WIND_FASTRESET			DISABLE
// #define VELOCITY_WIND_KC				7
// 
// #define TILT_KP							40
// #define TILT_KI							0
// #define TILT_KD							-4.5		//-3
// #define TILT_D_MES_TYPE					FROM_MEASURED_VALUE
// #define TILT_WIND_UMIN					-255
// #define TILT_WIND_UMAX					255
// #define TILT_WIND_MAX_I_CHANGE			0
// #define TILT_WIND_IMAX					0
// #define TILT_WIND_FASTRESET				DISABLE
// #define TILT_WIND_KC					0
// 
// #define ORIENTATION_KP					8
// #define ORIENTATION_KI					1
// #define ORIENTATION_KD					-25
// #define ORIENTATION_D_MES_TYPE			FROM_EXTRNAL_SOURCE
// #define ORIENTATION_WIND_UMIN			-255
// #define ORIENTATION_WIND_UMAX			255
// #define ORIENTATION_WIND_MAX_I_CHANGE	0
// #define ORIENTATION_WIND_IMAX			0
// #define ORIENTATION_WIND_FASTRESET		DISABLE
// #define ORIENTATION_WIND_KC				0
// 
// #define TURNING_KP						100
// #define TURNING_KI						0
// #define TURNING_KD						4.5
// #define TURNING_D_MES_TYPE				FROM_ERROR
// #define TURNING_WIND_UMIN				-255
// #define TURNING_WIND_UMAX				255
// #define TURNING_WIND_MAX_I_CHANGE		0
// #define TURNING_WIND_IMAX				0
// #define TURNING_WIND_FASTRESET			DISABLE
// #define TURNING_WIND_KC					0
// 
// #define WHEELPOS_KP						0.008
// #define MAXIMUM_SPEED					3
// 
// #define CoopDirectionDeadSpace			8
// #define CoopPositionDeadSpace			100
// #define CoopMinimumSpeed				0.2
// 
// #define SPEED_TRANSITION_TIME			100		//Hundredth of a second		
// 
// #define	DIRECTION_LPF_GAIN				0.35		
// #define	WHEEL_VELOCITY_LPF_GAIN			0.35		//0.45
// #define	MOTOR_PWM_LPF_GAIN				0.2			//0.45
// 
// #define COMP_ANGLE_GAIN					0.997

/*
 * IncFile1.h
 *
 * Created: 2014.07.03. 9:43:48
 *  Author: F
 */ 


#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "lowPassFilter.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define Motor_controll_DDR	DDRC
#define Motor_controll_PORT	PORTC
#define Motor_controll_PIN	PINC

#define STBY				PC4		//Stendby
#define AIN1				PC3		//Motor A
#define AIN2				PC2		//Motor A
#define BIN1				PC5		//Motor B
#define BIN2				PC6		//Motor B
#define PWMA				PD7		//Motor 'A' PWM
#define PWMA_DDR			DDRD
#define PWMB				PB3		//Motor 'B' PWM
#define PWMB_DDR			DDRB

#define PWM_LPF_GAIN		MOTOR_PWM_LPF_GAIN


void driveMotor(int motorA,int motorB);
void INIT_MotorController(void);

void driveMotor(int motorA,int motorB)
{
	if (motorA>0)
	{
		//positive direction
		Motor_controll_PORT |= 1<<AIN1;
		Motor_controll_PORT &= ~(1<<AIN2);
		OCR2 = constrain(motorA,0,255);
	}
	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value)
	else if (motorA<=0)
	{
		//negative direction
		Motor_controll_PORT |= 1<<AIN2;
		Motor_controll_PORT &= ~(1<<AIN1);
		OCR2 = constrain(abs(motorA),0,255);
 	}
	if (motorB>0)
	{	
		//positive direction	
		Motor_controll_PORT |= 1<<BIN1;
		Motor_controll_PORT &= ~(1<<BIN2);
		OCR0 = constrain(motorB,0,255);
	}
// 	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value) 
 	else if (motorB<=0)
	{
		//negative direction
		Motor_controll_PORT |= 1<<BIN2;
		Motor_controll_PORT &= ~(1<<BIN1);
		OCR0 = constrain(abs(motorB),0,255);
	}	
}

void INIT_MotorController(void)
{
	Motor_controll_DDR |= 1<<STBY | 1<<AIN1 | 1<<AIN2 | 1<<BIN1 | 1<<BIN2;
	Motor_controll_PORT|= 1<<STBY;
	PWMA_DDR |= 1<<PWMA;
	PWMB_DDR |= 1<<PWMB;
}
#endif /* INCFILE1_H_ */
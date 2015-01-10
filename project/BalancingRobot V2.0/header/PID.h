/*
 * balance.h
 *
 * Created: 2014.07.02. 14:36:12
 *  Author: F
 */ 

//******************Basic PID looks like this********************
// typedef struct
// {
// 	float Kp;
// 	float Ki;
// 	float Kd;
// 	float P;
// 	float I;
// 	float D;
// 	float lastError;
// 	float errorSum;
// }pidSTRUCT;
// 
// float updatePID(pidSTRUCT *PID, float measuredValue, float setpoint, float dt);
// 
// float updatePID(pidSTRUCT *PID, float measuredValue, float setpoint, float dt)
// {
// 	float error = measuredValue - setpoint;
// 	
// 	errorSum +=errorSum;
// 	
// 	PID->P = PID->Kp*error;
// 	PID->I = PID->Ki*errorSum*dt;
// 	PID->D = PID->Kd*(error - PID->lastError)/dt;
// 	
// 	PID->lastError = error;
// 	return PID->P+PID->I+PID->D;
// }
//****************************************************************

#ifndef PID_H_
#define PID_H_

typedef enum{
	FROM_ERROR			= 0,
	FROM_MEASURED_VALUE = 1,
	FROM_EXTRNAL_SOURCE = 2,
}dSource;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// typedef const struct
// {
// }
typedef enum{
	DISABLE=0,
	ON=1,
	OFF=2,
}fastReset;

typedef struct
{	
	float Umin;
	float Umax;
	float maxIchange;	//The maximum value whereby I(integrator) can change under one second before multiplied by Ki
	float Imax;
	fastReset fastReset;
	float Kc;
	float KcBase;
	float fastresetErrorLimit;
} antiWindupSTRUCT;

struct pidSTRUCT_S
{
	float Kp;
	float KpBase;
	float Ki;			//If Ki = 0 then we wouldn't increment the errorSum
	float KiBase;
	float Kd;
	float KdBase;
	dSource derivativeCalcType;
	antiWindupSTRUCT windup;	
	float P;
	float I;
	float D;
	float lastError;
	float lastInput;
	float errorSum;			
};
	
typedef struct pidSTRUCT_S pidSTRUCT;



//return with the calculated P+I+D parameter
float updatePID(pidSTRUCT *PID, float measuredValue, float setpoint, float dt);

//Reset the PID's last error, errorSum parameters
void resetPID(pidSTRUCT *PID);

float updatePID(pidSTRUCT *PID, float measuredValue, float setpoint, float dt)
{
	float error =  setpoint - measuredValue;
	float errorInc = 0;		//we will increment the errorSum by this number
	float PIDoutput = 0;
	
	//If Ki = 0 then we wouldn't increment the errorSum
	if (PID->Ki == 0) PID->errorSum = 0;
	else
	{
		 errorInc = error;
		//If maxIchange != 0 then the errorSum increment saturated by maxIchange
		if (PID->windup.maxIchange !=0) 
		{
			float normalisedMaxIchange = PID->windup.maxIchange/PID->Ki;
			errorInc = constrain(error,-normalisedMaxIchange,normalisedMaxIchange);
		}

		//If Integrator fast reset is allowed then rapidly decrease errorSum if its sign is the opposite then the error's sign
		if (((PID->errorSum < 0 && 0 < error) || (error < 0 && 0 < PID->errorSum)) && fabs(error) > PID->windup.fastresetErrorLimit && PID->windup.fastReset == ON)
		{
			errorInc = error * PID->windup.Kc;
		}
	}
		
	//The maximum value that Imax can get.
	if (PID->windup.Imax != 0)	
	{
		float normalisedImax = PID->windup.Imax/PID->Ki/dt;
		PID->errorSum = constrain(PID->errorSum+errorInc,-normalisedImax,normalisedImax);
	}
	else PID->errorSum += errorInc;		
	
	PID->P = PID->Kp*error;
	PID->I = PID->Ki*PID->errorSum*dt;
	switch (PID->derivativeCalcType)
	{
	case FROM_ERROR:
		PID->D = PID->Kd*(error - PID->lastError)/dt;
		break;
	case FROM_MEASURED_VALUE:
		PID->D = PID->Kd*(measuredValue - PID->lastInput)/dt;
		break;
	case FROM_EXTRNAL_SOURCE:
		//In this case PID->D will get value manualy from outside of this function	
		break;
	}
		
	PID->lastError = error;
	PID->lastInput = measuredValue;
	
	PIDoutput = PID->P+PID->I+PID->D;
	
	//If Umin != Umin then PID will be saaturated by Umin and Umax. 
	//undo the integration if it boost this situation.
	if(PID->windup.Umin != PID->windup.Umax)
	{
		if (PIDoutput > PID->windup.Umax)
		{
			PIDoutput = PID->windup.Umax;
			if(error > 0) PID->errorSum -= errorInc;
		}
		if (PIDoutput < PID->windup.Umin)
		{
			PIDoutput = PID->windup.Umin;
			if(error < 0) PID->errorSum -= errorInc;
		}		
	}// PIDoutput = constrain(PIDoutput,PID->windup.Umin,PID->windup.Umax);
	
	return PIDoutput;
}

void resetPID(pidSTRUCT *PID)
{
	PID->lastError = 0;
	PID->errorSum = 0;
}

#endif /* PID_H_ */
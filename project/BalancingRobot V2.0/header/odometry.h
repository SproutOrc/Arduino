/*
 * odometry.h
 *
 * Created: 2014.07.01. 15:01:21
 *  Author: F
 */ 


#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "lowPassFilter.h"

#define DirectionConst			0.26724 		//wheelDiameter / shaftLength
#define Encoder_to_mm			1.18682389		//wheelDiameter * PI / 180

#define	VELOCITY_LPF_GAIN		WHEEL_VELOCITY_LPF_GAIN				//wheel velocity low pass filter gain


typedef struct
{
	float x;
	float y;
}positionSTRUCT;

typedef struct
{
	positionSTRUCT position;
	float direction;
	float directionChange;		//shaft direction change.
	float filtDirectionChange[2];
	float filtVelocity[2];
	float directionLPFgain;
	float velocityLPFgain;
	int enc1;
	int enc2;
	float wheelPosition;
}odometrySTRUCT;



int		odometryCalc(float encoder1, float encoder2, odometrySTRUCT *rOdometry); //calculate position and returns with the direction overflow which must add to any direction change request.
void	odometryReset(odometrySTRUCT *rposition);
positionSTRUCT calcLocalXY(positionSTRUCT reqLocalPos, odometrySTRUCT rOdometry);
float	getDistFrom(positionSTRUCT target,positionSTRUCT current, float dist, float fi);
float	getDirectionToWP(positionSTRUCT new, positionSTRUCT current);
float	getNewDirectionFront(float newDir, float currentDir);
float	getNewDirectionShort(float newDir, float currentDir);

int odometryCalc(float encoder1, float encoder2, odometrySTRUCT *rOdometry)
{
	int overflow = 0;								//the global direction need to be internally changed by this number. 
	
	rOdometry->enc1 += encoder1;
	rOdometry->enc2 += encoder2;
	
	encoder1 = encoder1 * Encoder_to_mm;			//Convert encoder readings to mm
	encoder2 = encoder2 * Encoder_to_mm;			//Convert encoder readings to mm
	rOdometry->wheelPosition = (encoder1+encoder2)/2;
		
	rOdometry->directionChange = DirectionConst*(encoder1-encoder2);	
	LPfilter(rOdometry->directionChange,rOdometry->filtDirectionChange,rOdometry->directionLPFgain);
	LPfilter(((encoder1+encoder2)/2),rOdometry->filtVelocity,rOdometry->velocityLPFgain);
	
	rOdometry->direction += rOdometry->directionChange;			
	rOdometry->position.y += (encoder1+encoder2)/2 * cos(rOdometry->direction*DEG_to_RAD);
	rOdometry->position.x += (encoder1+encoder2)/2 * sin(rOdometry->direction*DEG_to_RAD);
	
	
	//restrict globalPosition.direction between -180 and 180 to get rid of possible overflow
	if (rOdometry->direction < -180)
	{
		overflow = 360;
		rOdometry->direction = rOdometry->direction+overflow;
	}
	else if (rOdometry->direction > 180)
	{
		overflow = -360;
		rOdometry->direction = rOdometry->direction+overflow;
	}	
	return overflow;
}

void odometryReset(odometrySTRUCT *rOdometry)
{
	rOdometry->direction = 0;
	rOdometry->position.x = 0;
	rOdometry->position.y = 0;
	rOdometry->filtVelocity[0] = 0;
	rOdometry->filtVelocity[1] = 0;
}

positionSTRUCT calcLocalXY(positionSTRUCT reqLocalPos, odometrySTRUCT rOdometry)
{
	positionSTRUCT reqGlobalPos;
	reqGlobalPos.x = rOdometry.position.x+sin(rOdometry.direction*DEG_to_RAD)*reqLocalPos.y+cos(rOdometry.direction*DEG_to_RAD)*reqLocalPos.x;
	reqGlobalPos.y = rOdometry.position.y+cos(rOdometry.direction*DEG_to_RAD)*reqLocalPos.y-sin(rOdometry.direction*DEG_to_RAD)*reqLocalPos.x;
	return reqGlobalPos;
}

float getDistFrom(positionSTRUCT new,positionSTRUCT current, float dist, float curentOrientation)
{	
	float alpha = 0;
	float beta = 0;

	alpha = atan2(fabs(current.x-new.x),fabs(current.y-new.y))*RAD_to_DEG;
	beta = alpha - curentOrientation;
	dist = fabs(dist * cos(beta*DEG_to_RAD));
	
	return dist;
}

float getDirectionToWP(positionSTRUCT new, positionSTRUCT current)
{
	float newDirection = 0;
	newDirection = atan2(new.x-current.x,new.y-current.y)*RAD_to_DEG;

	return newDirection;
}

//return with the shortest turn to the new direction if the maximum turn is 180 degree
float getNewDirectionFront(float newDir, float currentDir)
{
	if ((newDir-currentDir)>180 || ((newDir-currentDir)<0 && (newDir-currentDir)>-180))
	{
		if (newDir<currentDir) newDir -= currentDir;
		else newDir = newDir - 360 - currentDir;
	}
	else
	{
		if (newDir<currentDir) newDir = newDir + 360 - currentDir;
		else newDir -= currentDir;
	}
	
	return newDir;
}

//return with the shortest turn to the new direction if the maximum turn is 90 degree
//if this is subtract from the current position then the robot turn his back or face to the new direction. Depends on which direction has the smaller angle.
//So if the current direction is 100 degree and the new is 0 deg then this function returns with 80 deg so the new direction is 180 deg.
float getNewDirectionShort(float newDir, float currentDir)
{
    if ((newDir-currentDir)>180)
    {
	    if (newDir-currentDir <270) newDir = newDir - 180 - currentDir;
	    else newDir = newDir - 360 - currentDir;
    }
    else if ((newDir-currentDir)<0 && (newDir-currentDir)>-180)
    {
	    if (newDir-currentDir >= -90) newDir -= currentDir;
	    else newDir = 180 - (currentDir - newDir);
	    
    }
    else
    {
	    if (newDir<currentDir)
	    {
		    if (newDir-currentDir > -270) newDir = newDir + 180 - currentDir;
		    else newDir = newDir + 360 - currentDir;
	    }
	    else if (newDir-currentDir <= 90)newDir -= currentDir;
	    else newDir = newDir - 180 - currentDir;
    }
	return newDir;
}

uint8_t CheckPosition(positionSTRUCT requested, positionSTRUCT current, uint8_t deadzone)
{
	if (Between(current.x-requested.x,-deadzone,deadzone,0) && Between(current.y-requested.y,-deadzone,deadzone,0))return 1;
	else return 0;
}
#endif /* ODOMETRY_H_ */
/*
 * attitude.h
 *
 * Created: 2014.07.05. 22:00:17
 *  Author: F
 */ 


#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <stdbool.h>
#include "PID.h"
#include "odometry.h"
#include "balance.h"

#define BEHAVIOR_STAY	0
#define BEHAVIOR_COOP	1
#define BEHAVIOR_REMOTE	2
#define BEHAVIOR_PATROL 3
#define BEHAVIOR_GOTOXY 4

#define PC		0
#define REMOTE	1

typedef enum
{
	UNDERWAY = 0,
	ONTARGET = 1,
	MESSAGE_SENT = 2,
	DISABLED = 3,	
}pStatus;

#define MAX_WP		10

typedef struct
{	
	float	maxSpeed;
	float	posKP;
	uint8_t payload;
	uint8_t loopCounter;
	
}behaviorSTRUCT;

typedef struct
{
	bool	wpSET;	
	pStatus patrolStatus;
	uint8_t patrolType;	
	uint8_t nextWP;
	uint8_t lastWp;
	uint8_t stopWP;
	float	patrolVelocity;	
	positionSTRUCT positionList[MAX_WP];
}wpSTRUCT;


typedef struct  
{
	float distance;
	float direction;
	bool lost;
	bool lineLost;
}destinationSTRUCT;

typedef struct
{
	float	velocity;
	int		direction;
	int		directionRC;
	float	turningRate;
	float	angle;
	positionSTRUCT position;
	float	filtVelocity[2];
	float	filtTurningRate[2];
	float	velLPFgain;
	float	turnLPFgain;
	uint8_t prevBehavior;
	uint8_t behavior;
	uint8_t controllMode;
}RequestedSTRUCT;

RequestedSTRUCT			requested		= {0,0,0,0,0,{0,0},{0,0},{0,0},REQ_VEL_LPF_GAIN,REQ_TURN_LPF_GAIN,BEHAVIOR_STAY,BEHAVIOR_STAY,PC};
behaviorSTRUCT			robotBehavior	= {MAXIMUM_SPEED,WHEELPOS_KP,0,0};
wpSTRUCT				wp				= {false,DISABLED,0,0,0,0,0};
positionSTRUCT			destinationForGOTOxy = {0,0};

void manageBehavior(odometrySTRUCT *rOdometry, pidSTRUCT *velPID, pidSTRUCT *tiltPID, pidSTRUCT *tRatePID);

void	handleDist(odometrySTRUCT *rOdometry, positionSTRUCT destPos, destinationSTRUCT *wp, bool hasFront);
float	HandleTurnSpeed(destinationSTRUCT *destination, float velocity);
void	HandleWP(RequestedSTRUCT *requested);
void	ResetWP(void);
float	linearBetween(float x,float x1,float x2,float y1, float y2);

void manageBehavior(odometrySTRUCT *rOdometry, pidSTRUCT *velPID, pidSTRUCT *tiltPID, pidSTRUCT *tRatePID)
{
	static float originalDirectionAtHome = 0;
	static destinationSTRUCT destination;
	//static float lastTurningRate = 0;
	
	//constrain the critical parameters that may modified through the USART
	requested.velocity = constrain(requested.velocity,-MAX_VELOCITY_FROM_RC,MAX_VELOCITY_FROM_RC);
	wp.patrolVelocity = constrain(wp.patrolVelocity,-MAX_VELOCITY_FROM_RC,MAX_VELOCITY_FROM_RC);
	robotBehavior.payload = constrain(robotBehavior.payload,0,10);

	//I set the PID parameters to default that the behavior functions will modify.
	tiltPID->Kp = tiltPID->KpBase;
	tiltPID->Kd = tiltPID->KdBase;
	velPID->windup.Kc = velPID->windup.KcBase;
	if (!Between(rOdometry->filtVelocity[0],-WVelocity_Fastreset_speedlim,WVelocity_Fastreset_speedlim,0)) velPID->windup.fastReset = ON;
	else velPID->windup.fastReset = OFF;

	
	if (requested.controllMode == REMOTE)
	{
		robotBehavior.loopCounter++;
		if (robotBehavior.loopCounter>=20)
		{
			robotBehavior.loopCounter = 0;
			requested.behavior = BEHAVIOR_STAY;
			requested.controllMode = PC;
		}
	}
	
	if (requested.prevBehavior != requested.behavior)
	{
		requested.turningRate = 0;
		requested.velocity = 0;
		requested.direction = rOdometry->direction;	
		requested.position = rOdometry->position;	
		originalDirectionAtHome = rOdometry->direction;	
		requested.prevBehavior = requested.behavior;
		tRatePID->errorSum = 0;
	}
	switch (requested.behavior)
	{
		//The robot hold the requested.position. If it roll away then it try to go back.
		case BEHAVIOR_STAY:
			handleDist(rOdometry,requested.position,&destination,0);
			if (Between(destination.distance,-WVelocity_Fastreset_deadspace,WVelocity_Fastreset_deadspace,0))
			{
				float distABS = fabs(destination.distance);
				velPID->windup.Kc = linearBetween(distABS,WVelocity_Fastreset_deadspace/4,WVelocity_Fastreset_deadspace,1,velPID->windup.KcBase);
				if (Between(tiltPID->lastError,-1.5,1.5,0))
				{
					tiltPID->Kd = linearBetween(distABS,WVelocity_Fastreset_deadspace,0,tiltPID->KdBase,tiltPID->KdBase*1.8);
					tiltPID->Kp = linearBetween(distABS,WVelocity_Fastreset_deadspace,0,tiltPID->KpBase,tiltPID->KpBase*2);
				}
			}
			if (destination.lost == false) destination.direction = getNewDirectionFront(originalDirectionAtHome,rOdometry->direction);
			
			requested.direction =	rOdometry->direction+constrain(destination.direction,-maxAUTOturn,maxAUTOturn);
			requested.velocity =	HandleTurnSpeed(&destination,destination.distance * robotBehavior.posKP);
			requested.velocity =	constrain(requested.velocity,-robotBehavior.maxSpeed,robotBehavior.maxSpeed);
// 			USART_Transmit_int(originalDirectionAtHome);
// 			USART_Transmit(',');
		break;
		
		//The robot hold the requested.position. If it roll away then if its velocity drop below CoopMinimumSpeed then he try to hold its new position.
		case BEHAVIOR_COOP:
			if (Between(destination.distance,-WVelocity_Fastreset_deadspace,WVelocity_Fastreset_deadspace,0)) velPID->windup.fastReset = OFF;
			if (!Between(rOdometry->direction - requested.direction,-CoopDirectionDeadSpace,CoopDirectionDeadSpace,0)) requested.direction = rOdometry->direction;
			if (!Between(destination.distance, -CoopPositionDeadSpace,CoopPositionDeadSpace,0))
			{
				if(Between(rOdometry->filtVelocity[0],-CoopMinimumSpeed,CoopMinimumSpeed,0))
				{
					requested.position = rOdometry->position;
				}
			}
			else
			{
				float distABS = fabs(destination.distance);
				velPID->windup.Kc = linearBetween(distABS,WVelocity_Fastreset_deadspace/4,WVelocity_Fastreset_deadspace,1,velPID->windup.KcBase);
				if (velPID->windup.Kc < 1) velPID->windup.Kc = 1;
				if (Between(tiltPID->lastError,-1,1,0))
				{
					tiltPID->Kd = linearBetween(distABS,WVelocity_Fastreset_deadspace,0,tiltPID->KdBase,tiltPID->KdBase*1.5);
					tiltPID->Kp = linearBetween(distABS,WVelocity_Fastreset_deadspace,0,tiltPID->KpBase,tiltPID->KpBase*2);
				}
			}
			requested.velocity = -rOdometry->wheelPosition * robotBehavior.posKP;
			requested.velocity = constrain(requested.velocity,-robotBehavior.maxSpeed,robotBehavior.maxSpeed);
		break;		
		
		case BEHAVIOR_REMOTE:
			if ((requested.filtTurningRate[1] >= 0 && 0 >= requested.filtTurningRate[0]) || (requested.filtTurningRate[1] <= 0 && 0 <= requested.filtTurningRate[0])) tRatePID->errorSum = 0;
			requested.filtTurningRate[1] = requested.filtTurningRate[0];
			if (requested.turningRate != 0) requested.direction = rOdometry->direction;
			requested.turningRate = requested.turningRate*linearBetween(fabs(requested.velocity),TurningWithMinSpeed,TurningWithMaxSpeed,0.8,1); 		
		break;	
		
		//The robot will move between the waypoints
		case BEHAVIOR_PATROL:
			if (wp.wpSET)
			{
				if (CheckPosition(wp.positionList[wp.nextWP],rOdometry->position,PositionSmallCircle)) HandleWP(&requested);
				else
				{
					destination.direction	=	getDirectionToWP(wp.positionList[wp.nextWP],rOdometry->position);
					destination.direction	=	getNewDirectionFront(destination.direction,rOdometry->direction);
					requested.direction		=	rOdometry->direction+constrain(destination.direction,-maxAUTOturn,maxAUTOturn);
					requested.velocity		=	HandleTurnSpeed(&destination,wp.patrolVelocity);
				}
			}
			else
			{
				requested.behavior = BEHAVIOR_STAY;
				wp.patrolStatus = ONTARGET;
			}
		break;
		
		//The robot will go to the requested.destination
		case BEHAVIOR_GOTOXY:
			if (CheckPosition(destinationForGOTOxy,rOdometry->position,PositionSmallCircle))
			{
				requested.behavior = BEHAVIOR_STAY;
				wp.patrolStatus = ONTARGET;
			}
			else
			{
				destination.direction	=	getDirectionToWP(destinationForGOTOxy,rOdometry->position);
				destination.direction	=	getNewDirectionFront(destination.direction,rOdometry->direction);
				requested.direction		=	rOdometry->direction+constrain(destination.direction,-maxAUTOturn,maxAUTOturn);
				requested.velocity		=	HandleTurnSpeed(&destination,wp.patrolVelocity);
			}
		break;		
	}
}

//This function modifies the destination.distance and the requested.direction.
//destination.distance calculation has two different type:
//when the distance between the current position and the wp position is more then the PositionLargeCircle, then the destination.distance is the
//distance of the two point.
//when the distance between the current position and the wp position is less then the PositionSmallCircle, then the destination.distance is recalculated
//by the getDistFrom() function.
//
//The requested.direction is the difference between the current direction and the direction to the requested.position
//
//If the home position is behind us then the distant from home position is considered to negative.
void handleDist(odometrySTRUCT *rOdometry, positionSTRUCT destPos, destinationSTRUCT *destination, bool hasFront)
{
	float directionToWp		= 0;	//direction to wp from 0 deg. Value: (0-360)
	float directionToWp180	= 0;
	
	destination->distance		= hypotf(rOdometry->position.x-destPos.x,rOdometry->position.y-destPos.y);
	directionToWp				= getDirectionToWP(requested.position,rOdometry->position);
	directionToWp180			= getNewDirectionFront(directionToWp,rOdometry->direction);
	
	if		(destination->distance > PositionLargeCircle) destination->lost = true;
	else if (destination->distance < PositionSmallCircle) destination->lost = false;
	
	if (destination->lost)	
	{
		if (hasFront) destination->direction = directionToWp180;
		else destination->direction = getNewDirectionShort(directionToWp,rOdometry->direction);		
	}
	else destination->distance = getDistFrom(destPos,rOdometry->position,destination->distance,rOdometry->direction);
		
	//If the home position is behind us then the distant from home position is considered to negative.
	if (!Between(directionToWp180,-90,90,0)) destination->distance = -destination->distance;
}

//Constrain the robot's velocity when it turning.
float HandleTurnSpeed(destinationSTRUCT *destination, float velocity)
{
	if (destination->lineLost == false)
	{
		if (!Between(destination->direction,-LineLostAngle,LineLostAngle,0))
		{
			destination->lineLost=true;
			velocity = 0;
		}
		else velocity = velocity*linearBetween(destination->direction,SlowTurn,FastTurn,0,1);
	}
	else if (destination->direction<FastTurn && destination->direction>-FastTurn) destination->lineLost = false;
	return velocity;
}


void HandleWP(RequestedSTRUCT *requested)
{
	switch (wp.patrolType)
	{
		//elmegy a következõ wp-ig majd pozíció tartásra kapcsol, és lépteti eggyel a wp számozást
		case 0:
		requested->behavior = BEHAVIOR_STAY;
		wp.patrolStatus = ONTARGET;
		if ((wp.nextWP>wp.lastWp || wp.nextWP == 0) && wp.nextWP < wp.stopWP)
		{
			wp.lastWp = wp.nextWP;
			wp.nextWP++;
		}
		else if (wp.nextWP<wp.lastWp || wp.nextWP >= wp.stopWP)
		{
			if (wp.stopWP == 0)
			{
				requested->behavior = BEHAVIOR_STAY;
				wp.patrolStatus = ONTARGET;
			}
			else
			{
				wp.lastWp = wp.nextWP;
				wp.nextWP--;
			}
		}
		break;
		//az utolsó wp után az elsõ következik
		case 1:
		if (wp.nextWP>=wp.stopWP)
		{
			wp.nextWP = 0;
			if (wp.stopWP == 0)
			{
				requested->behavior = BEHAVIOR_STAY;
				wp.patrolStatus = ONTARGET;
			}
		}
		else wp.nextWP++;
		break;
		//az utolsó wp után elindul visszafele a wp-ken
		case 2:
		if ((wp.nextWP>wp.lastWp || wp.nextWP == 0) && wp.nextWP < wp.stopWP)
		{
			wp.lastWp = wp.nextWP;
			wp.nextWP++;
		}
		else if (wp.nextWP<wp.lastWp || wp.nextWP >= wp.stopWP)
		{
			if (wp.stopWP == 0)
			{
				requested->behavior = BEHAVIOR_STAY;
				wp.patrolStatus = ONTARGET;
			}
			else
			{
				wp.lastWp = wp.nextWP;
				wp.nextWP--;
			}
		}
		break;
	}
}

void ResetWP(void)
{
	wp.wpSET = false;
	wp.nextWP = 0;
	wp.stopWP = 0;
	wp.lastWp = 0;
	wp.positionList[wp.nextWP].x = 0;
	wp.positionList[wp.nextWP].y = 0;	
	destinationForGOTOxy.x = 0;
	destinationForGOTOxy.y = 0;
}

float linearBetween(float x,float x0,float x1,float y0, float y1)
{
	float y;
	y = y0+((x-x0)*(y1-y0))/(x1-x0);
	
	y = constrain(y,y0,y1);
	return y;
}

#endif /* BEHAVIOR_H_ */
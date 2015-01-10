/*
 * chechkBetween.h
 *
 * Created: 2014.07.07. 15:57:49
 *  Author: F
 */ 


#ifndef CHECKBETWEEN_H_
#define CHECKBETWEEN_H_

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

uint8_t Between(float checkable, float min, float max, uint8_t include);
uint8_t Between(float checkable, float min, float max, uint8_t include)
{
	if (include)
	{
		if (checkable <= max && checkable >= min) return 1;
		else return 0;
	}
	else
	{
		if (checkable < max && checkable > min) return 1;
		else return 0;
	}
	
}


#endif /* CHECHKBETWEEN_H_ */
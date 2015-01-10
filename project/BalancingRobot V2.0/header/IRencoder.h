/*
 * Encoder.h
 *
 * Created: 2014.06.30. 13:19:44
 *  Author: F
 */ 

//**************************************************************************************************************************
//***********************************************IR mouse ball encoder******************************************************
//
// The IR encoder don't give a clean square signal like the hall encoder.
// If you don't want to use other electrical components then use the ADC to "convert" the analog signal to a square signal
// If the ADC gives a number that >=X then it is a HIGH state, if gives a number that <=Y then it is a LOW state.
// the space between X and Y is the dead space. without that you get a bouncing signal.
//
//**************************************************************************************************************************
#ifndef IRENCODER_H_
#define IRENCODER_H_

typedef enum{
	LOW = 0,
	HIGH = 1,	
}pinstate;

#define ENCODER_TICK_PER_ROTATE	180
#define DT_TO_MINUTES			60		//if the dimension of dt is in seconds then DT_TO_MINUTES = 60

#define LOW_LVL_MARGIN			200
#define HIGH_LVL_MARGIN			400

typedef struct{
	
	pinstate state;
	pinstate lastState;
	
} encoderIrSTRUCT;


typedef struct{
	
	encoderIrSTRUCT A;	
	encoderIrSTRUCT B;
	volatile int8_t			counter;
	
}encoderSTRUCT;

//One struct for each encoder
encoderSTRUCT encoder1 = {{LOW,LOW},{LOW,LOW},0};
encoderSTRUCT encoder2 = {{LOW,LOW},{LOW,LOW},0};

int8_t encoder_counter(uint8_t pinLVL_A, uint8_t pinLVL_B);					//give back the direction of the rotation (+1 -1)

void encoder_checkSignal(uint16_t ADCvalue,volatile encoderIrSTRUCT *enc);	//check the state one of the encoder's IR sensors.
void encoder_read(volatile encoderSTRUCT *enc);								//if one of the encoder's IR sensors is change then count
float getRPM (float count, float dt);

int8_t encoder_counter(uint8_t pinLVL_A, uint8_t pinLVL_B)
{
	int8_t counter=0;
	switch (pinLVL_A<<1 | pinLVL_B)
	{
		case 0b01:
		counter--;
		break;
		
		case 0b10:
		counter--;
		break;
		
		case 0b00:
		counter++;
		break;
		
		case 0b11:
		counter++;
		break;
	}
	return counter;
}

void encoder_checkSignal(uint16_t ADCvalue,volatile encoderIrSTRUCT *encIR)
{
	if		(ADCvalue<=LOW_LVL_MARGIN)	encIR->state = LOW;
	else if (ADCvalue>=HIGH_LVL_MARGIN)	encIR->state = HIGH;
}

void encoder_read(volatile encoderSTRUCT *enc)
{	
	//int8_t counter = 0;
	if (enc->A.state != enc->A.lastState) 
	{
		enc->counter += encoder_counter(enc->A.state,enc->B.state);
		enc->A.lastState = enc->A.state;
	}
	if (enc->B.state != enc->B.lastState) 
	{
		enc->counter += encoder_counter(enc->A.state,!enc->B.state & 1);
		enc->B.lastState = enc->B.state;
	}
	//return counter;
}

float getRPM (float  count, float dt)
{
	return count/ENCODER_TICK_PER_ROTATE/dt*DT_TO_MINUTES;
}

#endif /* ENCODER_H_ */
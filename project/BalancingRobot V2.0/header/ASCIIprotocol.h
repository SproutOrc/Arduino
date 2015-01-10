/*
 * commandsForComunication.h
 *
 * Created: 2014.07.03. 21:02:24
 *  Author: F
 */ 


#ifndef ASCIIprotocol_H_
#define ASCIIprotocol_H_

#include <avr/io.h>
#include <stdlib.h>
#include "avr-uart-master/uart.h"

enum commandState{
	STARTED = 0,
	ENDED = 1,
	PROCESSED = 2
};

enum commandSection{
	COMMAND	= 0,
	VALUE1 = 1,
	VALUE2 = 2
};

typedef struct
{
	volatile enum commandState state;
	volatile enum commandSection section;
	volatile char name[10];
	volatile char value1[10];
	volatile char value2[10];
}commandSTRUCT;

commandSTRUCT	lastCommand = {PROCESSED,COMMAND};

void commandHandler(volatile commandSTRUCT *command, uint8_t receivedByte);
void USART_Transmit_float(float USART_data);
void USART_Transmit_int(int int_to_string);

//A correct command syntax looks like this:		/SETaKP 40/		(this set the robotTiltPID.Kp to 40)
void commandHandler(volatile commandSTRUCT *command, uint8_t receivedByte)
{
	static uint8_t ASCIIcount;
	if (receivedByte =='/')					//start and end of the command mark with a "/"
	{
		if (command->state == PROCESSED)
		{
			ASCIIcount=0;
			command->state = STARTED;
			command->section = COMMAND;
		}
		else if(command->state == STARTED)
		{
			command->state = ENDED;
			if (command->section == COMMAND) command->name[0] = '\0';		
			else if (command->section == VALUE1) command->value1[ASCIIcount] = '\0';			
			else if (command->section == VALUE2) command->value2[ASCIIcount] = '\0';
		}		
	}
	else if (receivedByte ==' ')			//after a commands name a " " is required. this indicate the beginning of the value of the command.
	{
		if (command->section == COMMAND)
		{
			command->section = VALUE1;
			command->name[ASCIIcount] = '\0';
			ASCIIcount=0;
		}
		else if (command->section == VALUE1)
		{
			command->section = VALUE2;
			command->value1[ASCIIcount] = '\0';
			ASCIIcount=0;
		}
	}
	else if (command->section == COMMAND)	//fill the command name or value strings
	{
		command->name[ASCIIcount]=receivedByte;
		ASCIIcount++;
	}	
	else if (command->section == VALUE1)
	{
		command->value1[ASCIIcount]=receivedByte;
		ASCIIcount++;
	}
	else if (command->section == VALUE2)
	{
		command->value2[ASCIIcount]=receivedByte;
		ASCIIcount++;
	}
}

void USART_Transmit_float(float USART_data)	//End line 0x0A
{
	uint8_t i=0;
	char CSV_data[10];
	
	dtostrf(USART_data,7,2,CSV_data);
	while (CSV_data[i]!=0)
	{
		uart_putc(CSV_data[i]);
		i++;
	}
}

void USART_Transmit_int(int int_to_string)	//End line 0x0A
{
	uint8_t Ns=10;
	uint8_t i=0;
	char string_to_USART[10];
	
	itoa(int_to_string,string_to_USART,Ns);
	while (string_to_USART[i]!=0)
	{
		uart_putc(string_to_USART[i]);
		i++;
	}
}
#endif /* ASCIIprotocol_H_ */
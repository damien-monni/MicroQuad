//*****************************************
//Damien Monni - www.damien-monni.fr
//
//Read multiple registers from LSM303D.
//Read accelerator values (2 bytes per value) on X, Y and Z.
//Read registers are 0x28 - 0x29 / 0x2A - 0x2B / 0x2C - 0x2D 
//*****************************************

#define F_CPU 8000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "monni_i2c.h"

//Timer related variables
volatile uint32_t t0OvfCount = 0;
uint8_t previousCount = 0;
uint8_t pastCount = 0;

//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 49 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
volatile uint32_t timeFromStartMs = 0;

volatile uint16_t servo[4] = {2300, 2300, 2300, 2300}; //Initial speed in microseconds
volatile int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4
volatile uint16_t startPmwTcnt1 = 0; //TCNT1 value when the PMW cycle starts

//Signed boolean to know where we are in the initialisation process.
//A value of -1 means initialisation completed.
volatile int8_t initStep = 0;

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	uint16_t timerValue = TCNT1;
	
	if(channel < 0){ //Every motors was pulsed, waiting for the next period
		//TCNT1 = 0;
		channel = 1;
		PORTD |= 1<<channel;
		startPmwTcnt1 = timerValue;
		OCR1A = timerValue + servo[0];
		timeFromStartMs += 20;
	}
	else{
		if(channel < 4){ //Last servo pin just goes high
			OCR1A = timerValue + servo[channel];
			PORTD &= ~(1<<channel); //Clear actual motor pin
			PORTD |= 1<<(channel + 1); //Set the next one
			channel++;
		}
		else{
			PORTD &= ~(1<<channel); //Clear the last motor pin
			OCR1A = startPmwTcnt1 + 20000;
			channel = -1; //Wait for the next period
		}
	}
}


//**********************************//
//Main
//**********************************//

int main(void){

	//PMW
	TCCR1B |= 1<<CS11; //Prescaler of 8 because 8MHz clock source
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //LED and motors as output
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	//Play with a LED on PORTD0 a few seconds
	DDRD |= 1<<DDD0; //PORTD0 as output	
	PORTD |= 1<<PORTD0; //Turn on LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s	
	PORTD &= ~(1<<PORTD0); //Turn off LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s
	
	//ATmega328p TWI initialisation 
	//Set SCL to 400kHz (for internal 8Mhz clock)
	TWSR = 0x00;
	TWBR = 0x02;
	
	//Configure sensors
	//Calculate sensors' offsets
	//Initialise Timer 0 for loop timing
	AhrsInit();
	
	sei(); //Enable global interrupts
	
	//*******************************
	//Main loop
	//*******************************
	
	while(1){
	
		if((timeFromStartMs > 2300) && (timeFromStartMs < 7000)){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if(timeFromStartMs > 15000){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if((timeFromStartMs > 7000) && (timeFromStartMs < 15000)){
			AhrsCompute();
		}
		
		
	}
}
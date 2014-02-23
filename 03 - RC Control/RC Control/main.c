/**************************************
Damien MONNI - 20/01/2014 (last update : 07/01/2014)
www.damien-monni.fr

Make brushless motors on pin 43, 44, 45, 46 to run at the initial speed (0 tr/min) on an ATmega2560.
Initial speeds in microsecond should be enter in the servo[] table.
**************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned int servo[4] = {2000, 2000, 2000, 2000}; //Initial speed - 700 to 2000 for ESC Turnigy Plush
volatile int8_t channel = 1; //Controlled motor number : 0, 1, 2 or 3

volatile uint16_t previousTime = 0, time = 0; //Time from 70(1.1ms) to 125(2ms) on 8 bits timer
volatile uint8_t isHigh = 0;

volatile uint16_t timeS = 0;

//For interrupts PCINT
volatile uint8_t portbhistory = 0;

int main(void){

	PCICR |= 1<<PCIE0; //Enable interrupt of PCINT7:0
	PCMSK0 |= 1<<PCINT0 | 1<<PCINT1;

	TCCR1B |= 1<<CS10; //Prescaler of 0
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4;
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	while(1){
		/*if(timeS > 50*8){
			servo[0] = time/(float)2.0f;
		}*/
		if(timeS > 125 && timeS < 130){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if(timeS > 300){
			servo[0] = time/1.4f;
			servo[1] = time/1.4f;
			servo[2] = time/1.4f;
			servo[3] = time/1.4f;
		}
	}

	return 0;
}

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	if(channel < 0){ //Every motors was pulsed, waiting for the next period
		//TODO : try to use TCNT1 >= usToTicks(20000) instead of TCNT1 >= 40000
		if(TCNT1 >= 20000){ //50Hz with a prescaler of 8 at 16MHz
			TCNT1 = 0;
			channel = 1;
			PORTD |= 1<<channel;
			OCR1A = servo[0];
			timeS++;
		}
		else{
			//TODO : try to use OCR1A = usToTicks(20000) instead of OCR1A = 40000
			OCR1A = 20000;
		}
	}
	else{
		if(channel < 4){ //Last servo pin just goes high
			OCR1A = TCNT1 + servo[channel];
			PORTD &= ~(1<<channel); //Clear actual motor pin
			PORTD |= 1<<(channel + 1); //Set the next one
			channel++;
		}
		else{
			PORTD &= ~(1<<channel); //Clear the last motor pin
			OCR1A = TCNT1 + 500; //Call again the interrupt a bit after that
			channel = -1; //Wait for the next period
		}
	}
}

ISR(PCINT0_vect){
	
	uint8_t changedbits;

	changedbits = PINB ^ portbhistory;
	portbhistory = PINB;
	
	if(changedbits & (1 << PB0))
	{
		/* PCINT0 changed */
		//Min just goes high, is now high
		if(PINB & 1<<PORTB0){ //Be careful of assigning the good PORTBx
			previousTime = TCNT1;
			isHigh = 1;
		}
		//Pin just goes low, is now low
		else{
			if(TCNT1 > previousTime){
				time = TCNT1 - previousTime;
			}
			else{
				time = (20000 - previousTime) + TCNT1;
			}
			isHigh = 0;
		}
	}
	
	if(changedbits & (1 << PB1))
	{
		/* PCINT1 changed */
		
	}
}

/**************************************
Damien MONNI - 20/01/2014 (last update : 07/01/2014)
www.damien-monni.fr

Make brushless motors on pin 43, 44, 45, 46 to run at the initial speed (0 tr/min) on an ATmega2560.
Initial speeds in microsecond should be enter in the servo[] table.
**************************************/
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us);

//Fonction inverse de usToTicks(). Convertie microsecondes en nombre de "tops d'horloge".
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t ticksToUs(uint32_t ticks);

float map(float x, float in_min, float in_max, float out_min, float out_max);

//Constantes pour la fonction usToTicks
const float clockSourceMhz = 8.0f;
const uint8_t prescaler = 8;

//Constantes PMW en microsecondes
const uint16_t rcMinUs = 1400;
const uint16_t rcMaxUs = 2000;
const uint16_t motorMinUs = 700;
const uint16_t motorMaxUs = 1400;

//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 46 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
volatile uint32_t timeFromStartMs = 0;

volatile unsigned int servo[4] = {2300, 2300, 2300, 2300}; //Initial speed in microseconds
volatile int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4
volatile uint16_t startPmwTcnt1 = 0; //TCNT1 value when the PMW cycle starts

volatile int16_t previousThrottle = 0; //Time from 70(1.1ms) to 125(2ms) on 8 bits timer
volatile int8_t rcIsLow = -1;

//For interrupts PCINT
volatile uint8_t portbhistory = 0;

//RC commands
volatile int16_t throttleUs = 0; //Altitude control
volatile int16_t roll = 0; //Left or right
volatile int16_t pitch = 0; //Up and down
volatile int16_t yaw = 0; //Left or right in level fly

//ISR functions
void pmw();
void pcint();

volatile uint16_t countDebug = 0;

int main(void){

	PCICR |= 1<<PCIE0; //Enable interrupt of PCINT7:0
	PCMSK0 |= 1<<PCINT3;
	
	TCCR0B |= 1<<CS00 | 1<<CS01; //timer 0 (8bit) prescaler 64
	
	
	TCCR1B |= 1<<CS11; //Prescaler of 8 because 8MHz clock source
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //LED and motors as output
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	while(1){
	
		if((timeFromStartMs > 7000) && (timeFromStartMs < 7500)){
			PORTD |= 1<<PORTD0;
		}
	
		uint16_t computedThrottle;
		
		if((timeFromStartMs > 2300) && (timeFromStartMs < 7000)){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
	
		
		if((timeFromStartMs > 7000) && (timeFromStartMs < 45000)){
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{	
				//computedThrottle = map(throttleUs, rcMinUs, rcMaxUs, motorMinUs, motorMaxUs);
				computedThrottle = ((float)throttleUs - 1400) * (1400 - 700) / (2000 - 1400) + 700;
			}
			
			servo[0] = computedThrottle;
			servo[1] = computedThrottle;
			servo[2] = computedThrottle;
			servo[3] = computedThrottle;
		}
		
		if(timeFromStartMs > 45000){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
	}

	return 0;
}

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	pmw();
}

void pmw(){
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
			OCR1A = startPmwTcnt1 + usToTicks(20000);
			channel = -1; //Wait for the next period
		}
	}
}

ISR(PCINT0_vect){
	pcint();
}

void pcint(){
uint16_t timerValue = TCNT1;
	
	uint8_t changedbits;

	//^ = XOR (exclusive OR, one bit or the other, but not both at the same time). Use to detect a bit that has changed.
	//
	//EXAMPLE:
	//    0001001
	//XOR 0000001
	//----------
	//    0001000
	changedbits = PINB ^ portbhistory;
	portbhistory = PINB;
	
	//PCINT3 changed
	if( (changedbits & (1 << PB3)) )
	{
		//Min just goes high, is now high
		if(PINB & 1<<PORTB3){ //Be careful of assigning the good PORTBx
			previousThrottle = timerValue;
		}
		//Pin just goes low, is now low
		else{
			int16_t tempThrottle;
			if(timerValue > previousThrottle){
				tempThrottle = ticksToUs(timerValue - previousThrottle);
			}
			else{
				tempThrottle = ticksToUs((usToTicks(20000) - previousThrottle) + timerValue);
			}
			if(tempThrottle > 1800){
				PORTD |= 1<< PORTD0;
			}
			else{
				PORTD &= ~(1<<PORTD0);
			}
				
			if((tempThrottle >= (rcMinUs - 400)) && (tempThrottle <= (rcMaxUs + 400))){
				throttleUs = tempThrottle;
			}
			else{
				countDebug++;
			}
		}
	}
}

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us){
	return (clockSourceMhz * us) / (float)prescaler;
}

//Fonction inverse de usToTicks(). Convertie microsecondes en nombre de "tops d'horloge".
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t ticksToUs(uint32_t ticks){
	return ((ticks * (float)prescaler) / clockSourceMhz);
}


float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	
	if(result < out_min){
		result = out_min;
	}
	else if(result > out_max){
		result = out_max;
	}
  
	return result; 
}

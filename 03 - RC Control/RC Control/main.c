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
volatile int16_t previousPitch = 0;
volatile int16_t previousRoll = 0;
volatile int8_t rcIsLow = -1;

//For interrupts PCINT
volatile uint8_t portbhistory = 0;

//RC commands
volatile int16_t throttleUs = 0; //Altitude control
volatile int16_t rollUs = 0; //Left or right
volatile int16_t pitchUs = 0; //Up and down
volatile int16_t yawUs = 0; //Left or right in level fly

volatile uint16_t throttleInitUs = 0;
volatile uint16_t throttleInitCounter = 0;
volatile uint8_t throttleInitCalculated = 0;

volatile uint16_t pitchCenterUs = 0;
volatile uint16_t pitchCenterCounter = 0;
volatile uint8_t pitchCenterCalculated = 0;

volatile uint16_t rollCenterUs = 0;
volatile uint16_t rollCenterCounter = 0;
volatile uint8_t rollCenterCalculated = 0;

//Signed boolean to know where we are in the initialisation process.
//A value of -1 means initialisation completed.
volatile int8_t initStep = 0;

uint8_t pcintNb = 0;

//ISR functions
void pmw();
void pcint();

volatile uint16_t countDebug = 0;

int main(void){
	
	TCCR1B |= 1<<CS11; //Prescaler of 8 because 8MHz clock source
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD0 | 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //LED and motors as output
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	while(1){
	
		unsigned int computedThrottle;
		unsigned int computedPitch;
		unsigned int computedRoll;
		
		if((timeFromStartMs > 2300) && (timeFromStartMs < 7000)){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if((timeFromStartMs > 7000) && (initStep == 0)){
			initStep = 1;
			PCICR |= 1<<PCIE0; //Enable interrupt of PCINT7:0
			PCMSK0 |= 1<<PCINT2;// | 1<<PCINT3 | 1<<PCINT4;
		}
		
		if(initStep == -1){
		
			if((timeFromStartMs > 7000) && (timeFromStartMs < 40000)){
				/*ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{	

				}*/
				
				computedThrottle = (((float)throttleUs - throttleInitUs) * (motorMaxUs - motorMinUs) / (rcMaxUs - throttleInitUs) + motorMinUs);
				computedPitch = ((((float)pitchUs - pitchCenterUs) * (300 - 0) / (rcMaxUs - pitchCenterUs) + 0));
				computedRoll = ((((float)rollUs - rollCenterUs) * (300 - 0) / (rcMaxUs - rollCenterUs) + 0));
				
				servo[0] = computedThrottle;
				servo[1] = computedThrottle + computedPitch;
				servo[2] = computedThrottle;
				servo[3] = computedThrottle + computedRoll;
			}
			
			if(timeFromStartMs > 40000){ 
				servo[0] = 700;
				servo[1] = 700;
				servo[2] = 700;
				servo[3] = 700;
			}
		
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
	
	if(pcintNb == 0){
	//PCINT2 changed
		if( (changedbits & (1 << PB2)) )
		{
			//Min just goes high, is now high
			if(portbhistory & 1<<PORTB2){ //Be careful of assigning the good PORTBx
				previousRoll = timerValue;
			}
			//Pin just goes low, is now low
			else{
				int16_t tempRoll;
				if(timerValue > previousRoll){
					tempRoll = ticksToUs(timerValue - previousRoll);
				}
				else{
					tempRoll = ticksToUs((65536 - previousRoll) + timerValue);
				}
				
				//Valid signal detected
				if((tempRoll >= (rcMinUs - 400)) && (tempRoll <= (rcMaxUs + 400))){
					
					rollUs = tempRoll;
					
					pcintNb = 1;
					PCMSK0 &= ~(1<<PCINT2); //Clear interrupt on PCINT2
					PCMSK0 |= 1<<PCINT3; //Enable interrupt on PCINT3
					
					//Initialisation process
					if(initStep == 1){
						if((rollCenterCounter < 60000) && (rollCenterUs < 40000)){
							rollCenterCounter++;
							rollCenterUs += rollUs;
						}
						else if(rollCenterCalculated == 0){
							rollCenterUs = (float)rollCenterUs / (float)rollCenterCounter;
							rollCenterCalculated = 1;
							if((rollCenterCalculated == 1) && (throttleInitCalculated == 1) && (pitchCenterCalculated == 1)){
								initStep == -1;
							}
						}
						
					}
				}
				else{
					countDebug++;
				}
			}
		}
	}
	else if(pcintNb == 1){
	//PCINT3 changed
		if( (changedbits & (1 << PB3)) )
		{
			//Min just goes high, is now high
			if(portbhistory & 1<<PORTB3){ //Be careful of assigning the good PORTBx
				previousThrottle = timerValue;
			}
			//Pin just goes low, is now low
			else{
				int16_t tempThrottle;
				if(timerValue > previousThrottle){
					tempThrottle = ticksToUs(timerValue - previousThrottle);
				}
				else{
					tempThrottle = ticksToUs((65536 - previousThrottle) + timerValue);
				}
					
				if((tempThrottle >= (rcMinUs - 400)) && (tempThrottle <= (rcMaxUs + 400))){
					throttleUs = tempThrottle;
					pcintNb = 2;
					PCMSK0 &= ~(1<<PCINT3); //Clear interrupt on PCINT3
					PCMSK0 |= 1<<PCINT4; //Enable interrupt on PCINT4
					if(initStep == 1){
						if((throttleInitCounter < 60000) && (throttleInitUs < 40000)){
							throttleInitCounter++;
							throttleInitUs += throttleUs;
						}
						else if(throttleInitCalculated == 0){
							throttleInitUs = (float)throttleInitUs / (float)throttleInitCounter;
							throttleInitCalculated = 1;
							if((rollCenterCalculated == 1) && (throttleInitCalculated == 1) && (pitchCenterCalculated == 1)){
								initStep == -1;
							}
						}
					}
				}
				else{
					countDebug++;
				}
			}
		}
	}
	else if(pcintNb == 2){
		//PCINT4 changed
		if( (changedbits & (1 << PB4)) )
		{
			//Min just goes high, is now high
			if(portbhistory & 1<<PORTB4){ //Be careful of assigning the good PORTBx
				previousPitch = timerValue;
			}
			//Pin just goes low, is now low
			else{
				int16_t tempPitch;
				if(timerValue > previousPitch){
					tempPitch = ticksToUs(timerValue - previousPitch);
				}
				else{
					tempPitch = ticksToUs((65536 - previousPitch) + timerValue);
				}
					
				if((tempPitch >= (rcMinUs - 400)) && (tempPitch <= (rcMaxUs + 400))){
					pitchUs = tempPitch;
					pcintNb = 0;
					PCMSK0 &= ~(1<<PCINT4); //Clear interrupt on PCINT2
					PCMSK0 |= 1<<PCINT2; //Enable interrupt on PCINT3
					if(initStep == 1){
						if((pitchCenterCounter < 60000) && (pitchCenterUs < 40000)){
							pitchCenterCounter++;
							pitchCenterUs += pitchUs;
						}
						else if(pitchCenterCalculated == 0){
							pitchCenterUs = (float)pitchCenterUs / (float)pitchCenterCounter;
							pitchCenterCalculated = 1;
							if((rollCenterCalculated == 1) && (throttleInitCalculated == 1) && (pitchCenterCalculated == 1)){
								initStep == -1;
							}
						}
					}
				}
				else{
					countDebug++;
				}
			
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

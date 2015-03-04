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

volatile uint16_t servo[4] = {2300, 2300, 2300, 2300}; //Initial speed in microseconds
volatile int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4
volatile uint16_t startPmwTcnt1 = 0; //TCNT1 value when the PMW cycle starts

volatile uint16_t previousRcValue = 0;

//For interrupts PCINT
volatile uint8_t portHistory = 0;

//RC commands
volatile int16_t throttleUs = 0; //Altitude control
volatile int16_t rollUs = 0; //Left or right
volatile int16_t pitchUs = 0; //Up and down
volatile int16_t yawUs = 0; //Left or right in level fly

//Initial values of RC commands
typedef volatile struct {
	uint16_t initUs;
	uint16_t initCounter; //Could be an uint8_t?
	uint8_t initCalculated;
} CenterRc;

CenterRc centers[4] = {0};

//Signed boolean to know where we are in the initialisation process.
//A value of -1 means initialisation completed.
volatile int8_t initStep = 0;

volatile uint8_t pcintNb = 1;

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
	
		int16_t computedThrottle;
		int16_t computedPitch;
		int16_t computedRoll;
		int16_t computedYaw;
		
		if((timeFromStartMs > 2300) && (timeFromStartMs < 7000)){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if((timeFromStartMs > 7000) && (initStep == 0)){
			initStep = 1;
			PCICR |= 1<<PCIE0; //Enable interrupt of PCINT7:0
			PCMSK0 |= 1<<PCINT1;
		}
		
		if(initStep == -1){
		
			if((timeFromStartMs > 7000) && (timeFromStartMs < 40000)){
					
				computedThrottle = (throttleUs - 1100) + 700;
				computedRoll = (rollUs - 1600);
				computedPitch = (pitchUs - 1600);
				computedYaw = (yawUs - 1600);
				
				servo[0] = computedThrottle + computedYaw + computedRoll;
				servo[1] = computedThrottle + computedPitch;
				servo[2] = computedThrottle + computedYaw;
				servo[3] = computedThrottle;
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
			OCR1A = startPmwTcnt1 + 20000;
			channel = -1; //Wait for the next period
		}
	}
}

ISR(PCINT0_vect){

	uint16_t timerValue = TCNT1;
	
	uint8_t changedBits;

	//^ = XOR (exclusive OR, one bit or the other, but not both at the same time). Use to detect a bit that has changed.
	//
	//EXAMPLE:
	//    0001001
	//XOR 0000001
	//----------
	//    0001000
	changedBits = PINB ^ portHistory;
	portHistory = PINB;
	
	if( (changedBits & (1<<pcintNb)) ){
		//Min just goes high, is now high
		if(portHistory & 1<<pcintNb){
			previousRcValue = timerValue;
		}
		//Min just goes low, is now low
		else{
			int16_t temp;
			if(timerValue > previousRcValue){
				temp = (timerValue - previousRcValue);
			}
			else{
				temp = (65536 - previousRcValue) + timerValue;
			}
			
			//Valid signal detected
			if((temp >= (rcMinUs - 400)) && (temp <= (rcMaxUs + 400))){
				switch(pcintNb){
					case 1:	yawUs = temp;
							break;
					case 2:	rollUs = temp;
							break;
					case 3:	throttleUs = temp;
							break;
					case 4:	pitchUs = temp;
							break;
					default: break;
				}
				
				//Initialisation process
				if(initStep == 1){
					if((centers[pcintNb - 1].initCounter < 60000) && (centers[pcintNb - 1].initUs < 40000)){
						centers[pcintNb - 1].initCounter++;
						centers[pcintNb - 1].initUs += temp;
					}
					else if(centers[pcintNb - 1].initCalculated == 0){
						centers[pcintNb - 1].initUs /= (float)centers[pcintNb - 1].initCounter;
						centers[pcintNb - 1].initCalculated = 1;
						if((centers[0].initCalculated == 1)
							&& (centers[1].initCalculated == 1)
							&& (centers[2].initCalculated == 1)
							&& (centers[3].initCalculated == 1)){
								initStep = -1;
						}
					}
				}
				
				if(pcintNb == 4){
					pcintNb = 1;
				}
				else{
					pcintNb++;
				}
				
				PCMSK0 &= ~(1<<(pcintNb - 1)); //Clear actual interrupt on PCINT
				PCMSK0 |= 1<<pcintNb; //Enable newt interrupt on PCINT
				
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

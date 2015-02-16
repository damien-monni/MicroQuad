/**************************************
Damien MONNI - 20/02/2014 (last update : 15/02/2015)
www.damien-monni.fr

Fait tourner 4 moteurs brushless avec un ATMega328p sur PD1, PD2, PD3, PD4
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us);

//Constantes pour la fonction usToTicks
const float clockSourceMhz = 8.0f;
const uint8_t prescaler = 8;

//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 46 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
volatile uint32_t timeFromStartMs = 0;

volatile uint32_t servo[4] = {700, 700, 700, 700}; //Initial speed - 700 to 2000 for ESC Turnigy Plush
volatile int8_t channel = 1; //Controlled motor number : 0, 1, 2 or 3

int main(void){

	TCCR1B |= 1<<CS11; //Prescaler 8 avec une horlge à 8Mhz
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //Ports set as OUTPUTS
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	while(1){
	
		if(timeFromStartMs > 5000 && timeFromStartMs < 7000){
			servo[0] = 850;
			servo[1] = 850;
			servo[2] = 850;
			servo[3] = 850;
		}
		
		if(timeFromStartMs > 7000){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
	
		/*if(count > 125 && count < 130){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}
		
		if(count > 300 && count < 320){
			servo[0] = 800;
			servo[1] = 800;
			servo[2] = 800;
			servo[3] = 800;
		}
		
		if(count > 800 && count < 810){
			servo[0] = 700;
			servo[1] = 700;
			servo[2] = 700;
			servo[3] = 700;
		}*/
		
	}

	return 0;
}

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	if(channel < 0){ //Every motors was pulsed, waiting for the next period
		//TODO : try to use TCNT1 >= usToTicks(20000) instead of TCNT1 >= 40000
		if(TCNT1 >= usToTicks(20000)){ //50Hz with a prescaler of 8 at 16MHz
			TCNT1 = 0;
			channel = 1;
			PORTD |= 1<<channel;
			OCR1A = servo[0];
			timeFromStartMs += 20;
		}
		else{
			//TODO : try to use OCR1A = usToTicks(20000) instead of OCR1A = 40000
			OCR1A = usToTicks(20000);
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
			OCR1A = TCNT1 + 500; //Call again the interrupt just after that
			channel = -1; //Wait for the next period
		}
	}
}

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us){
	return (clockSourceMhz * us) / (float)prescaler;
}
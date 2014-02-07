/**************************************
Damien MONNI - 07/02/2014 (last update : 07/02/2014)
www.damien-monni.fr

Calibration d'un ESC TURNIGY Plush 10amp
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//Convert microsecond to tick for a 16MHz clock source with a prescaler of 8
uint32_t usToTicks(uint32_t us);

volatile unsigned int vitesseArretMs = 700; //Temps à l'état haut du signal PMW pour le moteur à l'arret
volatile boolean isHigh;

int main(void){

	TCCR1B |= 1<<CS11; //Prescaler of 8
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = usToTicks(vitesseArretMs); //Set the first interrupt to occur when the first pulse was ended
	
	DDRL |= 1<<DDL0; //Ports 49, 48, 47, 46 on Arduino Mega 2560 set as OUTPUT
	PORTL = 1<<PORTL0; //Set first servo pin high
	isHigh = true;
	
	sei(); //Enable global interrupts
	
	while(1);

	return 0;
}

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	if(isHigh){ //Every motors was pulsed, waiting for the next period
		PORTL &= ~(1<<PORTL0); //Clear the last motor pin
		OCR1A = usToTicks(20000);
		isHigh = false;
	}
	else{
		TCNT1 = 0;
		PORTL |= 1<<0;
		OCR1A = usToTicks(vitesseArretMs);
	}
}

//Convert microsecond to tick for a 16MHz clock with a prescaler of 8
uint32_t usToTicks(uint32_t us){
	return (16.0 * us) / (float)8; // 16 = tick per microsecond (16MHz/1000000) ; 8 = prescaler
}
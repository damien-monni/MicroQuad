/**************************************
Damien MONNI - 07/02/2014 (last update : 07/02/2014)
www.damien-monni.fr

Calibration d'un ESC TURNIGY Plush 10amp avec un ATMega328P
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us);

//Constantes pour la fonction usToTicks
const float clockSourceMhz = 1.0f;
const uint8_t prescaler = 1;

//Temps de l'impulsion PMW pour un signal de puissance maximale
const unsigned uint16_t maxSpeedMs = 700;

//Donne l'état du signal PMW (bas ou haut)
volatile boolean isHigh;


int main(void){

	TCCR1B |= 1<<CS10; //CS10 : Aucun prescaler
	TIMSK1 |= (1<<OCIE1A); //Interruption déclenchée lorsque OCR1A est atteint
	OCR1A = usToTicks(maxSpeedMs); //Configure la première interruption à la fin de la première impulsion.
	
	DDRB |= 1<<DDB0; //Broche PB0 configurer comme sortie
	PORTB = 1<<PORTB0; //Broche PB0 à l'état haut
	isHigh = true; //Signal PMW à l'état haut
	
	sei(); //Activer les interruptions
	
	while(1); //Boucle infinie

	return 0;
}

//Interruption sur OCR1A
ISR(TIMER1_COMPA_vect)
{
	//Signal PMW à l'état haut.
	if(isHigh){
		PORTL &= ~(1<<PORTL0); //Passer le signal à l'état bas, terminer l'impulsion
		OCR1A = usToTicks(20000); //Configurer la prochaine interruption afin de créer un signal à 50Hz
		isHigh = false;
	}
	//Signal PMW à l'état bas, fin de la période de 20ms (50Hz)
	else{
		TCNT1 = 0; //Remettre timer à 0
		PORTL |= 1<<0; //Commencer une impulsion
		OCR1A = usToTicks(maxSpeedMs); //Configurer la prochaine interruption pour la fin de l'impulsion
	}
}

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us){
	return (clockSourceMhz * us) / (float)prescaler;
}
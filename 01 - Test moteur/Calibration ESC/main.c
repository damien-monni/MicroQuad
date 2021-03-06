/**************************************
Damien MONNI - 07/02/2014 (dernière mise à jour : 10/02/2015)
www.damien-monni.fr

Calibration d'un ESC TURNIGY Plush 10amp sur le port PD4 d'un ATMega328P 8Mhz
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

//Temps de l'impulsion PMW en microsecond (min : 700us - max : 2400us)
volatile uint32_t speedUs = 2300;

//Donne l'état du signal PMW (bas ou haut)
volatile uint8_t isHigh = 0;

//Donne le temps d'execution du programme en ms (compte au max environ 1.5 mois soit environ 46 jours)
//MIS A JOUR SEULEMENT TOUTES LES 20MS VIA LA GENERATION PMW.
volatile uint32_t timeFromStartMs = 0;

int main(void){

	TCCR1B |= 1<<CS11; //Prescaler 8 avec une horlge à 8Mhz
	TIMSK1 |= (1<<OCIE1A); //Interruption déclenchée lorsque OCR1A est atteint
	OCR1A = usToTicks(speedUs); //Configure la première interruption à la fin de la première impulsion.
	
	DDRD |= 1<<DDD1; //Broche PB0 configurer comme sortie
	PORTD = 1<<PORTD1; //Broche PB0 à l'état haut
	isHigh = 1; //Signal PMW à l'état haut
	
	sei(); //Activer les interruptions
	
	while(1){ //Boucle infinie

		//Après 5s d'initialisation
		if(timeFromStartMs > 2300){
			speedUs = 700;
		}
	}

	return 0;
}

//Interruption sur OCR1A
ISR(TIMER1_COMPA_vect)
{
	//Signal PMW à l'état haut.
	if(isHigh == 1){
		PORTD &= ~(1<<PORTD1); //Passer le signal à l'état bas, terminer l'impulsion
		OCR1A = usToTicks(20000); //Configurer la prochaine interruption afin de créer un signal à 50Hz (20ms)
		isHigh = 0;
		timeFromStartMs += 20;
	}
	//Signal PMW à l'état bas, fin de la période de 20ms (50Hz)
	else{
		TCNT1 = 0; //Remettre timer à 0
		PORTD |= 1<<PORTD1; //Commencer une impulsion
		OCR1A = speedUs; //Configurer la prochaine interruption pour la fin de l'impulsion
		isHigh = 1;
	}
}

//Renvoie le nombre de tops d'horloge d'une durée donnée en microseconde
//Il est important de bien renseigner les deux constantes
//globales clockSourceMhz et prescaler.
uint32_t usToTicks(uint32_t us){
	return (clockSourceMhz * us) / (float)prescaler;
}

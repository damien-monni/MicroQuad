/**************************************
Damien MONNI - 28/02/2014 (last update : 28/02/2014)
www.damien-monni.fr

Fait tourner un moteur en fonction de l'inclinaison du drone. 
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//*********************************************//
//********* Prototypes de fonctions ***********//
//*********************************************//

//Initialise l'ADC et démarre une première conversion
void initAdc();

//Initialise le PMW permettant le contrôle des ESC pour moteurs brushless
void initPmw();

//Converti x d'une plage à une autre
long map(long x, long in_min, long in_max, long out_min, long out_max);


//*********************************************//
//********* Variables globales ***********//
//*********************************************//

//PMW variables
volatile unsigned int servo[4] = {700, 700, 700, 700};
volatile int8_t channel = 1; //Controlled motor number : 1, 2, 3 or 4

//ADC variables
volatile uint16_t adc = 0;

//Compte le nombre de seconde/50 écoulées depuis le lancement du programme
//S'incrémente à 50Hz via la construction du signal PMW
volatile uint16_t timeS = 0;


//*********************************************//
//********* Programme principal ***********//
//*********************************************//
int main(void){
	
	initPmw();
	initAdc();
	
	sei(); //Activer les interruptions
	
	while(1){
	
		//Après initialisation des ESC (après 7s)
		if(timeS > 7*50){
			
			if(adc < 512){
				adc = 512;
			}
			else if(adc > 1023){
				adc = 1023;
			}
			
			servo[0] = map(adc, 512, 1023, 700, 1000);
		}
		
	}
	
	return 0;

}

//*********************************************//
//********* Interruptions ***********//
//*********************************************//

//Construction du signal PMW
ISR(TIMER1_COMPA_vect)
{
	if(channel < 0){ //Tous les ESC ont reçus leurs impulsions, on attends la prochaine période
		if(TCNT1 >= 20000){ //Fin de la période à 5Hz
			TCNT1 = 0;
			channel = 1;
			PORTD |= 1<<channel;
			OCR1A = servo[0];
			timeS++;
		}
		else{
			OCR1A = 20000; //Attendre la fin de la période
		}
	}
	else{
		if(channel < 4){ //Création de l'impulsion sur l'ESC suivant
			OCR1A = TCNT1 + servo[channel];
			PORTD &= ~(1<<channel); //Fin de l'impulsion sur l'ESC en cours
			PORTD |= 1<<(channel + 1); //Début de l'impulsion sur l'ESC suivant
			channel++;
		}
		else{
			PORTD &= ~(1<<channel); //Fin de l'impulsion sur l'ESC en cours
			OCR1A = TCNT1 + 500; //Rappeler l'interruption quelques microsecondes après
			channel = -1; //Attendre la prochaine période
		}
	}
}

//Interruption lorsqu'une conversion de l'ADC se termine
//Conversion en boucle
ISR(ADC_vect){

	//Lire le resultat
	adc = ADC;
	
	//Sélectionner le convertisseur suivant
	/*if(ADMUX == 5){
		ADMUX = 0;
	}
	else{
		ADMUX++;
	}*/
	
	//Démarrer une nouvelle conversion
	ADCSRA |= 1<<ADSC;
	
}

//*********************************************//
//********* Fonctions ***********//
//*********************************************//

//Initialise le PMW permettant le contrôle des ESC pour moteurs brushless
void initPmw(){

	TCCR1B |= 1<<CS11; //Prescaler 8 avec une horlge à 8Mhz
	TIMSK1 |= (1<<OCIE1A); //Interruption sur OCR1A
	OCR1A = servo[0]; //Prévoir la première interuption sur servo[0]
	
	DDRD |= 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //Sortie vers les ESC
	PORTD = 1<<channel; //Première impulsion PMW
	
}

//Initialise l'ADC et démarre une première conversion
void initAdc(){

	//Activer l'ADC via ADEN dans ADCSRA
	ADCSRA |= 1<ADEN;

	//Selectionner la référence de voltage via REFSn dans ADMUX
		//Rien à faire car utilisation de AREF, donc REFS0 et RES1 = 0
	
	//Sélectionner le convertisseur ADC5 via MUX dans ADMUX
	ADMUX |= 1<<MUX0 | 1<<MUX2;
	
	//Activer l'interruption sur conversion terminée via ADIE dans ADCSRA
	ADCSRA |= 1<<ADIE;
	
	//Paramétrer le prescaler de l'ADC à 64 :horloge 8Mhz -> horloge ADC de 125kHz
	ADCSRA |= 1<<ADPS1 | 1<<ADPS2;
	
	//Démarrer une première conversion
	ADCSRA |= 1<<ADSC;
	
}

//Converti x d'une plage à une autre
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
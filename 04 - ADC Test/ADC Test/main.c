/**************************************
Damien MONNI - 28/02/2014 (last update : 28/02/2014)
www.damien-monni.fr

Fait tourner un moteur en fonction de l'inclinaison du drone. 
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//Initialise l'ADC et démarre une première conversion
void initAdc();

volatile uint16_t adc = 0;

int main(void){
	
	initAdc();
	
	return 0;

}

//Interruption lorsqu'une conversion de l'ADC se termine
//Conversion en boucle
ISR(ADC_vect){

	//Lire le resultat
	uint8_t adcL = ADCL;
	uint8_t adcH = ADCH;
	adc = (adcH<<8) | adcL;
	
	//Sélectionner le convertisseur suivant
	if(ADMUX == 5){
		ADMUX = 0;
	}
	else{
		ADMUX++;
	}
	
	//Démarrer une nouvelle conversion
	ADCSRA =| 1<<ADSC;
	
}

//Initialise l'ADC et démarre une première conversion
void initAdc(){

	//Activer l'ADC via ADEN dans ADCSRA
	ADCSRA |= 1<ADEN;

	//Selectionner la référence de voltage via REFSn dans ADMUX
		//Rien à faire car utilisation de AREF, donc REFS0 et RES1 = 0
	
	//Sélectionner le premier convertisseur via MUX dans ADMUX
		//Rien à faire car l'on souhaite commencer par ADC0, donc MUX3...0 = 0
	
	//Activer l'interruption sur conversion terminée via ADPS dans ADCSRA
	ADCSRA |= 1<<ADIE;
	
	//Paramétrer le prescaler de l'ADC à 64 :horloge 8Mhz -> horloge ADC de 125kHz
	ADCSRA |= 1<<ADPS1 | 1<<ADPS2;
	
	//Démarrer une première conversion
	ADCSRA =| 1<<ADSC;
	
}
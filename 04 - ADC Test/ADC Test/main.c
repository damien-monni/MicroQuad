#include <avr/io.h>
#include <avr/interrupt.h>

int main(void)
{
	ADCSRA |= 1<<ADPS2 | 1<<ADPS1;
	ADMUX |= 1<<REFS0;
	ADMUX |= 1<<MUX0 | 1<<MUX2;
	ADCSRA |= 1<<ADIE;
	ADCSRA |= 1<<ADEN;

	sei();

	ADCSRA |= 1<<ADSC;

	while (1)
	{
	}
}

ISR(ADC_vect)
{
	uint8_t theLowADC = ADCL;
	uint16_t theTenBitResults = ADCH<<8 | theLowADC;
	
	if(theTenBitResults == 0){
		DDRB |= 1<<DDB0;
		PORTB |= 1<<PORTB0;
	}
	else{
		DDRB |= 1<<DDB0;
		PORTB &= ~(1<<PORTB0);		
	}

	ADCSRA |= 1<<ADSC; 
}
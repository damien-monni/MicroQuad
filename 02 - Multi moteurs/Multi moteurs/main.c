/**************************************
Damien MONNI - 20/02/2014 (last update : 20/02/2014)
www.damien-monni.fr

Fait tourner 4 moteurs brushless avec un ATMega328p sur PD1, PD2, PD3, PD4
**************************************/

#include <avr/io.h> 
#include <avr/interrupt.h>

//Convert microsecond to tick for a 16MHz clock with a prescaler of 8
uint16_t usToTicks(uint16_t us);

volatile unsigned int servo[4] = {2000, 2000, 2000, 2000}; //Initial speed - 700 to 2000 for ESC Turnigy Plush
volatile int8_t channel = 1; //Controlled motor number : 0, 1, 2 or 3

volatile int32_t count = 0;

int main(void){

	TCCR1B |= 1<<CS10; //Prescaler of 0
	TIMSK1 |= (1<<OCIE1A); //Interrupt on OCR1A
	OCR1A = servo[0]; //Set the first interrupt to occur when the first pulse was ended
	
	DDRD |= 1<<DDD1 | 1<<DDD2 | 1<<DDD3 | 1<<DDD4; //Ports 49, 48, 47, 46 on Arduino Mega 2560 set as OUTPUT
	PORTD = 1<<channel; //Set first servo pin high
	
	sei(); //Enable global interrupts
	
	while(1){
		if(count > 125 && count < 130){
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
		}
		
	}

	return 0;
}

//PMW Building ISR
ISR(TIMER1_COMPA_vect)
{
	if(channel < 0){ //Every motors was pulsed, waiting for the next period
		//TODO : try to use TCNT1 >= usToTicks(20000) instead of TCNT1 >= 40000
		if(TCNT1 >= 20000){ //50Hz with a prescaler of 8 at 16MHz
			TCNT1 = 0;
			channel = 1;
			PORTD |= 1<<channel;
			OCR1A = servo[0];
			count++;
		}
		else{
			//TODO : try to use OCR1A = usToTicks(20000) instead of OCR1A = 40000
			OCR1A = 20000;
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

//Convert microsecond to tick for a 16MHz clock with a prescaler of 8
uint16_t usToTicks(uint16_t us){
	return (1 * us) / (float)1; // 16 = tick per microsecond (16MHz/1000000) ; 8 = prescaler
}
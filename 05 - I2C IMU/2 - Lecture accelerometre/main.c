//*****************************************
//Damien Monni - www.damien-monni.fr
//
//Read multiple registers from LSM303D.
//Read accelerator values (2 bytes per value) on X, Y and Z.
//Read registers are 0x28 - 0x29 / 0x2A - 0x2B / 0x2C - 0x2D 
//*****************************************

#include <avr/io.h>
#include <util/delay.h>

#include "monni_i2c.h"


//**********************************//
//Structures definitions
//**********************************//

//Three axis
typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}Axis;

Axis accelerometer = {0};

//**********************************//
//Main
//**********************************//

int main(void){
	
	//Play with a LED on PORTD0 a few seconds
	DDRD |= 1<<DDD0; //PORTD0 as output	
	PORTD |= 1<<PORTD0; //Turn on LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s	
	PORTD &= ~(1<<PORTD0); //Turn off LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s
	
	
	//Set SCL to 400kHz (for internal 8Mhz clock)
	TWSR = 0x00;
	TWBR = 0x02;
	
	
	//Enable X, Y, Z accelerometer's axis and
	//set data rate selection to 400Hz.	
	//Full scale 8g
	while(twiWriteOneByte(0b0011101, 0x20, 0b10000111) == 0);
	while(twiWriteOneByte(0b0011101, 0x21, 0x18) == 0); //CTRL2 = 0x18 => full scale +/-8g
	
	
	//Get results
	uint8_t result[6];
	while(1){
	
		if(twiReadMultipleBytes(0b0011101, 0x28, result, 6) == 1){
			accelerometer.x = ((result[1] << 8) | (result[0] & 0xff));
			accelerometer.y = ((result[3] << 8) | (result[2] & 0xff));
			accelerometer.z = ((result[5] << 8) | (result[4] & 0xff));
		}
		
		if(accelerometer.y > 0){
			PORTD = 1;
		}
		else{
			PORTD = 0;
		}
	}
}
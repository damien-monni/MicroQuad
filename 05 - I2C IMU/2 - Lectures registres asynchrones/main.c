//*****************************************
//Damien Monni - www.damien-monni.fr
//
//Read multiple registers from LSM303D.
//Read accelerator values (2 bytes per value) on X, Y and Z.
//Read registers are 0x28 - 0x29 / 0x2A - 0x2B / 0x2C - 0x2D 
//Read it without blocking code.
//*****************************************

#include <avr/io.h>
#include <util/delay.h>


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
//TWI Functions
//**********************************//

void twiWaitFlag(){
	while(!(TWCR & (1<<TWINT)));
}

//Get status code
uint8_t twiGetStatus(){
	return (TWSR & 0xF8);
}

//Write a byte
void twiWriteByte(uint8_t byte){
	TWDR = byte;
	//Clear (by writing it to one) TWINT bit to continue
	TWCR = 1<<TWINT | 1<<TWEN;
}

//Initialize a TWI communication sending a Start and SLA+R/W
uint8_t twiInit(uint8_t slaveAddress, uint8_t isRead){
	//Send a (RE)START
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	if((twiGetStatus() == 0x08) || (twiGetStatus() == 0x10)){
		TWDR = ((slaveAddress << 1) | isRead);
		TWCR = 1<<TWINT | 1<<TWEN;
	}
	while(!(TWCR & (1<<TWINT)));
	
	return (TWSR & 0xF8);
}

//Write only one byte 
uint8_t twiWriteOneByte(uint8_t slaveAddress, uint8_t slaveRegister, uint8_t data){
	
	if(twiInit(slaveAddress, 0) == 0x18){
		twiWriteByte(slaveRegister);
		twiWaitFlag();
		if(twiGetStatus() == 0x28){
			twiWriteByte(data);
			twiWaitFlag();
			if(twiGetStatus() == 0x28){
				return 1;
			}
		}
	}

	return 0;
}

//Read only one byte
uint8_t twiReadOneByte(uint8_t slaveAddress, uint8_t slaveRegister){

	uint8_t value = 0;
	
	if(twiInit(slaveAddress, 0) == 0x18){
		twiWriteByte(slaveRegister);
		twiWaitFlag();
		if(twiGetStatus() == 0x28){
			if(twiInit(slaveAddress, 1) == 0x40){
				TWCR = 1<<TWINT | 1<<TWEN;
				while(!(TWCR & (1<<TWINT)));
				if((TWSR & 0xF8) == 0x58){
					value = TWDR;
					TWCR = 1<<TWINT | 1<<TWEN;
				}
			}
		}
	}
	
	return value;
	
}

//Read multiple bytes
//Return 1 if OK, 0 if error.
uint8_t twiReadMultipleBytes(uint8_t slaveAddress, uint8_t slaveRegister, uint8_t result[], uint8_t nbBytes){
	
	uint8_t i;
	
	if(twiInit(slaveAddress, 0) == 0x18){
		twiWriteByte((slaveRegister | (1<<7)));
		twiWaitFlag();
		if(twiGetStatus() == 0x28){
			if(twiInit(slaveAddress, 1) == 0x40){
				for(i = 0 ; i < nbBytes ; i++){
					TWCR = 1<<TWINT | 1<<TWEN | 1<<TWEA;
					while(!(TWCR & (1<<TWINT)));
					if((TWSR & 0xF8) == 0x50){
						result[i] = TWDR;
					}
					else{
						break;
					}
				}
				if(i == nbBytes){
					return 1;
				}
			}
		}
	}
	
	return 0;
	
}

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
	
	
	//**********************************//
	//Enable X, Y, Z accelerometer's axis and
	//set data rate selection to 400Hz.
	//**********************************//
	
	while(twiWriteOneByte(0b0011101, 0x20, 0b10000111) == 0);
	
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
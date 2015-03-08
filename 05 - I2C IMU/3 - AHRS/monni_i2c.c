#include "monni_i2c.h"

//Wait for the interrupt flag to be set
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
//Return the status code
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
//Return 1 if OK, 0 if error
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
//Return the read value
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
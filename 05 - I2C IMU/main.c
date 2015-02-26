//*****************************************
//Damien Monni - www.damien-monni.fr
//Read the WHO_I_AM register of LSM303D(0Fh) that should be 01001001
//*****************************************

#include <avr/io.h>

int main(void){
	//Send a START condition.
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	
	//Wait for the START condition to be send.
	while(!(TWCR & (1<<TWINT)));
	
	//Check if no error in status code (mask prescaler's 2 LSB bits). Should be 0x08.
	if(TWSR & 0xF8 != 0x08){
		//Write slave address 00111010b. LSB should be 0 for a write operation.
		TWDR = 00111010b;
		//Clear (by writing it to one) TWINT bit to continue
		TWCR |= 1<<TWINT | 1<<TWEN;
		//Wait for the slave's ACK or NoACK to be received.
		while(!(TWCR & (1<<TWINT)));
		//Check if ACK
		if(TWSR & 0xF8 != 0x18){
			//Write data. It should be the WHO_I_AM register's address (0Fh)
			TWDR = 0x0F;
		}
		
	}
	//If error
	else{
	
	}
}
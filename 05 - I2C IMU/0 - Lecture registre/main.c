//*****************************************
//Damien Monni - www.damien-monni.fr
//
//Read the WHO_I_AM register of LSM303D(0Fh) that should be 01001001.
//If the WHO_I_AM register is what we expected, turn on a LED on PORTD0.
//*****************************************

#include <avr/io.h>
#include <util/delay.h>

//Variable to read and store the WHO_I_AM register
uint8_t id = 0;

int main(void){

	DDRD |= 1<<DDD0; //PORTD0 as output	
	PORTD |= 1<<PORTD0; //Turn on LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s	
	PORTD &= ~(1<<PORTD0); //Turn off LED on PORTD0	
	_delay_ms(1500); //Wait 1.5s

	//Set SCL to 400kHz (for internal 8Mhz clock)
    TWSR = 0x00;
    TWBR = 0x02;

	//Send a START condition.
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//Wait for the START condition to be send.
	while(!(TWCR & (1<<TWINT)));	
	//Check if no error in status code (mask prescaler's 2 LSB bits). Should be 0x08.
	if(TWSR & 0xF8 == 0x08){
		//Write slave address 00111010b. LSB should be 0 for a write operation.
		TWDR = 00111010b;
		//Clear (by writing it to one) TWINT bit to continue
		TWCR = 1<<TWINT | 1<<TWEN;
		//Wait for the slave's ACK or NoACK to be received.
		while(!(TWCR & (1<<TWINT)));
		//Check if SLA+W has been transmitted and ACK received.
		if(TWSR & 0xF8 == 0x18){
			//Write data. It should be the WHO_I_AM register's address (0Fh)
			TWDR = 0x0F;
			//Clear (by writing it to one) TWINT bit to continue
			TWCR = 1<<TWINT | 1<<TWEN;
			//Wait for the slave's ACK or NoACK to be received.
			while(!(TWCR & (1<<TWINT)));
			//Check if a data byte has been transmitted and ACK received.
			if(TWSR & 0xF8 == 0x28){
				//Send a REPEATED START condition.
				TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
				//Wait for the REPEATED START condition to be send.
				while(!(TWCR & (1<<TWINT)));
				//Check if no error.
				if(TWSR & 0xF8 == 0x10){
					//Write slave address 00111011b. LSB should be 1 for a read operation.
					TWDR = 00111011b;
					//Clear (by writing it to one) TWINT bit to continue
					TWCR = 1<<TWINT | 1<<TWEN;
					//Wait for the slave's ACK or NoACK to be received.
					while(!(TWCR & (1<<TWINT)));
					//Check if SLA+R has been transmitted and ACK received.
					if(TWSR & 0xF8 == 0x40){
						//Clear (by writing it to one) TWINT bit to continue
						TWCR = 1<<TWINT | 1<<TWEN;
						//Wait for the slave's ACK or NoACK to be received.
						while(!(TWCR & (1<<TWINT)));
						//Check if a data byte has been received and NACK returned.
						if(TWSR & 0xF8 == 0x58){
							//Read and store data received. Should be 01001001b.
							id = TWDR;
							//Send a STOP condition.
							TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
						}
					}
				}
			}
			
		}
		
	}
	
	//If the WHO_I_AM register is what we expected, turn on a LED on PORTD0.
	if(id == 01001001b){
		PORTD |= 1<<PORTD0;
	}
	else{
		PORTD &= ~(1<<PORTD0);
	}

}
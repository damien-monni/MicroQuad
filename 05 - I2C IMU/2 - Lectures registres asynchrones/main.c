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

//One axis values on 2 bytes
typedef struct{
	uint8_t l;
	uint8_t h;
	uint16_t value;
}TwoBytesAxis;

//Three axis
typedef struct{
	TwoBytesAxis x;
	TwoBytesAxis y;
	TwoBytesAxis z;
}Axis;

Axis accelerometer = {0};


//**********************************//
//Functions
//**********************************//

//Send a start or restart condition
void twiSendStart(){
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
}

//Check if the interrupt flag to occur
uint8_t twiIsFlagged(){
	if(TWCR & (1<<TWINT)){
		return 1;
	}
	else{
		return 0;
	}
}

//Get status code
uint8_t twiGetStatus(){
	return (TWSR & 0xF8);
}

//Send the slave's address - isRead = 0 if Slave+W ; isRead = 1 if Slave+R
void twiSendSlaveAdd(uint8_t add, uint8_t isRead){
	TWDR = ((add << 1) & isRead);
	//Clear (by writing it to one) TWINT bit to continue
	TWCR = 1<<TWINT | 1<<TWEN;
}

//Write a byte
void twiWriteByte(uint8_t byte){
	TWDR = byte;
	//Clear (by writing it to one) TWINT bit to continue
	TWCR = 1<<TWINT | 1<<TWEN;
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
	
	uint8_t initOk = 0;
	
	while(initOk == 0){
	
		//Send a START condition.
		sendStart();
		//Wait for the START condition to be send.
		while(twiIsFlagged() == 0);
		//Check if no error in status code (mask prescaler's 2 LSB bits). Should be 0x08.
		if(twiGetStatus() == 0x08){
			//Write slave address 00111010b. LSB should be 0 for a write operation.
			twiSendSlaveAdd(0b0011101, 0);
			//Wait for the slave's ACK or NoACK to be received.
			while(twiIsFlagged() == 0);
			//Check if SLA+W has been transmitted and ACK received.
			if(twiGetStatus() == 0x18){
				//Write data. It should be the CTRL1 register's address (20h)
				twiWriteByte(0x20);
				//Wait for the slave's ACK or NoACK to be received.
				while(twiIsFlagged() == 0);
				//Check if a data byte has been transmitted and ACK received.
				if(twiGetStatus() == 0x28){
					//Write data. 400Hz data rate and X, Y, Z axis enabled
					twiWriteByte(0b10000111);
					//Wait for the slave's ACK or NoACK to be received.
					while(twiIsFlagged() == 0);
					//Check if a data byte has been transmitted and ACK received. Else, start again.
					if(twiGetStatus() == 0x28){
						initOk = 1;
					}
				}
			}
		}
		
	}
	
	//**********************************//
	//Read values
	//**********************************//
	
	//Send a START condition.
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//Wait for the START condition to be send.
	while(!(TWCR & (1<<TWINT)));
	//Check if no error in status code (mask prescaler's 2 LSB bits). Should be 0x08.
	if((TWSR & 0xF8) == 0x10){
		//Write slave address 00111010b. LSB should be 0 for a write operation.
		TWDR = 0b00111010;
		//Clear (by writing it to one) TWINT bit to continue
		TWCR = 1<<TWINT | 1<<TWEN;
		//Wait for the slave's ACK or NoACK to be received.
		while(!(TWCR & (1<<TWINT)));
		//Check if SLA+W has been transmitted and ACK received.
		if((TWSR & 0xF8) == 0x18){
			//Write data. It should be the OUT_X_L_A register's address (28h). 1 in MSB for repeated read.
			TWDR = (0x28 | (1 << 7)); //Or can be 0xA8
			//Clear (by writing it to one) TWINT bit to continue
			TWCR = 1<<TWINT | 1<<TWEN;
			//Wait for the slave's ACK or NoACK to be received.
			while(!(TWCR & (1<<TWINT)));
			//Check if a data byte has been transmitted and ACK received.
			if((TWSR & 0xF8) == 0x28){
				//Send a REPEATED START condition.
				TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
				//Wait for the REPEATED START condition to be send.
				while(!(TWCR & (1<<TWINT)));
				//Check if no error.
				if((TWSR & 0xF8) == 0x10){
					//Write slave address 00111011b. LSB should be 1 for a read operation.
					TWDR = 0b00111011;
					//Clear (by writing it to one) TWINT bit to continue
					TWCR = 1<<TWINT | 1<<TWEN;
					//Wait for the slave's ACK or NoACK to be received.
					while(!(TWCR & (1<<TWINT)));
					//Check if SLA+R has been transmitted and ACK received.
					if((TWSR & 0xF8) == 0x40){
						//Clear (by writing it to one) TWINT bit to continue and set TWEA for a Master ACK
						TWCR = 1<<TWINT | 1<<TWEA | 1<<TWEN;
						//Wait for the slave's ACK or NoACK to be received.
						while(!(TWCR & (1<<TWINT)));
						//Check if a data byte has been received and ACK returned.
						if((TWSR & 0xF8) == 0x50){
							//Read and store data received.
							xL = TWDR;
							//Clear (by writing it to one) TWINT bit to continue
							TWCR = 1<<TWINT | 1<<TWEN;
							//Wait for the slave's ACK or NoACK to be received.
							while(!(TWCR & (1<<TWINT)));
							if((TWSR & 0xF8) == 0x58){
								//Read and store data received. Should be 01001001b.
								xH = TWDR;
								//Send a STOP condition.
								TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
							}
						}
					}
				}
			}
		}
	}
	
	x = ((xH << 8) | (xL & 0xff)); //0xff should be mandatory because of the 2s complement
	
	//Turn on LED on PORTD0 if...
	if(x != 0){
		PORTD |= 1<<PORTD0;
	}
	else{
		PORTD &= ~(1<<PORTD0);
	}

}
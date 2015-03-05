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

void twiWaitFlag(){
	while(!(TWCR & (1<<TWINT)));
}

//Get status code
uint8_t twiGetStatus(){
	return (TWSR & 0xF8);
}

//Send the slave's address - isRead = 0 if Slave+W ; isRead = 1 if Slave+R
void twiSendSlaveAdd(uint8_t add, uint8_t isRead){
	TWDR = ((add << 1) | isRead);
	//Clear (by writing it to one) TWINT bit to continue
	TWCR = 1<<TWINT | 1<<TWEN;
}

//Write a byte
void twiWriteByte(uint8_t byte){
	TWDR = byte;
	//Clear (by writing it to one) TWINT bit to continue
	TWCR = 1<<TWINT | 1<<TWEN;
}

//Read a byte without acknowledge
uint8_t twiReadByteNoAck(){
	uint8_t value = TWDR;
	TWCR = 1<<TWINT | 1<<TWEN;
	return value;
}

//Read a byte and acknowledge
uint8_t twiReadByteAck(){
	uint8_t value = TWDR;
	TWCR = 1<<TWINT | 1<<TWEA | 1<<TWEN;
	return value;
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
				//Clear (by writing it to one) TWINT bit to continue
				TWCR = 1<<TWINT | 1<<TWEN;
				//Wait for the slave's ACK or NoACK to be received.
				while(!(TWCR & (1<<TWINT)));
				if((TWSR & 0xF8) == 0x58){
					value = twiReadByteNoAck();
				}
			}
		}
	}
	
	return value;
	
}

//Read multiple bytes
uint8_t twiReadMultipleBytes(uint8_t slaveAddress, uint8_t slaveRegister){

	uint8_t value = 0;
	uint8_t i = 0;
	
	if(twiInit(slaveAddress, 0) == 0x18){
		twiWriteByte((slaveRegister | (1<<7)));
		twiWaitFlag();
		if(twiGetStatus() == 0x28){
			if(twiInit(slaveAddress, 1) == 0x40){
				for(i = 0 ; i < 3 ; i++){
					TWCR = 1<<TWINT | 1<<TWEN | 1<<TWEA;
					while(!(TWCR & (1<<TWINT)));
					if((TWSR & 0xF8) == 0x50){
						value = twiReadByteNoAck(); //WORKS. BUT SHOULDNOT ? SHOULD BE ACK INSTEAD OF NOACK... ?
					}
					else{
						break;
					}
				}
				if(i == 2){
					
				}
			}
		}
	}
	
	return value;
	
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
	
	//while(twiWriteOneByte(0b0011101, 0x20, 0b10000111) == 0);
	
	//Read ID
	/*if(twiReadOneByte(0b0011101, 0x0F) == 0b01001001){
		PORTD |= 1<<PORTD0;
	}*/
	
	twiReadMultipleBytes(0b0011101, 0x28);
	
	
	/*
		//Send a START condition.
		twiSendStart();
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
						//
					}
				}
			}
		}
		
	*/


	//**********************************//
	//Read values
	//**********************************//
	
	/*
	
	//Send a RESTART condition.
	twiSendStart();
	//Wait for the RESTART condition to be send.
	while(twiIsFlagged() == 0);
	//Check that RESTART has been sent
	if(twiGetStatus() == 0x10){
		//Write slave address 00111010b. LSB should be 0 for a write operation.
		twiSendSlaveAdd(0b0011101, 0);
		//Wait for the slave's ACK or NoACK to be received.
		while(twiIsFlagged() == 0);
		//Check if SLA+W has been transmitted and ACK received.
		if(twiGetStatus() == 0x18){
			//Write data. It should be the OUT_X_L_A register's address (28h). 1 in MSB for repeated read.
			twiWriteByte(0x28 | (1 << 7));
			//Wait for the slave's ACK or NoACK to be received.
			while(twiIsFlagged() == 0);
			//Check if a data byte has been transmitted and ACK received.
			if(twiGetStatus() == 0x28){
				//Send a REPEATED START condition.
				twiSendStart();
				//Wait for the REPEATED START condition to be send.
				while(twiIsFlagged() == 0);
				//Check that RESTART has been sent
				if(twiGetStatus() == 0x10){
					//Write slave address 00111011b. LSB should be 1 for a read operation.
					twiSendSlaveAdd(0b0011101, 1);
					//Wait for the slave's ACK or NoACK to be received.
					while(twiIsFlagged() == 0);
					//Check if SLA+R has been transmitted and ACK received.
					if(twiGetStatus() == 0x40){
						//Clear (by writing it to one) TWINT bit to continue and set TWEA for a Master ACK
						TWCR = 1<<TWINT | 1<<TWEA | 1<<TWEN;
						//Wait for the slave's ACK or NoACK to be received.
						while(twiIsFlagged() == 0);
						if(twiGetStatus() == 0x50){
							xl = twiReadByteAck();
							while(twiIsFlagged() == 0);
							if(twiGetStatus() == 0x58){
								xh = twiReadByteAck();
								while(twiIsFlagged() == 0);
								if(twiGetStatus() == 0x58){
									yl = twiReadByteAck();
									while(twiIsFlagged() == 0);
									if(twiGetStatus() == 0x58){
										yh = twiReadByteAck();
									}
								}
							}
							
						}
					}
					
				}
			}
		}
	}*/
	
/*
	//Wait for the START condition to be send.
	while(!(TWCR & (1<<TWINT)));
	//Check if no error in status code (mask prescaler's 2 LSB bits).
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
	*/
	
	/*
	x = ((xH << 8) | (xL & 0xff)); //0xff should be mandatory because of the 2s complement
	
	//Turn on LED on PORTD0 if...
	if(x != 0){
		PORTD |= 1<<PORTD0;
	}
	else{
		PORTD &= ~(1<<PORTD0);
	}
	*/

}
//*****************************************
//Damien Monni - www.damien-monni.fr
//
//Read multiple registers from LSM303D.
//Read accelerator values (2 bytes per value) on X, Y and Z.
//Read registers are 0x28 - 0x29 / 0x2A - 0x2B / 0x2C - 0x2D 
//*****************************************
//Transfert number 50 to Arduino for test
//Wait 5s then transmit 50 via I2C/TWI to Arduino slave on address 4


#define F_CPU 8000000UL


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "monni_i2c.h"

//**********************************//
//Main
//**********************************//

int main(void){

	_delay_ms(5000);
	
	//Slave address : 4 - slave
	uint8_t slaveAddress = 4;
	
	if(twiInit(slaveAddress, 0) == 0x18){
		twiWriteByte(50);
		twiWaitFlag();
		/*if(twiGetStatus() == 0x28){
			twiWriteByte(data);
			twiWaitFlag();*/
			if(twiGetStatus() == 0x28){
				//return 1;
			}
		}
	}

}

//Arduino slave code
/**

#include <Wire.h>

void setup()
{
	Wire.begin(4);                // join i2c bus with address #4
	Wire.onReceive(receiveEvent); // register event
	Serial.begin(9600);           // start serial for output
}

void loop()
{
	//delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
	while(1 < Wire.available()) // loop through all but the last
	{
		char c = Wire.read(); // receive byte as a character
		Serial.print(c);         // print the character
	}
	int x = Wire.read();    // receive byte as an integer
	Serial.println(x);         // print the integer
}

**/
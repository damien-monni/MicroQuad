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

//7 bits accelerometer's address 
const uint8_t accelAdd = 0b0011101;

//7 bits gyro's address 
const uint8_t gyroAdd = 0b1101011;

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer



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
	
	//ATmega328p TWI initialisation 
	//Set SCL to 400kHz (for internal 8Mhz clock)
	TWSR = 0x00;
	TWBR = 0x02;
	
	//Accel initialisation
	while(twiWriteOneByte(accelAdd, 0x21, 0x18) == 0); //CTRL2 = 0x18 => full scale +/-8g
	while(twiWriteOneByte(accelAdd, 0x20, 0b01010111) == 0); //CTRL1 = 0b01010111 => 50Hz + 3 axis enable
	
	//Magneto initialisation
	while(twiWriteOneByte(accelAdd, 0x24, 0b01100100) == 0); //CTRL5 = 0b01100100 => High resolution and 6.25Hz
	while(twiWriteOneByte(accelAdd, 0x25, 0b00100000) == 0); //CTRL6 = 0b00100000 => +-4 gauss
	while(twiWriteOneByte(accelAdd, 0x26, 0b00000000) == 0); //CTRL7 = 0 => low power off and continuous conversion mode
	
	//Gyro initialisation
	while(twiWriteOneByte(accelAdd, 0x39, 0b00000000) == 0); //LOW_ODR disabled
	while(twiWriteOneByte(accelAdd, 0x23, 0x20) == 0); //CTRL4 = 0x20 => 2000dps full scale
	while(twiWriteOneByte(accelAdd, 0x20, 0x0F) == 0); //CTRL1 = 0x0F => Normal power mode, all axis enabled	
	
	//Wait for stabilisation
	_delay_ms(20);


	
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
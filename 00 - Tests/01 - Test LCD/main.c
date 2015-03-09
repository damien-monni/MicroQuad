//#define F_CPU 8000000UL //Not usefull, already on Makefile

#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "lcd_hd44780_avr.h"

int main(){

	LCDInit(LS_NONE);
	
	float f = 2.57;
	uint32_t a = f*100;
	
	LCDWriteFString(PSTR("Test LCD !"));
	
	while(1){

	}
}
//#define F_CPU 8000000UL //Not usefull, already on Makefile

#include <util/delay.h>
#include <avr/io.h>

#include "lcd_hd44780_avr.h"

int main(){

	LCDInit(LS_NONE);
	
	LCDWriteFString(PSTR("LCD Test !")); //Try without PSTR?
	
	while(1){

	}
}
/*
 * LCDDemo1.c
 *
 * Created: 29-12-2014 PM 12:52:58
 *  Author: Avinash
 */ 


#include <avr/io.h>
#include <avr/pgmspace.h>

#include "lib/lcd/lcd_hd44780_avr.h"

int main(void)
{
	LCDInit(LS_NONE);
	
	LCDWriteFString(PSTR("LCD Test !"));
	LCDWriteFStringXY(0,1,PSTR("By Avinash"));
	
	//infinite loop
	while(1);
}
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/io.h>

int main(){
	DDRD |= 1<<DDD1;
	
	while(1){
		PORTD ^= 1<<PORTD1;
		_delay_ms(1000);
	}
}
#define F_CPU 8000000UL

#include <util/delay.h>
#include <avr/io.h>

int main(){
	DDRB |= 1<<DDB0;
	
	while(1){
		PORTB ^= 1<<PORTB0;
		_delay_ms(1000);
	}
}
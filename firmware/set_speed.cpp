
#include <avr/interrupt.h>
#include <avr/io.h>
#include "set_speed.h"

void setSpeed(unsigned char clock) {
	cli(); 
	unsigned char a,b;
	TCCR1B = (0x0a-clock) | 0x40 ;
	CLKPR = 0x80;
	CLKPR = clock-1;
	sei();
}

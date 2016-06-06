
#include <avr/io.h>
#include "usi-spi.h"

void USI_SPI::init (void) {
	USICR = (1<<USIWM0) | (1<<USICS1) | (0<<USICS0) | (1<<USICLK);
	USIPP = 0x01;
} 

unsigned char USI_SPI::transfer(unsigned char data) {
	
	USISR = 0x00;
	USIDR = data;
	do {
		USICR |= 1<< USITC ;
		USICR |= 1<< USITC ;
	} while (USISR & 0x0f) ;
	return USIDR ;
}

void USI_SPI::send( unsigned char address,unsigned char* buffer, unsigned char bytes) {
	SENSOR_CS(0) ;
		for (unsigned char i=0; i!=bytes;i++) {
			transfer(address & 0x7f) ;
			transfer(*(buffer+i));
			address++;
		}
	SENSOR_CS(1) ;
}

unsigned char USI_SPI::receive(unsigned char address, unsigned char* buffer, unsigned char bytes) {
	SENSOR_CS(0);
	transfer(address | 0x80) ;
	for (unsigned char i=0; i!=bytes;i++) {
		*(buffer+i) = transfer(0);
	}
	SENSOR_CS(1);
}

//  CS - 0
//  write: R/W = 0 + ADDR, DATA
//  CS - 1

//  CS - 0
//  read: R/W = 1 + ADDR, DATA, DATA ...
//  CS - 1

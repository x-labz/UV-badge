
#include <avr/io.h>
#include <util/delay.h>
#include "font14.h"
#include <avr/pgmspace.h>
#include <string.h>
#include <avr/interrupt.h>
#include "set_speed.h"
#include "sharp_lcd.h"
#include "usi-spi.h"

uint8_t reverseByteWithShifts( uint8_t x )
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}




extern USI_SPI spi;

void lcdInit () {
	
	_delay_us(50);
	LCD_DISP(1);
	_delay_us(50);
	startExtCom();
	_delay_us(50);
}



void startExtCom() {
	// PWM
	TCCR1A = 0x10 ;  // OC1B toggle on compare match
	TCCR1B = 0x06; // CK/32  --> 1Mhz -> 61Hz | 0x0b --> ~2Hz
	// 2MHz: 0x07
	// 8MHz: 0x09
	OCR1B = 0x80; // compare reg.
	OCR1C = 0xff; // timer top val.
}

void lcdDispData(disp_elements* elements){ //, unsigned char cycle) {
	unsigned char ch;
	unsigned char size = sizeof(elements->strings) / sizeof(elements->strings[0]);
	unsigned char symSize = sizeof(elements->symbols) / sizeof(elements->symbols[0]);
	
	unsigned char outBuffer[20] ;
	unsigned char  byteWidth , bitWidth;
	unsigned char targetByte, targetBit ;
	unsigned char y, str, yPointer,  sourceByte, sourceBit ;
	
		
	for ( y = 0; y!=128; y++) {
				
		memset(outBuffer, 0xff, 20);
				
		outBuffer[0]=0x80;
		outBuffer[1]=reverseByteWithShifts(y+1);
		unsigned char drawLine = 0;
		
		if ( (elements->fullRefresh == 1) || ( elements->chartRefresh==1 && (y&0b11000000)!=0 ) ) {
			drawLine = 1;
		}
		else {			
			for (str=0; str != size ; str++) {
				if ( y >= elements->strings[str].startY && y <= elements->strings[str].endY && elements->strings[str].changed==1) {
					drawLine = 1;
					break;
				}
			}
			if (!drawLine) {
				for (unsigned char i=0; i != symSize ; i++) {
					if ( y >= elements->symbols[i].startY && y <= elements->symbols[i].endY && elements->symbols[i].changed==1) {
						drawLine = 1;
						break;
					}
				}
			}
		}
		
		if (drawLine) {
						
			for (str=0; str != size ; str++) {
				if ( y >= elements->strings[str].startY && y <= elements->strings[str].endY ) {
					yPointer = y-elements->strings[str].startY;
					targetByte = elements->strings[str].startX / 0x08;
					targetBit = elements->strings[str].startX % 0x08 ;
					
					ch=0;
					while ( elements->strings[str].chars[ch] != 0 ) {
						bitWidth = pgm_read_byte(&(Font14_array[elements->strings[str].chars[ch]-1].width))  ;
						byteWidth = bitWidth / 0x08 ;
						if (bitWidth % 0x08 != 0x00 ) {
							byteWidth++;
						}
						
						unsigned char pointerOffset,pointerOffsetOrig ;
						pointerOffsetOrig = 0xff;
						for (sourceBit=0; sourceBit!=bitWidth; sourceBit++ )
						{
							pointerOffset = byteWidth*yPointer+sourceBit/0x08 ;
							if (pointerOffsetOrig != pointerOffset) {
								pointerOffsetOrig = pointerOffset ;
								sourceByte = pgm_read_byte(  (unsigned char*)pgm_read_ptr(&Font14_array[elements->strings[str].chars[ch]-1].data)+pointerOffset) ;
							}
							if ( (sourceByte & (1<<((7-sourceBit%0x08)))) !=0 ) {
								outBuffer[2+(targetByte&0x0f)] &= 0xff ^ (1<<(7-targetBit));
							}
							targetBit++;
							if (targetBit == 0x08) {
								targetBit = 0x00;
								targetByte++;
							}
						}
						ch++;
						targetBit++;
						if (targetBit == 0x08) {
							targetBit = 0x00;
							targetByte++;
						}
					}
				}
			}
			
			for (unsigned char i=0;i != symSize ; i++) {
				if ( y >= elements->symbols[i].startY && y <= elements->symbols[i].endY && elements->symbols[i].code) {
					yPointer = y-elements->symbols[i].startY;
					targetByte = elements->symbols[i].startX / 0x08;
					targetBit = elements->symbols[i].startX % 0x08 ;
					bitWidth = pgm_read_byte(&(Font14_array[elements->symbols[i].code].width))  ;
					byteWidth = bitWidth / 0x08 ;
					if (bitWidth % 0x08 != 0x00 ) {
						byteWidth++;
					}
					
					unsigned char pointerOffset,pointerOffsetOrig ;
					pointerOffsetOrig = 0xff;
					for (sourceBit=0; sourceBit!=bitWidth; sourceBit++ )
					{
						pointerOffset = byteWidth*yPointer+sourceBit/0x08 ;
						if (pointerOffsetOrig != pointerOffset) {
							pointerOffsetOrig = pointerOffset ;
							sourceByte = pgm_read_byte(  (unsigned char*)pgm_read_ptr(&Font14_array[elements->symbols[i].code].data)+pointerOffset) ;
						}
						if ( (sourceByte & (1<<((7-sourceBit%0x08)))) !=0 ) {
							outBuffer[2+(targetByte&0x0f)] &= 0xff ^ (1<<(7-targetBit));
						}
						targetBit++;
						if (targetBit == 0x08) {
							targetBit = 0x00;
							targetByte++;
						}
					}
					
				}
			}
			
		}
		
		
		// --- WRITE-LINE ---
		if (drawLine) {
			CLEAR_BIT(PRR,1);
			spi.init();
			
			setSpeed(SP_2_MHz);
			LCD_SCS(1);
			
			for (unsigned char byte=0; byte!=20; byte++ )
			{
				spi.transfer(*(outBuffer+byte)); 
			}
			
			LCD_SCS(0);
			setSpeed(SP_8_MHz);
			SET_BIT(PRR,1);
		}
		// --- WRITE-LINE ---
		
	}
	elements->fullRefresh = 0;
	elements->chartRefresh = 0;
}
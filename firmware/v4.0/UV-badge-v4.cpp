
#define F_CPU 1000000UL  // 1 MHz

#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "sharp_lcd.h"
#include <avr/interrupt.h>
#include "set_speed.h"
#include <avr/sleep.h>
#include "usi-spi.h"

// --- MACRO ---

#define SET_BIT(PORT_NR,BIT) (PORT_NR |= (1<<BIT))
#define CLEAR_BIT(PORT_NR,BIT) (PORT_NR &= ~(1<<BIT))

#define VAL_MAX 24
#define AWAKE_TIME 60 // 1 min

#define SAMPLE_INTERVAL 1800 //15min //1800  // 30 min

#define SELECTOR_WIDTH 50
#define SELECTOR_HEIGHT 21

#define CHAR_WIDTH 9

#define STATE_IDLE	0
#define STATE_SENS	1
#define STATE_DONE	2
//#define STATE_RDY	2

#define  UV_DISABLE CLEAR_BIT(PORTB,PINB6)
#define  UV_ENABLE SET_BIT(PORTB,PINB6)

#define DISP_BAR 0
#define DISP_METER 1
#define DISP_FEET 2

typedef struct {
	int uvi_raw;
	int uvi;
	int uvi_offset;
	int temp;
	int dPres ;
	unsigned char rh;
	int uvi_1;
	int temp_1;
	int dPres_1 ;
	unsigned char rh_1;
	//unsigned long int uvi_sum;
	
	unsigned int dig_T1;
	int dig_T2;
	int dig_T3;
	
	unsigned int dig_P1;
	int dig_P2;
	int dig_P3;
	int dig_P4;
	int dig_P5;
	int dig_P6;
	int dig_P7;
	int dig_P8;
	int dig_P9;
	
	unsigned char dig_H1;
	int dig_H2, dig_H4,dig_H5;
	unsigned char dig_H3;
	signed char dig_H6;
	
} sensor_val ;

typedef struct {
	unsigned char uvi[VAL_MAX];
	signed char temp[VAL_MAX];
	int dPres[VAL_MAX] ;  // char --> int
	unsigned char rh[VAL_MAX];
	unsigned char poi;
} sensor_store ;

typedef union {
	unsigned char all;
	struct {
		unsigned char btn:1;
		unsigned char btn_1:1;
		unsigned char btn2:1;
		unsigned char btn2_1:1;
		unsigned char hibernate:1;
		unsigned char wakeup:1;
		unsigned char wdInt:1;
		unsigned char mode:1;
	}  ;
} flag_container ;

typedef union {
	unsigned char all;
	struct {
		unsigned char btn2long:1;
		unsigned char forceUpdate:1;
		unsigned char spare2:1;
		unsigned char spare3:1;
		unsigned char spare4:1;
		unsigned char spare5:1;
		unsigned char spare6:1;
		unsigned char spare7:1;
	}  ;
} flag_container2 ;

flag_container volatile flags ;
flag_container2 volatile flags2 ;


char static volatile altDispMode = DISP_BAR ;

static unsigned char volatile  sleepWalkTimer=AWAKE_TIME ;
static unsigned int bigTimer=0, btnTimeStamp = 0;

void initSensors(sensor_val* );
void measure(void);
void readSensors(sensor_val* );
unsigned int batt(void);
unsigned int UV_read(void) ;
unsigned char decToString ( char* ,  int , unsigned char ) ;
void select(unsigned char, disp_elements*);
void drawBar(disp_elements* , sensor_store* , unsigned char );

ISR( PCINT_vect ) {
	bool button1 = (PINB & (1<<PINB0)) ? 1 : 0 ;
	bool button2 = (PINB & (1<<PINB1)) ? 1 : 0 ;
	
	if (flags.hibernate == 1) {
		flags.wakeup = 1;
	}
	else {
		if (button1 != flags.btn_1 && button1) {
			flags.btn = 1;
		}

		if (button2 != flags.btn2_1 ) {
			if (!button2) {
				btnTimeStamp = bigTimer;
			}
			else {
				if ( btnTimeStamp+1 < bigTimer) {
					flags2.btn2long = 1;
				}
				else  {
					flags.btn2 = 1;
				}
			}
		}
	}
	flags.btn_1 = button1;
	flags.btn2_1 = button2;
}


//ISR( TIMER1_OVF_vect ) {
//timer++;
//}

ISR ( WDT_vect ) {
	SET_BIT(WDTCR,WDIE) ;
	flags.wdInt = 1;
}

ISR ( ADC_vect ) {
	return ;
}

unsigned char const selectCoords[] PROGMEM = {5,3, 67,3, 5,34, 67,34} ;

USI_SPI spi;


// --- MAIN ---

int main(void)
{
	static disp_elements dispElements = {
		{
			// SYMBOLS
			{0,61,62,1,68}, // batt
			// corners
			{17,0,0,0,0},
			{18,0,0,0,0},
			{19,0,0,0,0},
			{20,0,0,0,0},
			
			// texts
			{15,15,20,1,25},
			{14,80,20,1,25},
			{13,15,51,1,56},
			{16,81,51,1,56},
			
			//// BARs
			{21,  4,120,0,0},
			{21,  9,120,0,0},
			{21, 14,120,0,0},
			{21, 19,120,0,0},
			{21, 24,120,0,0},
			{21, 29,120,0,0},
			{21, 34,120,0,0},
			{21, 39,120,0,0},
			{21, 44,120,0,0},
			{21, 49,120,0,0},
			{21, 54,120,0,0},
			{21, 59,120,0,0},
			
			{21, 64,120,0,0},
			{21, 69,120,0,0},
			{21, 74,120,0,0},
			{21, 79,120,0,0},
			{21, 84,120,0,0},
			{21, 89,120,0,0},
			{21, 94,120,0,0},
			{21, 99,120,0,0},
			{21,104,120,0,0},
			{21,109,120,0,0},
			{21,114,120,0,0},
			{21,119,120,0,0} //,
			
		
		},
		{
			{{2,0},10,6,1,17} , //press
			{{3,0},80,6,1,17},//tmp
			{{4,0},10,36,1,47} , //uvi
			{{5,0},80,36,1,47}//, // rh
			//{{6,0},10,67,1,78},  //uvih
			//{{7,0},10,98,1,109}  //alt
			
		},
		1,
		1
	} ;
	
	sensor_val sensorValues ;
	
	sensorValues.uvi = 100;
	sensorValues.uvi_offset = 300;
	sensorValues.temp = 100;
	sensorValues.dPres = 0;
	sensorValues.rh = 110;
	
	//sensorValues.uvi_sum = 0;
	
	static sensor_store sensorStore ;
	
	unsigned int timeStamp ;
	
	sensorStore.poi = 0;
	
	flags.all = 0x87 ;
	flags2.all = 0b00000000;
	
	unsigned char selected=0;
	
	unsigned char dT = 1;
	
	setSpeed(SP_8_MHz);
	// -- initHw ;
	
	ACSRA = 0x80;  // disable An.comp.
	PRR = 0b00000100;
	
	// INIT PORTs
	
	//SET_BIT(PORTB,PINB0) ; // pullup
	//SET_BIT(PORTB,PINB1) ; // pullup
	PORTB = 0b00000111 ;   // PB5 analog IN, 
	
	//SET_BIT(DDRB,PINB3); // OC1B out
	//SET_BIT(DDRB,PINB4); // PB4 out (disp)
	//SET_BIT(DDRB,PINB6); // PB6 out (UV en)
	DDRB = 0b01011000 ;
	
	//SET_BIT(DDRA,PINA1); // PA1 out (spi DO)
	//SET_BIT(DDRA,PINA2); // PA2 out (spi SCL)
	//SET_BIT(DDRA,PINA4); // PA4 out (batt EN)
	//SET_BIT(DDRA,PINA6); // PA6 out (LCD CS)
	//SET_BIT(DDRA,PINA7); // PA7 out (sensor CS)
	DDRA = 0b11010110;
	
	//SET_BIT(PORTA,PINA4); // batt.EN -->H
	//CLEAR_BIT(PORTA,PINA7);
	//SET_BIT(PORTA,PINA7); // sens.CS. -->H
	// PA3 pullup
	PORTA = 0b10011000;
	
	// ADC SETUP
	
	ADCSRA = 0x06 ; // {/64} 125kHz @ 8MHz CPU clk.

	SET_BIT(DIDR0, ADC4D ); // dig.inp. disable ADC4
	SET_BIT(DIDR1, ADC8D ); // dig.inp. disable ADC8
	
	WDTCR = 0b00011000;
	WDTCR = 0b01000110; //1s

	// --- end init hw

	// PCINT
	SET_BIT(GIMSK,PCIE0);
	PCMSK0 = 0x00 ;
	PCMSK1 = 0x03; // EN PCINT8 - BTN INT + PCINT9
	
	// en Timer1 OVF int
	//SET_BIT(TIMSK,TOIE1) ;
	
	spi.init();

	lcdInit();
	
	initSensors(&sensorValues);
	
	sei();

	volatile unsigned int battLevel, prevBatt=99;
	unsigned char state_machine=0 ;
	//int uvihCurrent_1 = 100;
	int altRef = 10130;
	
	battLevel = batt();
	for (unsigned char idx = 0; idx != VAL_MAX; idx++) {
		sensorStore.dPres[idx] = 0;
	}
	
	while(1) {
		setSpeed(SP_8_MHz);
		
		if (flags.wakeup == 1) {
			flags.hibernate = 0;
			//timer = 0;
			//SET_BIT(TIMSK,TOIE1) ;
			
			prevBatt = battLevel;
			battLevel = batt();
			
			if ( battLevel != prevBatt) {
				dispElements.symbols[0].changed = 1;
				dispElements.symbols[0].code = (battLevel < 850 ? 12 : 0) ;
			}
			else {
				dispElements.symbols[0].changed = 0;
			}
			
			CLEAR_BIT(PRR,PRTIM1) ;
			LCD_DISP(1);
			//dT = 1;
			//WDTCR & 0b11011000 ;
			//WDTCR | 0b01000110 ;
			dispElements.fullRefresh = 1;
			dispElements.chartRefresh = 1;
			flags2.forceUpdate = 1;
			sleepWalkTimer = AWAKE_TIME;
		}
		
		if (flags.wdInt == 1 || flags.wakeup == 1) {
			flags.wakeup = 0;
			if (flags.wdInt) {
				flags.wdInt = 0;
				bigTimer+=dT ;
			}
			
			if (state_machine != STATE_SENS) {
				if (sleepWalkTimer != 0 || (sleepWalkTimer == 0 &&  bigTimer >= SAMPLE_INTERVAL ) ) {
					state_machine = STATE_SENS;
					measure();
					timeStamp = bigTimer ;
				}
			}

			if ( bigTimer >= SAMPLE_INTERVAL ) {   // ~30 min
				bigTimer = 0 ;
			}
			if (sleepWalkTimer != 0)  {
				sleepWalkTimer--;
				if (sleepWalkTimer == 0) {
					//CLEAR_BIT(TIMSK,TOIE1) ;
					flags.hibernate = 1;
					// LCD disp
					LCD_DISP(0);
					//dT = 8;
					//WDTCR & 0b11011000 ;
					//WDTCR | 0b01100001 ;
					SET_BIT(PRR,PRTIM1) ;
				}
			}
		}
		
		if ( state_machine==STATE_SENS && timeStamp != bigTimer ) {
			//if ( sleepWalkTimer==0 || (sleepWalkTimer!=0 && timeStamp != bigTimer) ) {
			sensorValues.dPres_1 = sensorValues.dPres;
			sensorValues.temp_1 = sensorValues.temp;
			sensorValues.uvi_1 = sensorValues.uvi ;
			sensorValues.rh_1 = sensorValues.rh ;
			
			readSensors(&sensorValues) ;
			
			UV_ENABLE ;
			setSpeed(SP_125_kHz);
			_delay_loop_1(36);
			setSpeed(SP_8_MHz);
			//UV_DISABLE;
			//UV_ENABLE ;
			unsigned int readValue = UV_read();
			UV_DISABLE;
			//int volatile test = batt();
			
			sensorValues.uvi_raw = ( int)((((uint32_t)readValue * (uint32_t)battLevel) >> 10) );
			
			if (sensorValues.uvi_raw>sensorValues.uvi_offset) {
				sensorValues.uvi = ((sensorValues.uvi_raw<<5) - (sensorValues.uvi_offset<<5) ) / 74 ; //53 ; 
			}
			else {
				sensorValues.uvi = 0;
			}
			//sensorValues.uvi_sum+=sensorValues.uvi ;
			
			state_machine = STATE_DONE ;
			
			if (bigTimer==0) {
				
				
				sensorStore.uvi[sensorStore.poi] = (unsigned char)(sensorValues.uvi >> 1 ) ;
				sensorStore.rh[sensorStore.poi] = (unsigned char)(sensorValues.rh >> 1 );
				sensorStore.temp[sensorStore.poi] = (signed char)((sensorValues.temp  + 100 )>>3);
				sensorStore.dPres[sensorStore.poi] = (int)(sensorValues.dPres_1-sensorValues.dPres);
				
				sensorStore.poi++;
				
				if (sensorStore.poi == VAL_MAX)	{
					sensorStore.poi = 0;
				}
				
				if (sleepWalkTimer !=0) {
					drawBar(&dispElements, &sensorStore, selected);
					flags2.forceUpdate = 1;
				}
				
			}
		}
		
		// BUTTONS
		
		if (flags.btn || flags.btn2 || flags2.btn2long) {
			flags2.forceUpdate = 1;
			sleepWalkTimer = AWAKE_TIME;
			
			if (flags.btn) {
				flags.btn = 0;

				if ( sleepWalkTimer!=0 ) {
					selected++;
					selected &= 0x03;
					select(selected,&dispElements);
					drawBar(&dispElements, &sensorStore, selected);
				}
			}
			if (flags.btn2 == 1) {
				flags.btn2 = 0;
				
				if (selected == 1) {
					flags.mode = !flags.mode ;
					dispElements.strings[1].changed = 1;
				}
				if (selected == 0) {
					altDispMode++ ;
					if (altDispMode > 2) altDispMode = 0;
					
					dispElements.symbols[5].code = (altDispMode == 0) ? 15 : (altDispMode == 1 ? 23 : 24) ;
					//flags2.altMode = !flags2.altMode ;
					dispElements.strings[0].changed = 1;
				}
				
				dispElements.fullRefresh = 1;
				dispElements.symbols[6].code = flags.mode ? 22 : 14 ;
				//dispElements.symbols[22].code = flags2.altMode ? 25 : 24 ;
			}
			
			if (flags2.btn2long) {
				flags2.btn2long = 0;
				//if (selected == 2) {
					//sensorValues.uvi_sum= 0;
				//}
				if (selected == 0) {
					altRef = sensorValues.dPres;
					dispElements.strings[0].changed = 1;
				}
				
				if (selected == 2) {
					sensorValues.uvi_offset = sensorValues.uvi_raw ;
				}
			}
		}
		
		//  ---- DISP. DATA ----
		
		if ((state_machine==STATE_DONE && sleepWalkTimer!=0) || flags2.forceUpdate) {
			flags2.forceUpdate = 0;
			dispElements.strings[2].changed = 0;
			if ( sensorValues.uvi != sensorValues.uvi_1 ) {
				decToString(dispElements.strings[2].chars,(int)sensorValues.uvi  ,1) ;
				dispElements.strings[2].changed = 1;
			}

			if ( sensorValues.temp != sensorValues.temp_1 || dispElements.strings[1].changed == 1 ) {
				int value ;
				if (flags.mode == 0) {
					value = (int)sensorValues.temp ;
				}
				else {
					value = ((int)sensorValues.temp * 9)/5 +320 ;
				}
				decToString(dispElements.strings[1].chars,value ,1) ;
				dispElements.strings[1].changed = 1;
			}
			else {
				dispElements.strings[1].changed = 0;
			}
			
			if ( sensorValues.dPres != sensorValues.dPres_1 || dispElements.strings[0].changed == 1) {
				int val ;
				if (altDispMode == 0) {
					val = ( int)sensorValues.dPres ;
				}
				else {
					val = (int)( 10000*((7*(int32_t)(altRef-sensorValues.dPres_1)+((int32_t)(altRef-sensorValues.dPres)))/8) / ( int32_t)(altDispMode == 1 ? 1185 : 361 ));
				}
								
				decToString(dispElements.strings[0].chars, val ,1) ;
				//dispElements.strings[0].changed = 1;
				//decToString(dispElements.strings[5].chars, (int)( 10000*((7*(int32_t)(altRef-sensorValues.dPres_1)+((int32_t)(altRef-sensorValues.dPres)))/8) / ( int32_t)(!flags2.altMode ? 1185 : 361 )) ,1) ;
				//decToString(dispElements.strings[5].chars, (int)(((int32_t)(altRef-sensorValues.dPres))<<14) / ( int32_t)(!flags2.altMode ? 1941 : 591 ) ,1) ;
				//dispElements.strings[5].changed = 1;
			}
			else {
				dispElements.strings[0].changed = 0;
			}
			
			
			dispElements.strings[3].changed = 0;
			if ( sensorValues.rh != sensorValues.rh_1 ) {
				decToString(dispElements.strings[3].chars,( int)sensorValues.rh ,0) ;
				dispElements.strings[3].changed = 1;
			}
			
			//dispElements.strings[4].changed = 0;
			//int uvihCurrent = (int)(sensorValues.uvi_sum / (unsigned long int)1800) ;
			//if ( uvihCurrent_1 != uvihCurrent ) {
				//decToString(dispElements.strings[4].chars, uvihCurrent ,1) ;
				//dispElements.strings[4].changed = 1;
			//}
			//uvihCurrent_1 = uvihCurrent ;
			
			lcdDispData(&dispElements) ;
			
		}
		
		//state_machine = STATE_IDLE ;
				
		setSpeed(SP_125_kHz);
		
		if (flags.hibernate == 1) {
			set_sleep_mode( SLEEP_MODE_PWR_DOWN ) ;
		}
		else {
			set_sleep_mode(SLEEP_MODE_IDLE) ;
		}

		sleep_enable();
		sleep_cpu();
		sleep_disable();
	}
}

unsigned int batt(void) {
	CLEAR_BIT(PRR,PRADC) ;
	CLEAR_BIT(PORTA,PINA4);
	ADCSRA = 0b10001110 ;
	ADMUX = 0b10000100 ; // ref 1.1V - sel. ADC4
	SET_BIT(ADCSRA,ADSC) ;			
	set_sleep_mode(SLEEP_MODE_ADC) ;
	sleep_enable();
	sleep_cpu();
	sleep_disable();
	while (ADCSRA & 0x40) ;  // wait...
	unsigned int result = ADCL ;
	result += (ADCH<<8) ;
	CLEAR_BIT(ADCSRA,ADEN ); // disable AD
	SET_BIT(PORTA,PINA4);
	SET_BIT(PRR,PRADC);
	return result ;
}

unsigned int UV_read(void) {
	CLEAR_BIT(PRR,PRADC);
	ADCSRA = 0b10001110 ;
	ADMUX = 0b00001000 ; // ref VCC - ADC8
	SET_BIT(ADCSRA,ADSC) ;               
	set_sleep_mode(SLEEP_MODE_ADC) ;
	sleep_enable();
	sleep_cpu();
	sleep_disable();
	while (ADCSRA & 0x40) ;  // wait...
	unsigned int result = ADCL ;
	result += (ADCH<<8) ;	
	CLEAR_BIT(ADCSRA,ADEN ); // disable AD
	SET_BIT(PRR,PRADC);
	return result ;
}

void initSensors(sensor_val* sensorValues) {
	// SPI W -0 | R -1
	
	// F2 01 - hum.oversamp. 1x
	// F4 0b00100100 - ovr.smp. 1x - sleep mode
	// F5 0b00000000 - cfg
	
	
	SENSOR_CS(0) ;
	spi.transfer(0xf2 & 0x7f);
	spi.transfer(1);
	spi.transfer(0xf4 & 0x7f);
	spi.transfer(0b00100100);
	spi.transfer(0xf5 & 0x7f);
	spi.transfer(0);
	
	// calib
	// calib26..calib41 0xE1…0xF0
	// calib00..calib25 0x88…0xA1
	
	unsigned char buffer[26];
	
	spi.receive(0x88,buffer,26) ;
	
	sensorValues->dig_T1 = buffer[0] | (buffer[1]<<8) ;
	sensorValues->dig_T2 = buffer[2] | (buffer[3]<<8) ;
	sensorValues->dig_T3 = buffer[4] | (buffer[5]<<8) ;
	sensorValues->dig_P1 = buffer[6] | (buffer[7]<<8) ;
	sensorValues->dig_P2 = buffer[8] | (buffer[9]<<8) ;
	sensorValues->dig_P3 = buffer[10] | (buffer[11]<<8) ;
	sensorValues->dig_P4 = buffer[12] | (buffer[13]<<8) ;
	sensorValues->dig_P5 = buffer[14] | (buffer[15]<<8) ;
	sensorValues->dig_P6 = buffer[16] | (buffer[17]<<8) ;
	sensorValues->dig_P7 = buffer[18] | (buffer[19]<<8) ;
	sensorValues->dig_P8 = buffer[20] | (buffer[21]<<8) ;
	sensorValues->dig_P9 = buffer[22] | (buffer[23]<<8) ;
	
	sensorValues->dig_H1 = buffer[25];
	
	spi.receive(0xe1,buffer,7) ;
	sensorValues->dig_H2 = buffer[0] | (buffer[1]<<8) ;
	sensorValues->dig_H3 = buffer[2] ;
	sensorValues->dig_H4 = (buffer[3]<<4) | (buffer[4]&0x0f) ;
	sensorValues->dig_H5 = ((buffer[4]&0xf0) >> 4) | (buffer[5]<<4);
	sensorValues->dig_H6 = buffer[6] ;

}

void readSensors(sensor_val* data) {

	CLEAR_BIT(PRR,PRUSI);
	spi.init();
	
	unsigned char buffer[8];
	spi.receive(0xf7,buffer,8) ;
	
	SET_BIT(PRR,PRUSI);
	
	int32_t t_fine, adc_T;
	adc_T = ((int32_t)buffer[3] << 12) | ((int32_t)buffer[4] << 4) | (buffer[5] >> 4) ;
	
	int32_t var1, var2;
	var1 = ((((adc_T>>3)-((int32_t)data->dig_T1<<1))) * ((int32_t)data->dig_T2)) >> 11;
	var2 = (((((adc_T>>4)-((int32_t)data->dig_T1)) * ((adc_T>>4)-((int32_t)data->dig_T1))) >> 12) * ((int32_t)data->dig_T3)) >> 14;
	t_fine = var1 + var2;
	data->temp = (int)(((t_fine * 5 + 128) >> 8) / 10) ;
	
	
	// pressure
	int32_t adc_P = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((int32_t)buffer[2] >> 4) ;
	
	int32_t p_var1, p_var2;
	uint32_t p;
	p_var1 = (((int32_t)t_fine)>>1)-(int32_t)64000;
	p_var2 = (((p_var1>>2) * (p_var1>>2)) >> 11 ) * ((int32_t)data->dig_P6);
	p_var2 = p_var2 + ((p_var1*((int32_t)data->dig_P5))<<1);
	p_var2 = (p_var2>>2)+(((int32_t)data->dig_P4)<<16);
	p_var1 = (((data->dig_P3 * (((p_var1>>2) * (p_var1>>2)) >> 13 )) >> 3) + ((((int32_t)data->dig_P2) * p_var1)>>1))>>18;
	p_var1 =((((32768+p_var1))*((int32_t)data->dig_P1))>>15);
	if (p_var1 != 0)
	{
		p = (((uint32_t)(((int32_t)1048576)-adc_P)-(p_var2>>12)))*3125;
		if (p < 0x80000000)
		{
			p = (p << 1) / ((uint32_t)p_var1);
		}
		else
		{
			p = (p / (uint32_t)p_var1) * 2;
		}
		p_var1 = (((int32_t)data->dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
		p_var2 = (((int32_t)(p>>2)) * ((int32_t)data->dig_P8))>>13;
		p = (uint32_t)((int32_t)p + ((p_var1 + p_var2 + data->dig_P7) >> 4));
		data->dPres = (int)(p / 10) ;
	}
	
	// humidity
	int32_t adc_H = ((int32_t)buffer[6] << 8) | (buffer[7] )  ;
	
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine-((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14)-(((int32_t)data->dig_H4) << 20)-(((int32_t)data->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)data->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)data->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)data->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r-(((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)data->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	data->rh = (unsigned char) (v_x1_u32r>>22);
}

void measure(void) {
	CLEAR_BIT(PRR,PRUSI);
	spi.init();
	unsigned char data = 0b00100101;
	spi.send(0xf4,&data,1);
	SET_BIT(PRR,PRUSI);
}



unsigned char decToString ( char* string, int rem, unsigned char dig) {
	// HEX --> DEC

	unsigned char tx[7];
	const int dec[] = {10000,1000,100,10} ;
	unsigned char first = 0;
	unsigned char t=0;
	unsigned char pointer =0;
	
	if (rem<0) {
		*(string) = 0x01; //0x2d;
		pointer++;
		rem *=-1;
	}
	
	for (unsigned char i=0; i!=4; i++)
	{
		tx[t] =  rem/dec[i];
		if (tx[t] != 0 || first !=0) {
			t++;
			first = 1;
		}
		rem = rem%dec[i];
	}
	if (dig) {
		if (t==0) {
			tx[t]=0;
			t++;
		}
		tx[t] = 0xff;
		t++;
	}
	tx[t]=  rem%10;
	
	for(unsigned char i=0;i!=(t+1);i++) {
		*(string+pointer) = (tx[i] != 0xff ? (tx[i]+ 0x03) : 0x02) ;
		pointer++;
	}
	*(string+pointer) = 0;

	return pointer;
}

void select(unsigned char selected, disp_elements* dispElements) {
	selected = selected<<1 ;

	dispElements->symbols[1].startX = pgm_read_byte(&selectCoords[selected]) ;
	dispElements->symbols[1].startY = pgm_read_byte(&selectCoords[(selected)+1]) ;
	dispElements->symbols[1].endY = dispElements->symbols[1].startY + 3;
	
	dispElements->symbols[2].startX = pgm_read_byte(&selectCoords[selected]) + SELECTOR_WIDTH;
	dispElements->symbols[2].startY = pgm_read_byte(&selectCoords[(selected)+1]) ;
	dispElements->symbols[2].endY = dispElements->symbols[2].startY + 3;
	
	dispElements->symbols[3].startX = pgm_read_byte(&selectCoords[selected]) ;
	dispElements->symbols[3].startY = pgm_read_byte(&selectCoords[(selected)+1]) + SELECTOR_HEIGHT ;
	dispElements->symbols[3].endY = dispElements->symbols[3].startY + 3;
	
	dispElements->symbols[4].startX = pgm_read_byte(&selectCoords[selected]) + SELECTOR_WIDTH;
	dispElements->symbols[4].startY = pgm_read_byte(&selectCoords[(selected)+1]) + SELECTOR_HEIGHT  ;
	dispElements->symbols[4].endY = dispElements->symbols[4].startY + 3;
	
	dispElements->fullRefresh = 1;
}

void drawBar(disp_elements* dispElements, sensor_store* values, unsigned char selected) {
	unsigned char poi, poi_1;
	signed char val;
	void* basePoi;
	
	switch (selected) {
		case 2: basePoi = values->uvi;
		break;
		case 1: basePoi = values->temp;
		break;
		case 0: basePoi = values->dPres;
		break;
		case 3: basePoi = values->rh;
		break;
	}
	
	//for (unsigned char i=0; i!=VAL_MAX; i++) {
	for ( signed char i=VAL_MAX-1; i>=0; i--) {
		poi = values->poi+i;
		if (poi>=VAL_MAX ) {
			poi-=VAL_MAX;
		}
		poi_1 = poi-1;
		if (poi_1>=VAL_MAX ) {
			poi_1-=VAL_MAX;
		}
		
		if (selected == 0) {
			int tempVal = *( (int*)basePoi +poi );
			if (tempVal>60) tempVal = 60;
			if (tempVal<-60) tempVal = -60;
			val =  (i != VAL_MAX-1) ? (signed char)(tempVal) + (125-dispElements->symbols[9+i+1].startY) : 30 ;
		}
		if (selected==1 || selected==2) {
			val = *( (unsigned char*)basePoi +poi )  ;
		}
		if (selected == 3) {
			val = *( (signed char*)basePoi +poi )  ;
		}
		
		if (val > 61) {
			val = 61;
			} else if (val < 0) {
			val = 0;
		}
		
		dispElements->symbols[9+i].startY = 125 - val ;
		dispElements->symbols[9+i].endY = 125 - val + 2 ;
	}
	
	dispElements->chartRefresh = 1;
}



#define F_CPU 1000000UL  // 1 MHz

#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "sharp_lcd.h"
#include <avr/interrupt.h>
#include "set_speed.h"
#include <avr/sleep.h>
#include "usi-spi.h"


// --- MACRO ---

#define SET_BIT(PORT_NR,BIT) (PORT_NR |= (1<<BIT))
#define CLEAR_BIT(PORT_NR,BIT) (PORT_NR &= ~(1<<BIT))

#define VAL_MAX 12 
#define SUM_CNT 1024  //4 //

#define SELECTOR_WIDTH 50
#define SELECTOR_HEIGHT 20

#define CHAR_WIDTH 9 

#define STATE_IDLE	0
#define STATE_SENS	1
#define STATE_UV	2
#define STATE_RDY	3

#define  UV_DISABLE CLEAR_BIT(PORTB,PINB6)
#define  UV_ENABLE SET_BIT(PORTB,PINB6)

volatile unsigned char btnVal = 1;

typedef struct {
	int uvi;
	int temp;
	int dPres ;
	unsigned char rh;
	int uvi_1;
	int temp_1;
	int dPres_1 ;
	unsigned char rh_1;
	
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
	uint32_t sumUVI;
	int32_t sumTemp;
	int32_t sumDPres;
	uint32_t sumRh;
	
	unsigned char uvi[VAL_MAX];
	char temp[VAL_MAX];
	char dPres[VAL_MAX] ;
	unsigned char rh[VAL_MAX];
	unsigned char poi;
} sensor_store ;

typedef union {
	unsigned char all;
	struct {
		unsigned char btn:1;
		unsigned char btnLong:1;
		unsigned char hibernate:1;
		unsigned char wakeup:1;
		unsigned char btn2:1;
		unsigned char justWokeUp:1;
		unsigned char spare1:1;
		unsigned char spare0:1;
	}  ;
} flag_container ;

flag_container volatile flags ;

static unsigned int volatile timer = 0, btnTimeStamp=0 ;
static unsigned char volatile sysCnt = 0, sysCnt_1=0;

void initHw(void);
void initSensors(sensor_val* );
void measure(void);
void readSensors(sensor_val* );
unsigned int batt(void);
unsigned int UV_read(void) ;
unsigned char decToString ( char* ,  int , unsigned char ) ;
void select(unsigned char, disp_elements*);
void drawBar(disp_elements* , sensor_store* , unsigned char );

ISR( PCINT_vect ) {

	if (flags.hibernate == 1) {
		flags.wakeup = 1;
	}
	
	unsigned char btn = (PINB & (1<<PINB0)) ? 1 : 0 ;
	if ( btn != btnVal ) {
		btnVal = btn ;
		if (btn == 0) {
			if (flags.btn == 0) {
				timer = 0;
			}
			btnTimeStamp = timer ;
		}
		else {
			if (timer > (btnTimeStamp+20)) {
				flags.btn = 1;
				flags.btnLong = 0 ;
				
				if(timer > (btnTimeStamp+380)) {
					flags.btnLong = 1 ;
				}	
			}
		}
	}
	
	if ( (PINB & (1<<PINB1))  == 0) {
		flags.btn2 = !flags.btn2 ;
	}
	
}


ISR( TIMER1_OVF_vect ) {
	timer++;
	sysCnt++;
}

unsigned char const selectCoords[] PROGMEM = {3,3, 67,3, 3,34, 67,34} ; 

	

USI_SPI spi;

void initHw(void) {
	
	ACSRA = 0x80;  // disable An.comp.
	PRR = 0b00000100; 
	
	// INIT PORTs
	
	//SET_BIT(PORTB,PINB0) ; // pullup
	//SET_BIT(PORTB,PINB1) ; // pullup
	PORTB = 0x03;
	
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
	PORTA = 0b10010000; 
	
	
	// ADC SETUP
	
	ADCSRA = 0x06 ; // {/64} 125kHz @ 8MHz CPU clk.

	SET_BIT(DIDR0, ADC4D ); // dig.inp. disable ADC4
	SET_BIT(DIDR1, ADC8D ); // dig.inp. disable ADC8
}

// --- MAIN ---

int main(void)
{	
	static disp_elements dispElements = {
		{
			// SYMBOLS
			{0,25,110,1,116}, // batt
			// corners
			{17,0,0,0,0},
			{18,0,0,0,0},
			{19,0,0,0,0},
			{20,0,0,0,0},
			
			// texts
			{15,19,20,1,25},
			{14,92,20,1,25},
			{13,24,51,1,56},
			{16,88,51,1,56},
			
			//// BARs
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
			{21,119,120,0,0},
			
			{22,22,82,1,87}
		},
		{
			{{2,0},5,5,1,16} , //uvi
			{{3,0},80,5,1,16},//tmp
			{{4,0},5,36,1,47} , //prs
			{{5,0},80,36,1,47} , // rh
			{{6,0},6,67,1,78}  //uvih
			
		},
		0,
		0
	} ;
	
	sensor_val sensorValues = {100,0,0,0,0,100,100,100} ;
	static sensor_store sensorStore ;
	
	setSpeed(SP_8_MHz); 
	initHw();
	
	sensorStore.poi = 0;
	
	
	flags.all = 0; 
	flags.btn =1;
	
	unsigned char selected=0;
				
	// PCINT
	SET_BIT(GIMSK,PCIE0);
	PCMSK0 = 0x00 ;   
	PCMSK1 = 0x03; // EN PCINT8 - BTN INT + PCINT9
		
	// en Timer1 OVF int
	SET_BIT(TIMSK,TOIE1) ;	
		
	spi.init();

	lcdInit();
	
		
	set_sleep_mode(SLEEP_MODE_IDLE) ;
		
	initSensors(&sensorValues);
		
	sei();
		
	unsigned int bigTimer=0;
		
	uint32_t uvih = 0;
	unsigned char uvihDisp = 0xff;
	volatile unsigned int battLevel, prevBatt=99;	
	unsigned char state_machine=0 ;
	
	battLevel = batt(); 
		
	
	while(1) {
		setSpeed(SP_8_MHz);  
		if (flags.wakeup == 1) {
			flags.wakeup = 0;
			flags.hibernate = 0;
			flags.justWokeUp = 1;
			
			set_sleep_mode(SLEEP_MODE_IDLE) ;

			LCD_DISP(1);
			uvih = 0;
			bigTimer = SUM_CNT-1;
		}
		
		if (flags.btn) {
			if ( flags.btnLong==0) {
				
				flags.btn = 0;
				selected++;
				selected &= 0x03;
			
				select(selected,&dispElements);
				drawBar(&dispElements, &sensorStore, selected);
				lcdDispData(&dispElements) ;
			}
		
			else {
				flags.btn = 0;
				flags.hibernate = 1;
				
			
				// LCD disp
				LCD_DISP(0);
			
				// DEEP SLEEP
				setSpeed(SP_125_kHz);
				set_sleep_mode(SLEEP_MODE_PWR_DOWN) ;
				sleep_enable();
				sleep_cpu();
				sleep_disable();
			}
		}
		
		if (state_machine!=STATE_IDLE) {

			if (sysCnt_1  < sysCnt) {
				if(state_machine == STATE_SENS) {	 
					readSensors(&sensorValues) ;
					UV_ENABLE ;
				}

				if ( state_machine == STATE_UV) {	 
					sensorValues.uvi_1 = sensorValues.uvi ;
					sensorValues.uvi = ( int)((((uint32_t)1000 * (uint32_t)UV_read() * (uint32_t)battLevel) >> 20) );
					if (!flags.btn2) {
						if (sensorValues.uvi>(int)285) {
							sensorValues.uvi = ((sensorValues.uvi<<5) - (280<<5) ) / 74 ;
						}
						else {
							sensorValues.uvi = 0;
						}
					}
					uvih += sensorValues.uvi;
					UV_DISABLE;			
				}
			
				if (state_machine == STATE_RDY) {	 
					sensorStore.sumUVI += sensorValues.uvi;
					sensorStore.sumRh += sensorValues.rh;
					sensorStore.sumDPres += sensorValues.dPres;
					sensorStore.sumTemp += sensorValues.temp;
						
					unsigned char charNum;
						
					if ( sensorValues.uvi != sensorValues.uvi_1 ) {
						charNum = decToString(dispElements.strings[2].chars,(int)sensorValues.uvi  ,1) ;
						dispElements.strings[2].startX = (64 - charNum * CHAR_WIDTH) / 2 ;
						dispElements.strings[2].changed = 1;
					}
					else {
						dispElements.strings[2].changed = 0;
					}
						
					if ( sensorValues.temp != sensorValues.temp_1 ) {
						charNum = decToString(dispElements.strings[1].chars,(int)sensorValues.temp ,1) ;
						dispElements.strings[1].startX = (64 - CHAR_WIDTH * charNum) / 2 + 64 ;
						dispElements.strings[1].changed = 1;
					}
					else {
						dispElements.strings[1].changed = 0;
					}
						
					if ( sensorValues.dPres != sensorValues.dPres_1 ) {
						charNum = decToString(dispElements.strings[0].chars,( int)sensorValues.dPres ,1) ;
						dispElements.strings[0].startX = (64 - CHAR_WIDTH * charNum) /2 ;
						dispElements.strings[0].changed = 1;
					}
					else {
						dispElements.strings[0].changed = 0;
					}
						
					if ( sensorValues.rh != sensorValues.rh_1 ) {
						charNum = decToString(dispElements.strings[3].chars,( int)sensorValues.rh ,0) ;
						dispElements.strings[3].startX = (64 - CHAR_WIDTH * charNum)/2 +64 ;
						dispElements.strings[3].changed = 1;
					}
					else {
						dispElements.strings[3].changed = 0;
					}
						
					unsigned char uvihCurrent = (unsigned char)(uvih / 1800) ;
					if ( uvihDisp != uvihCurrent ) {
						charNum = decToString(dispElements.strings[4].chars,( int)uvihCurrent ,1) ;
						dispElements.strings[4].startX = (64 - CHAR_WIDTH * charNum)/2  ;
						dispElements.strings[4].changed = 1;
					}
					else {
						dispElements.strings[4].changed = 0;
					}
						
					if ( battLevel != prevBatt) {
						dispElements.symbols[0].changed = 1;
						dispElements.symbols[0].code = (battLevel < 850 ? 12 : 0) ;
					}
					else {
						dispElements.symbols[0].changed = 0;
					}
						
					bigTimer++;
					if ( bigTimer == SUM_CNT ) {
						bigTimer = 0;
							
						prevBatt = battLevel;
						battLevel = batt();
																									
						if (flags.justWokeUp == 1) {
							flags.justWokeUp = 0;
						}
						else {
							sensorStore.uvi[sensorStore.poi] = (unsigned char)(sensorStore.sumUVI >> 11) ; //%2
							sensorStore.rh[sensorStore.poi] = (unsigned char)(sensorStore.sumRh >> 11 ); //%2
							sensorStore.dPres[sensorStore.poi] = (char)((((sensorStore.sumDPres>> 10)-10000)>>3)+30 ); //%8
							sensorStore.temp[sensorStore.poi] = (char)(sensorStore.sumTemp >> 14)+12; //%16
							
							//sensorStore.uvi[sensorStore.poi] = (unsigned char)(sensorStore.sumUVI >> 3) ; //%2
							//sensorStore.rh[sensorStore.poi] = (unsigned char)(sensorStore.sumRh >> 3); //%2
							//sensorStore.dPres[sensorStore.poi] = (char)((((sensorStore.sumDPres>> 2)-10000)>>3)+30 ); //%8
							//sensorStore.temp[sensorStore.poi] = (char)(sensorStore.sumTemp >> 6)+12; //%16
							
							sensorStore.poi++;
						}
						if (sensorStore.poi == VAL_MAX)	{
							sensorStore.poi = 0;
						}
						
						sensorStore.sumUVI = 0;
						sensorStore.sumRh = 0;
						sensorStore.sumDPres = 0;
						sensorStore.sumTemp = 0;
							
						drawBar(&dispElements, &sensorStore, selected);
					}
						
					lcdDispData(&dispElements) ;
				
				}
				state_machine++;
				sysCnt_1 = sysCnt ;
				
				if (state_machine > STATE_RDY) { 
					state_machine=STATE_IDLE ;
				} 
			}	
		}
		
		if (sysCnt == 250) {  // ~2s
			state_machine = STATE_SENS;
			sysCnt_1 = sysCnt = 0;
			measure();
			
		}
		
		setSpeed(SP_125_kHz);
		sleep_enable();
		sleep_cpu();
		sleep_disable();
		
		
	}
}

unsigned int batt(void) {
	CLEAR_BIT(PRR,0) ; 

	CLEAR_BIT(PORTA,PINA4);
	
	SET_BIT(ADCSRA,ADEN);

	ADMUX = 0b10000100 ; // ref 1.1V - sel. ADC4
	sleep_enable();
	
	sleep_cpu();
	sleep_disable();
	while (ADCSRA & 0x40) ;  // wait...				
	
	unsigned int result = ADCL ;
	result += (ADCH<<8) ;
	
	CLEAR_BIT(ADCSRA,ADEN ); // disable AD
	SET_BIT(PORTA,PINA4);  
	
	SET_BIT(PRR,0);
	return result ; 
}

unsigned int UV_read(void) {
	CLEAR_BIT(PRR,0);
	SET_BIT(ADCSRA,ADEN);
		
	ADMUX = 0b00001000 ; // ref VCC - ADC8
	
	sleep_enable();

	sleep_cpu();
	sleep_disable();
	while (ADCSRA & 0x40) ;  // wait...				
	
	unsigned int result = ADCL ;
	result += (ADCH<<8) ;
	
	CLEAR_BIT(ADCSRA,ADEN ); // disable AD
	SET_BIT(PRR,0);
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

	CLEAR_BIT(PRR,1);
	spi.init(); 
	
	unsigned char buffer[8];
	spi.receive(0xf7,buffer,8) ;
	
	SET_BIT(PRR,1);
	
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
	CLEAR_BIT(PRR,1);
	spi.init(); 
	unsigned char data = 0b00100101;
	spi.send(0xf4,&data,1);
	SET_BIT(PRR,1);
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
	dispElements->symbols[1].endY = dispElements->symbols[1].startY + 2;
	
	dispElements->symbols[2].startX = pgm_read_byte(&selectCoords[selected]) + SELECTOR_WIDTH;
	dispElements->symbols[2].startY = pgm_read_byte(&selectCoords[(selected)+1]) ;
	dispElements->symbols[2].endY = dispElements->symbols[2].startY + 2;
	
	dispElements->symbols[3].startX = pgm_read_byte(&selectCoords[selected]) ;
	dispElements->symbols[3].startY = pgm_read_byte(&selectCoords[(selected)+1]) + SELECTOR_HEIGHT ;
	dispElements->symbols[3].endY = dispElements->symbols[3].startY + 2;
	
	dispElements->symbols[4].startX = pgm_read_byte(&selectCoords[selected]) + SELECTOR_WIDTH;
	dispElements->symbols[4].startY = pgm_read_byte(&selectCoords[(selected)+1]) + SELECTOR_HEIGHT  ;
	dispElements->symbols[4].endY = dispElements->symbols[4].startY + 2;
	
	dispElements->fullRefresh = 1;
}

void drawBar(disp_elements* dispElements, sensor_store* values, unsigned char selected) {
	unsigned char poi, offset, div;
	char val;
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
	
	for (unsigned char i=0; i!=VAL_MAX; i++)
	{	
		poi = values->poi+i;
		if (poi>=VAL_MAX ) {
			poi-=VAL_MAX;
		}
		
		if (selected==1 || selected==2) {
			val = *( (unsigned char*)basePoi +poi )  ;
		}
		else {
			val = *( (char*)basePoi +poi )  ;
		}
				
		if (val > 61) {
			val = 61;
		}
		if (val < 0) {
			val = 0;
		}
		
		dispElements->symbols[9+i].startY = 125 - val ;
		dispElements->symbols[9+i].endY = 125 - val + 2 ;
	}
	
	dispElements->chartRefresh = 1;
}


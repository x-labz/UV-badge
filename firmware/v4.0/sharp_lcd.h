
#ifndef SHARP_LCD_H_
#define SHARP_LCD_H_

#define SET_BIT(PORT_NR,BIT) (PORT_NR |= (1<<BIT))
#define CLEAR_BIT(PORT_NR,BIT) (PORT_NR &= ~(1<<BIT))

#define LCD_DISP(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTB,PINB4) : SET_BIT(PORTB,PINB4)
#define LCD_SCS(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTA,PINA6) : SET_BIT(PORTA,PINA6)
#define LCD_SCLK(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTA,PINA2) : SET_BIT(PORTA,PINA2)
#define LCD_SI(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTA,PINA1) : SET_BIT(PORTA,PINA1)

typedef struct
{
	char chars[7];
	unsigned char startX;
	unsigned char startY:7;
	unsigned char changed:1;
	unsigned char endY;
} text_type ;

typedef struct {
	unsigned char code ;
	unsigned char startX;
	unsigned char startY:7;
	unsigned char changed:1;
	unsigned char endY;
} symbol_type;

typedef struct {
	symbol_type symbols[33];
	text_type strings[4];
	unsigned char fullRefresh;
	unsigned char chartRefresh;
} disp_elements ;

uint8_t reverseByteWithShifts( uint8_t );
extern void lcdInit(void) ;
extern void lcdClear(void);

void startExtCom(void);

extern void lcdDispData(disp_elements*); 

#endif /* SHARP-LCD_H_ */
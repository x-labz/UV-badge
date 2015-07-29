
#ifndef USI_SPI_H_
#define USI_SPI_H_

#define SET_BIT(PORT_NR,BIT) (PORT_NR |= (1<<BIT))
#define CLEAR_BIT(PORT_NR,BIT) (PORT_NR &= ~(1<<BIT))

#define SENSOR_CS(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTA,PINA7) : SET_BIT(PORTA,PINA7)
#define LCD_SCS(LEVEL) LEVEL==0 ? CLEAR_BIT(PORTA,PINA6) : SET_BIT(PORTA,PINA6)

class USI_SPI {
	public:
		void init (void) ;
		unsigned char transfer (unsigned char);
		void send(unsigned char,unsigned char* , unsigned char );
		unsigned char receive(unsigned char, unsigned char* , unsigned char) ;
};

#endif 
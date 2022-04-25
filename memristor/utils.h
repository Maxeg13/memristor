#ifndef PARAMS
#define PARAMS

#define CHAN_N 64
#define LDAC PD2
#define DDR_SPI DDRB
#define DD_MISO PB4
#define DD_MOSI PB3
#define DD_SCK PB5
#define SPI_SS PB2 

#define BAUD 9600                                   
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
#define SET_BIT(port, pos) port|=(1<<pos)
#define RESET_BIT(port, pos) port&=~(1<<pos)
#define DUMMY_BYTE 0

#include <avr/io.h>

uint8_t chan_addrs[8] = {	0,1,2,3 ,  4,5,6,7};  //while for one channel

uint8_t* REGS_OUT[] = {&PORTD, &PORTD, &PORTD, &PORTB, 
						&PORTB, &PORTC, &PORTC, &PORTC};
						
uint8_t SYNC_PINS[] = {PD5, PD6, PD7, PB0, 		//pd3 for multiplex or pd5
						PB1, PC2, PC3, PC4};
						

			
void gatherMult()
{
	RESET_BIT(PORTD, 6);
	RESET_BIT(PORTD, 7);				
	RESET_BIT(PORTD, 5);
	RESET_BIT(PORTC, 4);
	
	
	SET_BIT(PORTC, 1);				
	SET_BIT(PORTB, 2);
	SET_BIT(PORTB, 1);
	SET_BIT(PORTB, 0);
}
void separMult()
{
	//RESET_BIT(PORTB, 1);
	(PORTB, 2);
	RESET_BIT(PORTC, 1);
	
	SET_BIT(PORTC, 4);						
	SET_BIT(PORTD, 6);
	SET_BIT(PORTD, 7);
	SET_BIT(PORTB, 0);
	SET_BIT(PORTD, 5);
}


void set_reverser(uint8_t ind, uint8_t x)
{
	if(0)
	switch(ind)
	{
		case 0:  
		if(x)
			PORTD|=(1<<5);
		else
			PORTD&=~(1<<5);
		break;
		
				case 1:  
		if(x)
			PORTD|=(1<<6);
		else
			PORTD&=~(1<<6);
		break;
		
				case 2:  
		if(x)
			PORTD|=(1<<7);
		else
			PORTD&=~(1<<7);
		break;
		
				case 3:  
		if(x)
			PORTB|=(1<<0);
		else
			PORTB&=~(1<<0);
		break;
		
				case 4:  
		if(x)
			PORTB|=(1<<1);
		else
			PORTB&=~(1<<1);
		break;
		
				case 5:  
		if(x)
			PORTB|=(1<<2);
		else
			PORTB&=~(1<<2);
		break;
		
				case 6:  
		if(x)
			PORTC|=(1<<2);
		else
			PORTC&=~(1<<2);
		break;
		
				case 7:  
		if(x)
			PORTB|=(1<<4);
		else
			PORTB&=~(1<<4);
		break;

	}
}
						
#endif
#ifndef PARAMS
#define PARAMS

//pd3 <-> pd5

#define CHAN_N 64
#define LDAC PD2
#define DDR_SPI DDRB
#define DD_MISO PB4
#define DD_MOSI PB3
#define DD_SCK PB5
#define SPI_SS PB2 

#define BAUD 9600                                   
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
#define SET_PIN(port, pos) port|=(1<<pos)
#define RESET_PIN(port, pos) port&=~(1<<pos)
#define DUMMY_BYTE 0

#include <avr/io.h>

uint8_t chan_addrs[8] = {	0,1,2,3 ,  4,5,6,7};  //while for one channel

struct Pin_t {
	uint8_t* reg_out;
	uint8_t pin;
};
					
struct Pin_t SYNC_PINS[] = {	{&PORTD, PD3}, {&PORTD, PD6}, {&PORTD, PD7}, {&PORTB, PB0}, 		//pd3 for multiplexing or pd5 for 64
						{&PORTB, PB1}, {&PORTC, PC2}, {&PORTC, PC3}, {&PORTC, PC4} };
						
						
void set_pin(struct Pin_t pin) {
	SET_PIN(*pin.reg_out, pin.pin);
}

void reset_pin(struct Pin_t pin) {
	RESET_PIN(*pin.reg_out, pin.pin);
}

struct Pin_t sep1_pin = {&PORTD, 5};	
struct Pin_t sep2_pin = {&PORTD, 6};
struct Pin_t sep3_pin = {&PORTD, 7};	
struct Pin_t sep4_pin = {&PORTB, 0};
struct Pin_t resister_pin = {&PORTC, 4};		
void gatherMult()
{
	reset_pin(sep2_pin);
	reset_pin(sep3_pin);
	reset_pin(sep1_pin);
	reset_pin(resister_pin);
	
	// 3-4
	SET_PIN(PORTC, 1);	
	// 2-3	
	SET_PIN(PORTB, 2);
	// 1-2
	SET_PIN(PORTB, 1);
	
	set_pin(sep4_pin);
}
void usualMult()
{
	RESET_PIN(PORTB, 1);
	RESET_PIN(PORTB, 2);
	RESET_PIN(PORTC, 1);
	
	set_pin(resister_pin);
	set_pin(sep2_pin);
	set_pin(sep3_pin);
	set_pin(sep4_pin);
	set_pin(sep1_pin);
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
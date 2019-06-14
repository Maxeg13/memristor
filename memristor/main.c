#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#define MIG 300



#define LDAC PD2
#define SYNC PD3

#define DDR_SPI DDRB
#define DD_MISO PB4
#define DD_MOSI PB3
#define DD_SCK PB5
#define SPI_SS PB2

#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
//#define DD_SS PB0
uint16_t x16=0;
uint16_t y16=0;
uint8_t t1;
uint8_t t2;
uint8_t Tt2;
uint8_t T;
uint8_t send8;
uint8_t ptr=0;
char c;
int ctr;
int event_ctr;
int time_step=4;
int eventN=100;

void timer_init()
{
    TCCR2A = 0;        // set entire TCCR1A register to 0
    TCCR2B = 0;
    //    TCCR2B |= (1<<CS22) | (1<<CS20); // PRESCALER 1024
    TCCR2B |= (1<<CS21) ;
    // enable Timer1 overflow interrupt:
    TIMSK2 = (1 << TOIE2);
}

void uart_init(unsigned int ubrr)
{
	
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void SPI_MasterInit()
{
/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)|(1<<SPI_SS);
/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
	//SPSR = (0<<SPI2X);
}
//(1<<RXCIE0)|
void OneSend(uint16_t x)
{
	x+=2048;
	PORTD&=~(1<<SYNC);
	//_delay_us(30);  
	send8 = (x >> 8);
	send8 &= 0b00001111;
	SPI_WriteByte(send8);
	send8=x;
	//send8&=0b11111111;
	SPI_WriteByte(send8);		
	PORTD|=(1<<SYNC);

}


void SPI_WriteByte(uint8_t data)
{
   //PORTB &= ~(1<<SPI_SS);
   SPDR = data;
  while(!(SPSR & (1<<SPIF)));
   //PORTB |= (1<<SPI_SS); 
}

void main(void)
{
	sei();
	SPI_MasterInit();
	timer_init();
    DDRD = 0b000001100;	
	uart_init(BAUDRATE);
	
	
	
    while(1)
    {
		
		//x16+=1;	
/*		
		OneSend(x16);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(t1);
		
		OneSend(0);
				PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(T);
		
				OneSend(y16);				
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(t2);
		OneSend(0);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		_delay_ms(50);
*/

    }

}

ISR(TIMER2_OVF_vect)
{

	if(ctr==time_step)
	{
		if(event_ctr==0)
		{
		OneSend(x16);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		}
		else if(event_ctr==t1)
		{
		OneSend(0);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		}
		else if(event_ctr==T)
		{
		OneSend(y16);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		}
		else if(event_ctr==Tt2)
		{
		OneSend(0);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		}
	
		ctr=0;
		event_ctr++;
		if(event_ctr==eventN)
			event_ctr=0;
	}
	ctr++;
		
		/*
			OneSend(x16);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(t1);
		
		OneSend(0);
				PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(T);
		
				OneSend(y16);				
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		
		_delay_ms(t2);
		OneSend(0);
		PORTD&=~(1<<LDAC);
		PORTD|=(1<<LDAC);
		_delay_ms(50);
		*/
}

ISR(USART_RX_vect)
{
	switch(ptr)
	{
		case 0:
		x16=UDR0*16;
		break;
		case 1:	
		y16=UDR0*16;
		break;
		case 2:
		t1=UDR0;
		break;		
		case 3:
		t2=UDR0;
		break;	
		case 4:
		T=UDR0;
		break;
	}
	Tt2=T+t2;
	//UDR0=x16/16;
	ptr++;
	ptr%=5;
}
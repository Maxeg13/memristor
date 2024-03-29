#define F_CPU 16000000UL

#include <avr/io.h>
//#include <avr/interrupt.h>

#include <util/delay.h>

#define MIG 300



#define LDAC PD2
#define SYNC PD3

#define DDR_SPI DDRB
#define DD_MISO PB4
#define DD_MOSI PB3
#define DD_SCK PB5
#define SPI_SS PB2
#define N 1010
#define BAUD 9600                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
//#define DD_SS PB0
uint16_t x16=0;
uint8_t x8;

uint8_t send8;
uint16_t ptr=0;
uint16_t ptrh=0;
int16_t masi[N];
char c;
uint8_t _adc;


int eventN=2;
int ADC_cnt;

/* Функция инициализация АЦП */
void ADC_Init(){
 ADCSRA |= (1 << ADEN) // Включаем АЦП
 |(1 << ADPS1)|(1 << ADPS0)|(1 << ADPS1);    // устанавливаем предделитель преобразователя на 8
 ADMUX |= (0 << REFS1)|(1 << REFS0) //выставляем опорное напряжение, как внешний ИОН
 |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // снимать сигнал будем с  входа PC0 
}


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
	UCSR0B = (1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void SPI_MasterInit()
{
/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)|(1<<SPI_SS);
/* Enable SPI, Master, set clock rate fck/16 */
	//SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0);
	//SPSR = (0<<SPI2X);
}
//(1<<RXCIE0)|
void OneSend(uint16_t x)
{
	//x+=2048;
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
	//sei();
	SPI_MasterInit();
	//timer_init();
    DDRD = 0b000001100;	
	//uart_init(BAUDRATE);
	ADC_Init();
	
			ADCSRA |= (1 << ADSC); 
	ADCL;
	ADCL;
	
    while(1)
    {


x16=((ADCL)|(ADCH<<8));
ADCSRA |= (1 << ADSC); 
x16-=512;

masi[ptr]=x16;


_delay_us(60);

//masi[ptr]+=(masi[ptrh]>>1);
masi[ptr]+=0.49*masi[ptrh];

x16=masi[ptr];

//x16=masi[ptr]+masi[ptrh];
x16+=512;


ptr++;
if(ptr==(N))
	ptr=0;

ptrh=ptr+1;
if(ptrh==(N))
	ptrh=0;

//x16=x8;
x16=x16<<1;
OneSend(x16);
    }

}

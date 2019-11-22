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
#define BAUD 9600                                   
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL

//три возможных режима
typedef enum
{
	CUSTOM,
	VAC,
	PROGRAM
} MODE;

//CUSTOM - ручной режим 
//VAC - режим вольт-амперной характеристики
//PROGRAM - режим программирования проводимости мемристора
MODE MD=CUSTOM;//CUSTOM - режим по умолчанию
int16_t VAC16=0, VAC16_H=0, VAC16_HH=0;
int16_t prog_val=0;
int16_t x16=0;
int16_t y16=0;
uint8_t sync=0;
uint8_t t1=2;
uint8_t t2=2;
uint8_t dTt2=10;
uint8_t dT;
uint8_t T;
uint8_t pos_phase=1;
uint8_t send8;
uint8_t ptr=0, UDP_cnt;
uint8_t PROGRAM_done=0;
char c;
uint8_t ADC_on;
uint16_t _adc;
int ctr;

uint16_t accum;
int event_ctr;
int time_step=6;//3
int eventN=100;
int ADC_cnt;
uint8_t ADCH_, ADCL_;
uint8_t accum_cnt;

//функция инициализаци АЦП
//АЦП используется для регистрации тока, проходящего через мемристор
void ADC_Init(){
 ADCSRA |= (1 << ADEN) // Включаем АЦП
 //ADCSRA&=~(1 << ADPS1)|(1 << ADPS0)|(1 << ADPS1);
 |(1 << ADPS1)|(1 << ADPS0)|(1 << ADPS1);    // устанавливаем предделитель преобразователя на 8
 ADMUX |= (0 << REFS1)|(1 << REFS0) //выставляем опорное напряжение, как внешний ИОН
 |(0 << MUX0)|(0 << MUX1)|(0 << MUX2)|(0 << MUX3); // снимать сигнал будем с  входа PC0 
}

//функция инициализации таймера
// Таймер используется для формирования временной последовательности значений, формируемых с ЦАПа
void timer_init()
{
    TCCR2A = 0;        
    TCCR2B = 0;
    
    TCCR2B |= (1<<CS21) ;    
    TIMSK2 = (1 << TOIE2);
}

//функция инициализации UART-интерфейса
void uart_init(unsigned int ubrr)
{	
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

//функция инициализации SPI
void SPI_MasterInit()
{
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)|(1<<SPI_SS);
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
}

//функция управления ЦАПом 
// при этом, управление регистром LDAC должно использоваться 
//вне функции в перспепктиве создания многоканальной схемы
void setDAC(int16_t x)//_____________bipolar!!! and <<4 larger
{
	x=-x;
	x+=2048;
	PORTD&=~(1<<SYNC);
	send8 = (x >> 8);
	send8 &= 0b00001111;
	SPI_WriteByte(send8);
	send8=x;
	SPI_WriteByte(send8);		
	PORTD|=(1<<SYNC);

}


void SPI_WriteByte(uint8_t data)
{
   SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}

void main(void)
{
	sei();
	SPI_MasterInit();
	timer_init();
    DDRD = 0b000001100;	
	uart_init(BAUDRATE);
	ADC_Init();
	
			ADCSRA |= (1 << ADSC); 
	ADCL;
	ADCL;
	
    while(1)
    {


    }

}

//главный цикл работы контроллера
//Здесь устанавливаются значения ЦАП и передаются данные на компьютер
//через UART-интерфейс
ISR(TIMER2_OVF_vect)
{
	if(ctr>time_step)
	{
		if(MD==CUSTOM)
		{
			if(event_ctr==(1))
			{	
			ADCL_=ADCL;
			ADCH_=ADCH;
			UDR0=ADCL_;			
			}
			
			if(event_ctr==(2))
			{	
			UDR0=ADCH_;				
			}
			
			
			
			if(event_ctr==0)
			{
			UDR0=255;
			setDAC(x16);
			PORTD&=~(1<<LDAC);
			PORTD|=(1<<LDAC);
			}
			
			else if(event_ctr==t1)
			{
				
			setDAC(0);
			PORTD&=~(1<<LDAC);
			PORTD|=(1<<LDAC);
			}
			else if(event_ctr==dT)
			{		
			setDAC(y16);
			PORTD&=~(1<<LDAC);
			PORTD|=(1<<LDAC);
		
			}
			else if(event_ctr==(dT+1))
				ADCSRA |= (1 << ADSC); 

			else if(event_ctr==dTt2)
			{
			if ( ( UCSR0A & (1<<UDRE0)) )			
				UDR0=_adc;	
			accum=0;
			ADC_on=0;
			accum_cnt=0;			
			setDAC(0);
			PORTD&=~(1<<LDAC);
			PORTD|=(1<<LDAC);

			}		

		}
		else if(MD==VAC)
		{			
			static int i=0;
			i++;						
			
			switch(UDP_cnt)
			{
				case 0:					
				 ADCSRA |= (1 << ADSC); 
				UDR0=255;
				
				break;
				
				
				
				case 1:		
				ADCL_=ADCL;
				ADCH_=ADCH;
				
				UDR0=ADCL_;
				break;
				
				case 2:
				UDR0=ADCH_;
				
				break;
				
				
				case 3:
								
				//VAC16_HH=VAC16_H;
				VAC16_H=VAC16;
				
				if(pos_phase)
				{
					VAC16+=48;
					if(VAC16>(y16-1))//just positive
					{
					pos_phase=0;				
					}
				}
				else
				{
					VAC16-=48;
					if(VAC16<(-x16+1))//just positive
					{
					pos_phase=1;
					//VAC16=-x16;				
					}
				}	
				
				
				
				UDR0=VAC16_H>>4;
				setDAC(VAC16);	
				PORTD&=~(1<<LDAC);
				PORTD|=(1<<LDAC);
				
			}
						
			
			UDP_cnt++;
			UDP_cnt%=4;

			
		}
		else if(MD==PROGRAM)
		{
			T=16;
			static uint16_t adc_h;
			
			if(event_ctr==(1))
			{	
				UDR0=PROGRAM_done;
			}

			if(event_ctr==(2))//ADC GET 
			{	
				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_;
			}
			
			if(event_ctr==(3))//ADC GET CONTINUE
			{	
				
				UDR0=ADCH_;
				
				_adc=(ADCL_|(ADCH_ <<8));
				adc_h=((uint16_t)(512)-_adc);
				
				if((adc_h)==(uint16_t)(t1))
				{
					PROGRAM_done=1;
					prog_val=0;
				}
			}
			
			if(event_ctr==0)
			{
				UDR0=255;
				
				
				prog_val+=32;
				
				if(prog_val==(-x16+32))
					prog_val=0;
				else
				if(prog_val>(t2<<4))
					prog_val=-x16;
				
				if(PROGRAM_done)
					prog_val=0;
				
				setDAC(prog_val);
				PORTD&=~(1<<LDAC);
				PORTD|=(1<<LDAC);
			}			
			else if(event_ctr==7)//t1
			{
				setDAC(0);
				PORTD&=~(1<<LDAC);
				PORTD|=(1<<LDAC);
			}
			else if(event_ctr==9)//dT
			{		
				setDAC(y16);
				PORTD&=~(1<<LDAC);
				PORTD|=(1<<LDAC);
			}
			else if(event_ctr==(9+1))
				ADCSRA |= (1 << ADSC); 

			else if(event_ctr==14)//
			{

			accum=0;
			ADC_on=0;
			accum_cnt=0;
			
			setDAC(0);
			PORTD&=~(1<<LDAC);
			PORTD|=(1<<LDAC);

			}
		}
		
		
		
		
		
		ctr=0;
		event_ctr++;

		if(event_ctr>T)
			event_ctr=0;
	}
	ctr++;
}

//прием команд в зависимости от режима
ISR(USART_RX_vect)
{
	switch(ptr)
	{
		case 0:
		if(UDR0!=255)
		{
			sync=0;
			ptr--;
			ptr%=7;
		}
		else
			sync=1;
		break;
		case 1:
		MD=UDR0;
		if(MD==VAC)
			time_step=4;//5
		else
			time_step=6;//4
		
		if(MD==PROGRAM)
		{
			//PROGRAM_start=1;
			PROGRAM_done=0;
			prog_val=0;
		}
		break;
		case 2:
		x16=UDR0<<4;
		break;
		case 3:	
		y16=UDR0<<4;
		break;
		case 4:
		t1=UDR0;
		break;		
		case 5:
		t2=UDR0;
		break;	
		case 6:
		dT=UDR0;
		break;
		case 7:
		T=UDR0;
		break;
	}
	dTt2=dT+t2;
	//UDR0=x16/16;
	ptr++;
	ptr%=8;
}
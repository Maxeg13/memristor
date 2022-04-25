//#NEW PINOUT WITH MULTIPLEXING
// enum chans? (for code reading)


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define MIG 300

#include "utils.h"

// режимы
typedef enum
{
	CUSTOM,
	VAC,
	PROGRAM,
	GATHER_MULT,
	SEPAR_MULT,
	ONE_SHOT,
	ANALYZE,
	MODE_SET
} MODE;

typedef enum
{
	CHAN_1,
	CHAN_2,
	CHAN_3,
	CHAN_4
} CHAN_I;

//CUSTOM - ручной режим 
//VAC - режим вольт-амперной характеристики
//PROGRAM - режим программирования проводимости мемристора
MODE MD=CUSTOM;//CUSTOM - режим по умолчанию



uint8_t STAT_N = 17;
uint8_t STAT_CYCLE = 5;
uint8_t BIG_STAT_N;

								//0,1,2,3 ,  4, 5, 6, 7}; 
int16_t voltage16=0, voltage16_h=0, voltage16_hh=0;
int16_t proging_val=0;
int16_t x16_grad;
int16_t x16=0;
int16_t x16_simple;
int16_t ref16=0;
int16_t reset16=0;
uint8_t sync=0;
uint8_t t1=2;//useless
uint8_t t2=2;
uint8_t dTt2=10;
uint8_t dT;
uint8_t T;
uint8_t pos_phase=1;
uint8_t STAT_dt_step=0;
uint8_t STAT_V_step=0;
uint8_t ptr=0, UDP_cnt;// WHAAAAT???
uint8_t PROGRAM_done=0;
uint8_t chan=0;
char c;
uint16_t _adc;
uint16_t an_cnt=0, an_cnt_fast=0;
uint8_t reverted[CHAN_N]={0,0,0,0,0,0,0,0, 
						0,0,0,0,0,0,0,0,
						0};
int ctr;

int event_cnt;
int time_step=6;//3
int eventN=100;
int ADC_cnt;
uint8_t ADCH_, ADCL_, ADCH__, ADCL__;
//uint

//функция управления ЦАПом 
// при этом, управление регистром LDAC должно использоваться 
//вне функции в перспепктиве создания многоканальной схемы
void prepareSetDAC(int16_t x,uint8_t chan)//_____________bipolar!!! and <<4 larger
{
	static uint8_t send8;
	
	x=-x;
	x+=2048;
	*(REGS_OUT[chan>>3])&=~(1<<SYNC_PINS[chan>>3]);
	send8 = (x >> 8);
	send8 &= 0b00001111;
	send8|= (chan_addrs[chan%8]);
	SPI_WriteByte(send8);
	send8=x;
	SPI_WriteByte(send8);		
	*(REGS_OUT[chan>>3])|=(1<<SYNC_PINS[chan>>3]);
}

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
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<SPI_SS);
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA);//|(1<<SPR1)|(0<<SPR0);
}




void prepareResetDAC(int8_t chan)//_____________bipolar!!! and <<4 larger
{
	PORTD&=~(1<<SYNC_PINS[chan>>3]);
	SPI_WriteByte(0b00001000|chan_addrs[chan%8]); // magic numbers, fuck
	SPI_WriteByte(0);		
	PORTD|=(1<<SYNC_PINS[chan>>3]);
}



void SPI_WriteByte(uint8_t data)
{
   SPDR = data;
  while(!(SPSR & (1<<SPIF)));
}


void setDAC(){
	PORTD&=~(1<<LDAC);
	PORTD|=(1<<LDAC);	
}


void main(void)
{
	BIG_STAT_N = STAT_N*STAT_CYCLE;
	
	for (uint8_t i=0; i< 8;i++)
		chan_addrs[i]=chan_addrs[i]<<4;
	
	DDRC= 0b00011110;
	DDRD =0b11111111;	
	DDRB= 0b00011111;
	sei();
	SPI_MasterInit();
	timer_init();
	
	uart_init(BAUDRATE);
	ADC_Init();
	
	ADCSRA |= (1 << ADSC); 
	ADCL;
	ADCL;


	//for(int i=0;i<8;i++)
		//set_reverser(i,1);
	
	//set_reverser(0,0);
	for (int i=0;i<CHAN_N;i++)
	{
		prepareSetDAC(0,i);
	}
	setDAC();	
	
	//separMult();
	//пустой цикл программы (главный цикл основан на прерваниях)	
    while(1)
    {
//	PORTB|=3;
//	_delay_ms(20);
//	PORTB&=~3;
//	_delay_ms(20);
		//PORTD=PORTD^(PORTD&(1<<6));
		//PORTD=0xFF^PORTD;
    }

}

//главный цикл работы контроллера
//Здесь устанавливаются значения ЦАП и передаются данные на компьютер
//через UART-интерфейс
//данный участок кода повторяется при переполнении TIMER2
ISR(TIMER2_OVF_vect)
{
	if(ctr>time_step)
	{
		if(MD==CUSTOM)
		{
			if(event_cnt==(1))
			{	
			ADCL_=ADCL;
			ADCH_=ADCH;
			UDR0=ADCL_;			
			}
			
			if(event_cnt==(2))
			{	
			UDR0=ADCH_;				
			}
			
			
			
			if(event_cnt==0)
			{
			UDR0=255;
			prepareSetDAC(0,chan);
			//prepareSetDAC(x16,2);
			setDAC();
			}
			
			else if(event_cnt==t1)
			{
				
			//prepareSetDAC(0,chan);
			//setDAC();
			}
			else if(event_cnt==dT)
			{		
			//prepareSetDAC(0,chan);
			//setDAC();
		
			}
			else if(event_cnt==(dT+1))
				ADCSRA |= (1 << ADSC); 

			else if(event_cnt==dTt2)
			{
			if ( ( UCSR0A & (1<<UDRE0)) )			
				UDR0=_adc;			
			//prepareSetDAC(0,chan);
			//prepareSetDAC(0,2);
			//setDAC();

			}		

		}
		else if(MD == MODE_SET) {
			
		}
		else if(MD==VAC)
		{			
			//static int i=0;
			//i++;						
			
			switch(UDP_cnt)
			{
				case 0:					
				 ADCSRA |= (1 << ADSC); 
				UDR0=255;
				
				break;
				
				
				
				case 1:	
				ADCL__=	ADCL_;
				ADCH__ = ADCH_;
				ADCL_=ADCL;
				ADCH_=ADCH;
				
				UDR0=ADCL_;
				break;
				
				case 2:
				UDR0=ADCH_;				
				break;				
				
				case 3:								
					
					voltage16_h=voltage16;
					
					if(pos_phase)
					{
								//PORTC=0b00000010;
			//PORTB=0b00011111;
			//PORTD=0b11101100;
						voltage16+=32;
						if(voltage16>(ref16-1))
						{
						pos_phase=0;				
						}
					}
					else
					{
						//PORTB=0;
			//PORTC=0;
			//PORTD=0;
						voltage16-=32;
						if(voltage16<(-x16+1))
						{
						pos_phase=1;									
						}
					}	
					
					
					
					UDR0=voltage16>>4;
					prepareSetDAC(voltage16,chan);
					//prepareSetDAC(voltage16,1);
					//prepareSetDAC(voltage16,2);
					//prepareSetDAC(voltage16,3);
					//prepareSetDAC(voltage16,4);
					//prepareSetDAC(voltage16,5);
					//prepareSetDAC(voltage16,6);
					//prepareSetDAC(voltage16,7);				
					setDAC();
				case 4:
					UDR0=DUMMY_BYTE;	
				
			}
						
			
			UDP_cnt++;
			UDP_cnt%=5;
		}
		else if(MD==PROGRAM)
		{
			T=16;
			static uint16_t adc_h;
			

			if(event_cnt==0)//dT
			{		
				UDR0=255;
			
				prepareSetDAC(ref16,chan);
				setDAC();
				ADCSRA |= (1 << ADSC); 
			}
			else if(event_cnt == 1)
			{
				UDR0 =PROGRAM_done;
			}	

			else
			if(event_cnt==2)//ADC GET 
			{					 
				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_;   //2
			}
			
			//ADC make CONTINUE
			//make decision
			else
			if(event_cnt==3)
			{	
				
				UDR0=ADCH_; //3
				
				_adc=(ADCL_|(ADCH_ <<8));
				adc_h=((uint16_t)(512)-_adc);
				
				//optimization
				//put some window val here maybe?
				//be  carefull with -
				if((adc_h)<(uint16_t)(t1))
				{
					proging_val = -x16;  //set!
				}
				else if((adc_h)<(uint16_t)(t1+1)) //done!
				{

					PROGRAM_done=1;
					proging_val=0;
					prepareSetDAC(ref16,chan);
					setDAC();
				}
				
			}
			//DACset proging val
			else if(event_cnt==4)
			{
				UDR0 =DUMMY_BYTE;		
				if(PROGRAM_done)
					proging_val=0;	
				
				prepareSetDAC(proging_val,chan);
				setDAC();
							
				if(proging_val == -x16)
				{
					proging_val=0;
				}					
				else
				if(proging_val>(t2<<4))
				{
					proging_val= -x16;
				}	
				else
				{
					proging_val+=32;
				}	
			}
			else if(event_cnt==5)//
			{	
				UDR0 =DUMMY_BYTE;			
				prepareSetDAC(0,chan);
				setDAC();
			}
			
			

		}
		else if(MD == ONE_SHOT)
		{
			//готовим reset
				if(event_cnt==1){
				separMult();
			}//reseting
			else if(event_cnt==2)
			{
				prepareSetDAC(reset16,CHAN_4);
				prepareSetDAC(reset16,CHAN_3);
				setDAC();
			}
			else if(event_cnt==3)
			{
				prepareSetDAC(0,CHAN_4);
				prepareSetDAC(0,CHAN_3);
				setDAC();
			}		//is reset		
			else if(event_cnt==4)
			{
				gatherMult();
			}
			else if(event_cnt==5)
			{
				prepareSetDAC(x16,CHAN_4);				
				setDAC();
			}		
			else if(event_cnt==6)
			{
				prepareSetDAC(0,CHAN_4);				
				setDAC();
			}	
			
			//пнули		
			//посмотрим, что вышло
			else if(event_cnt==7)
			{
				UDR0=255;//1
				separMult();				
			}			
			else if(event_cnt==8)
			{
				prepareSetDAC(ref16,CHAN_4);
				setDAC();				
				ADCSRA |= (1 << ADSC); 
			}	
			else if(event_cnt==9)
			{
				//prepareSetDAC(ref16,3);
				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_;//2
			}
			else if(event_cnt==10)
			{
				UDR0=ADCH_; //3 1st chan
				
				prepareSetDAC(0,CHAN_4);
				setDAC();
			}		
			//4й просмотрен
			else if(event_cnt==11)
			{				 
				prepareSetDAC(ref16,CHAN_3);
				setDAC();
				
				ADCSRA |= (1 << ADSC); 
			}
			else if(event_cnt==12)
			{		
				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_; //4
			}
			else if(event_cnt == 13)
			{
				UDR0=ADCH_; // 5
				
				prepareSetDAC(0,CHAN_3);
				setDAC();
				event_cnt=0;	
			}
			
		}
		
		
		else if(MD == ANALYZE)//5 by 5
		{
			//reset
			if(event_cnt==0)
			{
				prepareSetDAC(reset16, chan);
				setDAC();
				UDR0=255;
			}
			else if(event_cnt==1)
			{
				prepareSetDAC(0, chan);				
				setDAC();
			}
			//create set impulse
			else if(event_cnt==3)
			{
				if(an_cnt<(BIG_STAT_N))
				{
					STAT_V_step=0;					 
				}
				else if(an_cnt<(BIG_STAT_N*2))
				{
					STAT_V_step=1;
				}
				else if(an_cnt<(BIG_STAT_N*3))
				{
					STAT_V_step=2;
				}
				else if(an_cnt<(BIG_STAT_N*4))
				{
					STAT_V_step=3;
				}
				else if(an_cnt<(BIG_STAT_N*5))
				{
					STAT_V_step=4;
				}////////////						
				//prepareSetDAC(x16, chan);
				UDR0=STAT_V_step;
				x16_grad = (-(STAT_V_step+1)*8 )<<4;//16
				prepareSetDAC(x16_grad, chan);
				setDAC();
				
				
				if(an_cnt_fast<(STAT_N))//20 us
				{
					STAT_dt_step=0;									
				}
				else if(an_cnt_fast<(STAT_N*2))//80 us
				{
					STAT_dt_step=1;
					for(int i=0;i<40;i++)//28
					{
					setDAC();
					}
				}
				else if(an_cnt_fast<(STAT_N*3))//220 us
				{
					STAT_dt_step=2;
					for(int i=0;i<160;i++)
					{
					setDAC();
					}
				}
				else if(an_cnt_fast<(STAT_N*4))//900 us
				{
					STAT_dt_step=3;
					for(int i=0;i<640;i++)
					{
					setDAC();
					}
				}
				else if(an_cnt_fast<(BIG_STAT_N))//3.5 ms
				{
					STAT_dt_step=4;
					for(int i=0;i<2560;i++)
					{
					setDAC();
					}
				}////////////////
									
				
				prepareResetDAC(chan);
				setDAC();			
				
				an_cnt++; // upper				
				if(an_cnt>(BIG_STAT_N*5)) an_cnt=0; // lower
				
				an_cnt_fast = an_cnt%BIG_STAT_N;
			}
			else if(event_cnt==4)
			{
				UDR0=STAT_dt_step;
				prepareSetDAC(0, chan);
				setDAC();
			}
			else if(event_cnt==6)//start measure
			{
				prepareSetDAC(ref16, chan);
				setDAC();
				ADCSRA |= (1 << ADSC); 
				
			}
			else if(event_cnt==7)
			{
				prepareSetDAC(0, chan);
				setDAC();
				
				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_;
			}
			else if(event_cnt==8)
			{
				UDR0=ADCH_;
			}
			
		}
		
		
		
		
		ctr=0;
		
		
		if(MD == ONE_SHOT)
		{
			if(event_cnt<16)
				event_cnt++;
		}
		else if(MD == PROGRAM)
		{
			event_cnt++;
			if(event_cnt>8)
				event_cnt = 0;
		}
		else{
			event_cnt++;
			if(event_cnt>T)
				event_cnt=0;
		}
		
		//if(MD!=ONE_SHOT)
		//{
		//	event_cnt++;
		//	if(event_cnt>T)
		//		event_cnt=0;
		//}
		//else
		//{
		//	if(event_cnt<16)
		//		event_cnt++;			
		//}
	}
	ctr++;
}

//прием команд от компьютера по UART в зависимости от режима
ISR(USART_RX_vect)
{
	switch(ptr)
	{
		case 0:
		if(UDR0!=255)//байт 255 является синхронизирующим
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
			proging_val=0;
		}
		break;
		
		
		case 2:		
		x16_simple = UDR0;
		x16 = x16_simple<<4;
		break;
		case 3:	
		ref16=UDR0<<4;
		break;
		case 4:
		if(MD==PROGRAM)
			t1=UDR0;
		else
			reset16=UDR0<<4;
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
		case 8:
		chan=UDR0;
		break;
		
		case 9:
			reverted[chan]=UDR0;
		
			//event_cnt=0;			
			
			if(MD==GATHER_MULT)
			{
			//	PORTD=0b00100000;
			//static int ff=1<<5;
			//if(x16>>4)
			gatherMult();
			//PORTD=(1<<5)^PORTD;
			//PORTD=ff;
			}
			else if(MD==SEPAR_MULT)	
			{
			separMult();	
			}
			if(MD == ONE_SHOT)
			{
				
			}
			
			
			//if(MD!=PROGRAM)
			//	set_reverser(chan, reverted[chan]);
			//else 
			//	set_reverser(chan, 0);
		
		break;
	}
	

	
	dTt2=dT+t2;
	//UDR0=x16/16;
	ptr++;
	ptr%=10;
}
//#NEW PINOUT WITH MULTIPLEXING
//PB2 Problem
// enum chans? (for code reading)


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define MIG 300 		// МиГ25??

#define CHAN_N 17

#define LDAC PD2
#define DDR_SPI DDRB
#define DD_MISO PB4
#define DD_MOSI PB3
#define DD_SCK PB5
#define SPI_SS PB2 	// is it needed?
uint8_t SYNC_PINS[] = {PD5, PD6, PD7};

#define BAUD 9600                                   
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
#define SET_BYTE(port, pos) port|=(1<<pos)
#define reset_BYTE(port, pos) port&=~(1<<pos)
#define DUMMY_BYTE 0



//три возможных режима
typedef enum
{
	CUSTOM,
	VAC,
	PROGRAM,
	GATHER_MULT,
	SEPAR_MULT,
	ONE_SHOT,
	ANALYZE
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

uint8_t STAT_N = 14;
uint8_t STAT_CYCLE = 5;
uint8_t BIG_STAT_N;
uint8_t chan_addrs[8] = {	0,1,2,3 ,  4,5,6,7};  //while for one channel
								//0,1,2,3 ,  4, 5, 6, 7}; 
int16_t VAC16=0, VAC16_H=0, VAC16_HH=0;
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
uint8_t send8;
uint8_t ptr=0, UDP_cnt;// WHAAAAT???
uint8_t PROGRAM_done=0;
uint8_t chan=0;
char c;
uint16_t _adc;
uint16_t an_cnt=0, an_cnt_fast=0;
uint8_t reverted[CHAN_N]={0,0,0,0,0,0,0,0, 
						0,0,0,0,0,0,0,0};
int ctr;

int event_cnt;
int time_step=6;//3
int eventN=100;
int ADC_cnt;
uint8_t ADCH_, ADCL_, ADCH__, ADCL__;

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
	SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(1<<CPOL)|(0<<CPHA);//|(1<<SPR1)|(0<<SPR0);
}

//функция управления ЦАПом 
// при этом, управление регистром LDAC должно использоваться 
//вне функции в перспепктиве создания многоканальной схемы
void prepareSetDAC(int16_t x,uint8_t chan)//_____________bipolar!!! and <<4 larger
{
	x=-x;
	x+=2048;
	PORTD&=~(1<<SYNC_PINS[chan>>3]);
	send8 = (x >> 8);
	send8 &= 0b00001111;
	send8|= (chan_addrs[chan%8]);
	SPI_WriteByte(send8);
	send8=x;
	SPI_WriteByte(send8);		
	PORTD|=(1<<SYNC_PINS[chan>>3]);

}


void prepareResetDAC(int8_t chan)//_____________bipolar!!! and <<4 larger
{
	PORTD&=~(1<<SYNC_PINS[chan>>3]);
	SPI_WriteByte(0b00001000|chan_addrs[chan%8]); // magic numbers, fuck
	SPI_WriteByte(0);		
	PORTD|=(1<<SYNC_PINS[chan>>3]);
}

			
void gatherMult()
{
	reset_BYTE(PORTD, 6);
	reset_BYTE(PORTD, 7);				
	reset_BYTE(PORTD, 5);
	reset_BYTE(PORTC, 4);
	
	
	SET_BYTE(PORTC, 1);				
	SET_BYTE(PORTB, 2);
	SET_BYTE(PORTB, 1);
	SET_BYTE(PORTB, 0);
}
void separMult()
{
	reset_BYTE(PORTB, 1);
	reset_BYTE(PORTB, 2);
	reset_BYTE(PORTC, 1);
	
	SET_BYTE(PORTC, 4);						
	SET_BYTE(PORTD, 6);
	SET_BYTE(PORTD, 7);
	SET_BYTE(PORTB, 0);
	SET_BYTE(PORTD, 5);
}



void SPI_WriteByte(uint8_t data)
{
   SPDR = data;
  while(!(SPSR & (1<<SPIF)));
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

void setDAC(){
	PORTD&=~(1<<LDAC);
	PORTD|=(1<<LDAC);	
}


void main(void)
{
	BIG_STAT_N = STAT_N*STAT_CYCLE;
	
	for (uint8_t i=0; i< 8;i++)
		chan_addrs[i]=chan_addrs[i]<<4;
	
	PORTC|=0b00000000;
	DDRC= 0b00011110;
	DDRD =0b11111111;
	//PORTD|=0b00100000;	
	DDRB= 0b00011111;
	sei();
	SPI_MasterInit();
	timer_init();
    //DDRD = 0b000001100;	
	
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

		//_delay_ms(1000);
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
								
					//VAC16_HH=VAC16_H;
					VAC16_H=VAC16;
					
					if(pos_phase)
					{
								//PORTC=0b00000010;
			//PORTB=0b00011111;
			//PORTD=0b11101100;
						VAC16+=32;
						if(VAC16>(ref16-1))
						{
						pos_phase=0;				
						}
					}
					else
					{
						//PORTB=0;
			//PORTC=0;
			//PORTD=0;
						VAC16-=32;
						if(VAC16<(-x16+1))
						{
						pos_phase=1;									
						}
					}	
					
					
					
					UDR0=VAC16>>4;
					prepareSetDAC(VAC16,chan);
					//prepareSetDAC(VAC16,1);
					//prepareSetDAC(VAC16,2);
					//prepareSetDAC(VAC16,3);
					//prepareSetDAC(VAC16,4);
					//prepareSetDAC(VAC16,5);
					//prepareSetDAC(VAC16,6);
					//prepareSetDAC(VAC16,7);				
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
			
			//DACset proging val
			if(event_cnt==0)
			{
				UDR0=255;
				
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
			else 
			if(event_cnt==(1))
			{	
				prepareSetDAC(0,chan);
				setDAC();
				UDR0=PROGRAM_done;
			}else
			if(event_cnt==(2))//ADC GET 
			{	

				ADCL_=ADCL;	
				ADCH_=ADCH;
				UDR0=ADCL_;
			}
			
			//ADC make CONTINUE
			//make decision
			else
			if(event_cnt==(3))
			{	
				
				UDR0=ADCH_;
				
				_adc=(ADCL_|(ADCH_ <<8));
				adc_h=((uint16_t)(512)-_adc);
				
				//optimization
				//put some window val here maybe?
				//be  carefull with -
				if((adc_h)<(uint16_t)(t1-1))
				{
					proging_val = -x16;  //set!
				}
				else if((adc_h)<(uint16_t)(t1+2)) //done!
				{
					PROGRAM_done=1;
					proging_val=0;
					prepareSetDAC(ref16,chan);
					setDAC();
				}
			}
			//else		
			//if(event_cnt==7)//t1
			//{
				//prepareSetDAC(0,chan);
				//setDAC();
			//}
			else if(event_cnt==5)//dT
			{		
				prepareSetDAC(ref16,chan);
				setDAC();
			}
			else if(event_cnt==(5+1))
			{
				ADCSRA |= (1 << ADSC); 
			}		
			else if(event_cnt==7)//
			{						
				prepareSetDAC(0,chan);
				setDAC();
			}
		}
		else if(MD == ONE_SHOT)
		{
			//готовим reset
				if(event_cnt==1){
				separMult();
			}//resetting
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
			if(event_cnt>9)
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
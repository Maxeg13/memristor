#include<util/delay.h>
#include<avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/iom2560.h>
#include <avr/iomxx0_1.h>//atmega2560 and so on


#define PORTS_N 6
#define byte uint8_t
#define BAUD 38400                                   // define baud
#define BAUDRATE ((F_CPU)/(BAUD*2*16UL)-1)//16UL
char c;
//int

byte pwm=20, pwm_h;
int rec;
int i;
float s=0;
byte cnt_N=40;
int cnt=0;
byte state_med[PORTS_N],state[PORTS_N],state_PWM;
byte state_rec[8];
byte key1='a', key2='c';
byte  port_ind=0;
int ptr=0;

void portIncrement()
{
    ptr++;
    if(ptr==8)
    {
        ptr=0;
        port_ind++;
        if(port_ind==PORTS_N)
            port_ind-=1;
    }
}

void uart_init (unsigned int ubrr)
{
    //    UCSR0A=0x00;

    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;



    UCSR0A|=(1<<UDRE0);
    UCSR0B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<USBS1)|(3<<UCSZ10);//UCSR1C = (1<<USBS1)|(3<<UCSZ10);
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

void main(void)
{

    uart_init(BAUDRATE);
    timer_init();
    sei();


    //    _SFR_MEM8(0x104)=0xFF;
    DDRA=0xFF;
    DDRC=0xFF;
    DDRD=0xFF;
    DDRL=0xFF;
    DDRH=0xFF;
    DDRK=0xFF;
            //        DDRH=0xFF;
    //    PORTB=0xFF;

    for(;;)
    {

        //        PORTL=0xFF;
        //        _delay_ms(1000);
        //        while ((UCSR0A & (1<<UDRE0)));
        //        UCSR0A &=~ (1<<RXC0);
        //        PORTH=~PORTH;


    }

}

ISR(TIMER2_OVF_vect)
{

    s+=0.001;
    if(cnt==cnt_N)
    {
        cnt=0;
        PORTA=state[0];
        PORTC=state[1];//?
        PORTD=state[2];
        PORTL=state[3];
        PORTH=state[4];
        PORTK=state[5];
        //            PORTL=0xFF;
    }
    //        else if(cnt>((1+sin(s))*8+4))/////////
    else if(cnt>pwm)
    {
        PORTA=0;
        PORTC=0;
        PORTD=0;
        PORTL=0;
        PORTH=0;
        PORTK=0;
    }
    cnt++;
}

ISR(USART0_RX_vect)
{
    rec=UDR0;
    if(ptr!=-1)
    {
        if(rec==key1)
        {
            ptr=-1;
            port_ind=0;
            for(byte i=0;i<PORTS_N;i++)
                state_med[i]=0x00;
        }
        else if(rec=='s')
        {
            state_med[port_ind]|=(1<<ptr);
            portIncrement();
        }
        else if(rec=='d')
        {
            state_med[port_ind]&=~(1<<ptr);
            portIncrement();
        }
        else if(rec==key2)
        {
            for(byte i=0;i<PORTS_N;i++)
                state[i]=state_med[i];
            pwm=pwm_h;
        }
    }
    else
    {
        pwm_h=rec;
        portIncrement();
    }

    //    PORTL=~PORTL;
    //    c=UDR1;
    //    while(!(UCSR1A & (1<<UDRE1)));
    //    UDR1=c;
    //    UCSR0A;

    //   UDR0=rec;
    //    for(long h=0;h<100000;h++)
    //        i++;
    //    UCSR0A|=(1<<UDRE0);

}

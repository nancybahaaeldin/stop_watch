

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

unsigned char clock_second = 0;
unsigned char clock_minute = 0;
unsigned char clock_hour = 0;


void TIMER1_Init(void)
{
	TCCR1A=(1<<FOC1A) | (1<<FOC1B) ;
	TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10);
	TCNT1 = 0;
	OCR1A = 1000;
	SREG=(1<<7);
	TIMSK |= (1<<OCIE1A);

}

ISR (TIMER1_COMPA_vect)
{
	                clock_second++;
			         if(clock_second==60)
			         {
			            clock_second=0;
			            clock_minute++;
			         }
			      if(clock_minute==60)
			      {
			         clock_minute=0;
			         clock_hour++;
			      }
			      if(clock_hour==24)
			      {
			    	  clock_hour=0;

			      }

}

void INT0_Init_reset()
{
	SREG  &= ~(1<<7);
	DDRD  &= (~(1<<PD2));
	PORTD |= (1<<PD2);
	MCUCR |=  (1<<ISC01);
	GICR  |= (1<<INT0);
	SREG  |= (1<<7);
}
ISR(INT0_vect)
{
	 clock_hour=0;
	 clock_minute=0;
	 clock_second=0;
}

void INT1_Init_pause()
{
	SREG  &= ~(1<<7);
	DDRD  &= (~(1<<PD3));
	PORTD |= (1<<PD3);
	MCUCR |=  (1<<ISC10) | (1<<ISC11);
	GICR  |= (1<<INT1);
	SREG  |= (1<<7);
}
ISR(INT1_vect)
{
	TCCR1B &=~(1<<CS12)|(1<<CS11)|(1<<CS10);
}
void INT2_Init_resume()
{
	SREG  &= ~(1<<7);
	DDRB  &= (~(1<<PB2));
	PORTB |= (1<<PB2);
	MCUCR &=~(1<<ISC2);
	GICR  |= (1<<INT2) ;
	SREG  |= (1<<7);
}
ISR(INT2_vect)
{
	TCCR1B =(1<<CS12)|(1<<CS11)|(1<<CS10);
}

int main ()
{

	DDRC |= 0x0F;
	PORTC &= 0xF0;
	DDRA |= 0x3F;
	PORTA |=0x3F;

	 TIMER1_Init();
	 INT0_Init_reset();
	 INT1_Init_pause();
	 INT2_Init_resume();




	while(1)
	{


			        PORTA=(1<<5);
					PORTC=clock_second % 10;
					_delay_ms(5);
					PORTA=(1<<4);
					PORTC=clock_second / 10;
					_delay_ms(5);
					PORTA=(1<<3);
					PORTC=clock_minute % 10;
					_delay_ms(5);
					PORTA=(1<<2);
					PORTC=clock_minute / 10;
					_delay_ms(5);
					PORTA=(1<<1);
					PORTC=clock_hour % 10;
					_delay_ms(5);
					PORTA=(1<<0);
					PORTC=clock_hour / 10;
					_delay_ms(5);





	}

	return 0;
}







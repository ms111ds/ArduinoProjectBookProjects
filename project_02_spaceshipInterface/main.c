/*
 * ArduinoProject2.c
 *
 * Created: 7/26/2019 3:33:45 PM
 * Author : ms111ds 
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>


int main(void)
{
	// open ports D3, D4, and D5 for writing. The rest are for reading.
	DDRD &= 0b00000000;	
	DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD5);
	
	
    /* Replace with your application code */
    while (1) 
    {
		// flash ports 4 and 5 and turn off port 3 if switch is on
		if( PIND & (1 << PIND2) )
		{
			PORTD &= ~ (1 << PORTD3); // port 3 off
			PORTD |= (1 << PORTD5); // port 5 on
			_delay_ms(250);
			PORTD &= ~ (1 << PORTD5); // port 5 off
			PORTD |= (1 << PORTD4); // port 4 on
			_delay_ms(250);
			PORTD &= ~ (1 << PORTD4); // port 4 off
		}
		// turn off red ports 4 and 5 and turn on port 3
		else
		{
			PORTD &= ~ ( (1 << PORTD4) | (1 << PORTD5) ); //ports 4 and 5 off
			PORTD |= (1 << PORTD3); // port 3 on
		}
    }
}


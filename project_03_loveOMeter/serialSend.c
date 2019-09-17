/*
 * serialSend.c
 *
 * Created: August 1, 2019
 * Author : Diego
 * Actually, much of this was copied from: https://efundies.com/avr-usart-serial/
 * and https://appelsiini.net/2011/simple-usart-with-avr-libc/
 */

#define F_CPU 16000000
#define BAUD 9600

#include <avr/io.h>
#include <util/setbaud.h>
#include "serialSend.h"


/**************************************************************************
 * usart_init
 * description: initialize USART (universal synchronous and asynchronous
 *		serial receiver and transmitter). 
 **************************************************************************/
void usart_init()
{
	// set usbrr (USART baud rate register) to set UART speed
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
	// determine if 2X speed in needed
	if( USE_2X )
	{
		UCSR0A = (1 << U2X0 );
	}
	
	// enable the reciever and transmitter
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);
	
	// frame to 8 a length of 8 bits (also 1 stop bit and no parity bits which are not shown)
	UCSR0C = (1 << UCSZ01) | (1 <<UCSZ00);
	 
}

/**************************************************************************
 * serialSendChar
 * Description: sends a single character over an serial connection
 * Requirements: An already initialized serial connection must be created
 *************************************************************************/
void serialSendChar( char c )
{
	// loop while transmission not ready to be sent
	while ( !(UCSR0A & (1 << UDRE0 )) ) {}
	// load char in USARD I/O data register to send
	UDR0 = c;
}


/*
 * ArduinoProject3PotentiometerVersion.c
 *
 * Created: 7/28/2019 11:04:37 PM
 * Author : ms111ds 
 */ 

#define F_CPU 16000000UL
#define BAUD 9600

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include "./serialSend.h"

/***************************************************************************************************
 * adcInit
 * description: initializes the analog to digital converter
 ***************************************************************************************************/
void adcInit()
{
	// Set reference voltage (max voltage) equal to the 5V digital supply
	ADMUX = ( 1 << REFS0 );
	
	// select ADC0 as input channel (not really necessary to do since these values start as zero)
	ADMUX &= ~( ( 0 << MUX2 ) | ( 0 << MUX1 ) | ( 0 << MUX0 ) );
		
	// Analog to Digital Converter (ADC) operates on a frequency between 50KHz and 200KHz need to apply
	// a prescaler to divide the 16MHz CPU frequency to be within this range. The setting below applies
	// a prescaler of 128 to get the ADC frequency to 16MHz / 128 =  125KHz.
	ADCSRA = ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 );
	
	// turn on auto triggering so that the analog input continuously samples
	ADCSRA |= ( 1 << ADATE );
	ADCSRB &= ~( ( 1 << ADTS2 ) | ( 1 << ADTS1 ) | ( 1 << ADTS0 ) );
	
	// Enable Analog to digital conversion
	ADCSRA |= ( 1 << ADEN );

	// Disable digital input for PC0 ( ADC0 )
	DIDR0 |= ( 1 << ADC0D );
}


int main(void)
{
    /* Replace with your application code */
	float voltage;
	char stringVoltage[4];
	int i;
	
	// Analog pin settings
	adcInit();
	
	// start serial transmitter
	usart_init();
	
	// Digital pin settings
	DDRD |= ( 1 << DDD2 ) | ( 1 << DDD3 ) | ( 1 << DDD4 ); // make digital ports output ports

	
    while (1) 
    {
		// start reading voltage	
		ADCSRA |= ( 1 << ADSC );
		
		// get voltage between 0V and 5V
		// weird formula for characters is to ensure result of all operations fit inside 16 bits
		voltage = (float)ADC * 5.0 / 1024.0;
		stringVoltage[0] = (char)( (int)voltage + 0x30);
		stringVoltage[1] = '.';
		stringVoltage[2] = (char)( (int)( ( voltage * 10.0 ) - (float)( (int)voltage * 10 ) ) + 0x30 );
		stringVoltage[3] = (char)( (int)( ( voltage * 100.0 ) - (float)( (int)( voltage * 10.0 ) * 10 ) ) + 0x30 );
		
		// turn lights on depending on the voltage
		if ( voltage > 3.75 )
		{
			PORTD |= ( 1 << PORTD2 ) | ( 1 << PORTD3 ) | ( 1 << PORTD4 ); // ports d2, d3, and d4 on	
		
		}
		else if ( voltage > 2.5 )
		{
			PORTD |= ( 1 << PORTD2 ) | ( 1 << PORTD3 ); // ports d2 and d3 on
			PORTD &= ~( 1 << PORTD4 );					// port d4 off
		}
		else if ( voltage > 1.25 )
		{
			PORTD |= ( 1 << PORTD2 );							// port d2 on
			PORTD &= ~( ( 1 << PORTD3 ) | ( 1 << PORTD4 ) );	// ports d3 and d4 off
		}
		else
		{
			PORTD &= ~ ( ( 1 << PORTD2 ) | ( 1 << PORTD3 ) | ( 1 << PORTD4 ) ); // ports d2, d3, and d4 off
		}
		
		// send voltage readings to console
		for( i = 0; i < 4; i++)
		{
			serialSendChar(stringVoltage[i]);
		}
		serialSendChar('\n');
		_delay_ms(250);	
    }
}

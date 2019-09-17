/*
 * ArduinoProject5_Servo.c
 *
 * Created: 8/26/2019 10:43:58 PM
 * Author : ms111ds 
 * used: https://www.servocity.com/how-does-a-servo-work
 * serial send code copied from https://www.youtube.com/watch?v=3_omxGIL0kw
 */ 
#define F_CPU	16000000UL
#define BAUD	9600
#define BRC		((F_CPU/BAUD/16)-1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

								// note servo neutral position at about 1500 us (micro second) of a 20000 us period
								// our servo only has motion of 180 degrees, 0 degrees at 1000us and 180 degrees at 2000us
#define MIN_PULSE_WIDTH 450	// microsecond pulse width of a 20000 us period, also try 1000/ 544 / 450 (450 found by testing)
#define MAX_PULSE_WIDTH 2400	// microsecond pulse width of a 20000 us period, also try 2000/ 2400*, > MIN_PULSE_WIDTH
#define REFRESH_PERIOD  20000	// total refresh period
#define PRESCALER		64		// prescaler we will use
#define BUFSIZE			64		// size of serial buffer, must be less than 256

// clock speed * refresh period = 16 ticks/us * 20,000 us = 320,000 ticks
// min pulse width				= 16 ticks/us * 1,000 us  =  16,000 ticks
// max pulse width				= 16 ticks/us * 2,000 us  =  32,000 ticks
// if we use a prescaler of 64...
// refresh period (prescaler = 64)	= 5,000 ticks
// min pulse width (prescaler = 64) =   250 ticks
// max pulse width (prescaler = 64) =   500 ticks
// 500 - 250 = 250, which is a better resolution than 180 (for each degree) so we will use the prescaler of 64

uint16_t rpTicks		= F_CPU / 1000000 * REFRESH_PERIOD  / PRESCALER;
uint16_t minpwTicks		= F_CPU / 1000000 * MIN_PULSE_WIDTH / PRESCALER;
uint16_t maxpwTicks		= F_CPU / 1000000 * MAX_PULSE_WIDTH / PRESCALER;
uint32_t sampleTicks    = F_CPU / 1000000 * MIN_PULSE_WIDTH / PRESCALER;	// ticks calculated from latest sample reading
uint32_t curPulseTicks	= F_CPU / 1000000 * MIN_PULSE_WIDTH / PRESCALER;	// current ticks used for wave we are on.
uint8_t  onPulse		= 0;												// CTC pulse used starts low
char	 serialBuf[BUFSIZE];												// serial buffer
uint8_t  bufReadPos		= 0;												// position for next character in serial buffer to be sent
uint8_t  bufWritePos	= 0;												// position for next character to be added to serial buffer


// function prototypes
int8_t	initAnalogRead(uint8_t);
void	startADC();
void	initPin9PWM();
void	PWMStart();
void	serialBufWrite(char);
void	sendString(char *, uint8_t);
void	sendInt(int);

int main(void)
{
	// start USART
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;
	UCSR0B = (1 << TXCIE0) | (1 << TXEN0);	// start transmitter and transmitter interrupts
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);	// asynchronous, no parity bits, 1 stop bit, 8 bit data
	
	// start analog and PWM
    if (initAnalogRead(0) == -1)
	{
		return -1;	
	}
    initPin9PWM();
	
	sei();
	startADC();
    PWMStart();
	
    while (1) 
    {
		_delay_ms(1000);
		sendInt(sampleTicks);
		sendString("- - - - -",9);
		sendInt(ADC);
		
    }
}

/********************************************************************************************************
 * initAnalogRead
 * Description: initialize analog pin by setting up required registers. Sets up for single conversion.
 *******************************************************************************************************/
int8_t initAnalogRead(uint8_t pinNum)
{
	if (pinNum > 5)
	{
		return -1;
	}
	ADMUX	= (1 << REFS0) | pinNum;						// set reference voltage to Vcc (5V) and enable the pin
	ADCSRA	= (1 << ADEN)  | (1 << ADIE) |					// start ADC, enable ADC interrupts
			  (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// set ADC prescaler to 128
	DIDR0  =  (1 << pinNum);								// disable digital input on selected pin
	
	return 0;
}

/********************************************************************************************************
 * startADC
 * Description: Starts single conversion.
 *******************************************************************************************************/
void startADC()
{
	ADCSRA |= (1 << ADSC);
}

/********************************************************************************************************
 * initPin9PWM
 * Description: Initialize settings to have pin 9 output pulse.
 *******************************************************************************************************/
void initPin9PWM()
{
	DDRB   = (1 << DDB1);									// set pin 9 to output
	TCCR1A = 0x00;											// will turn on pin 9 manually on compare match
	TCCR1B = (1 << WGM12);									// enable CTC (clear counter on compare)
	OCR1A  = 0;												// set the initial number of ticks, PWM starts low 
	TIMSK1 = (1 << ICIE1)  | (1 << OCIE1A);					// set interrupts for compare match
}

/********************************************************************************************************
 * PWMStart
 * Description: Start the pulse width modulation with the prescaler set in the PRESCALER macro.
 *******************************************************************************************************/
void PWMStart()
{
	switch(PRESCALER)
	{
		case 1:
			TCCR1B |= 0x01;
			break;
		case 8:
			TCCR1B |= 0x02;
			break;
		case 64:
			TCCR1B |= 0x03;
			break;
		case 256:
			TCCR1B |= 0x04;
			break;
		case 1024:
			TCCR1B |= 0x05;
			break;
		default:
			TCCR1B |= 0x00;
			break;
	}
}


/********************************************************************************************************
 * serialBufWrite
 * Description: place a character into the serial buffer to wait to be sent. It will be sent across the
 * serial cable when its turn comes around.
 *******************************************************************************************************/
void serialBufWrite(char mychar)
{
	serialBuf[bufWritePos] = mychar;
	bufWritePos = (bufWritePos + 1) % BUFSIZE;
}

/********************************************************************************************************
 * sendString
 * Description: place an entire string into the serial buffer. The number of characters in the string
 *		must be provided.
 *******************************************************************************************************/
void sendString(char * stringPtr, uint8_t strSize)
{
	uint8_t i;
	
	for( i = 0; i < strSize; i++)
	{
		serialBufWrite(*(stringPtr + i * sizeof(char)));
	}
	serialBufWrite('\n');
	
	// start sending information across serial line
	if (UCSR0A & (1 << UDRE0))
	{
		UDR0 = '>';
	}
}

/********************************************************************************************************
 * sendInt
 * Description: send a 16-bit integer across the serial buffer.
 *******************************************************************************************************/
void sendInt(int myNum)
{
	char	numString[6];
	int		curNum = myNum;
	uint8_t i = 0;
	
	if (myNum < 0)
	{
		numString[0] = '-';
	}
	else
	{
		numString[0] = ' ';
	}
	
	for (i = 0 ; i < 5; i ++)
	{
		numString[5 - i] = curNum % 10 + 0x30;
		curNum /= 10;
	}
	
	sendString(numString, 6);
}

// Analog to Digital Conversion complete interrupt
ISR(ADC_vect)
{
	// output reading (in ADCH and ADCL) a fraction of 1024. Convert to value between
	// minpwTicks and maxpwTicks
	// need to typecast a 16-bit number to long 32-bit number as compiler 16-bit multiplication is not good
	sampleTicks = (long)ADC * (maxpwTicks - minpwTicks) / 1024 + minpwTicks;
	
	startADC();	// get a new reading
}

// Timer comparison achieved interrupt
ISR(TIMER1_COMPA_vect)
{
	if(onPulse == 0)
	{
		PORTB |= (1 << PORTB1);				// turn on pin 9
		curPulseTicks = sampleTicks;		// set the number of ticks for the current pulse equal to the ticks
											// calculated from the latest analog sample

		OCR1AH  = (uint8_t)(curPulseTicks >> 8);
		OCR1AL  = (uint8_t)(curPulseTicks);
	}
	else
	{
		PORTB &= ~(1 << PORTB1);			// turn off pin 9
		OCR1AH  = (uint8_t)(( rpTicks - curPulseTicks ) >> 8);
		OCR1AL  = (uint8_t)( rpTicks - curPulseTicks );
	}
	
	onPulse = !onPulse;				// switch state
}

// Serial transmission complete interrupt
ISR(USART_TX_vect)
{
	// only send next character in buffer if there is something to send
	if (bufReadPos != bufWritePos)
	{
		UDR0 = serialBuf[bufReadPos];
		bufReadPos = (bufReadPos + 1) % BUFSIZE;
	}
}



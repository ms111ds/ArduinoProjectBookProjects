/*
 * ArduinoProject7_hourglass.c
 *
 * Created: 9/11/2019 11:03:44 PM
 * Author : ms111ds 
 * external interrupt help from https://www.youtube.com/watch?v=aT1tU0EnSHw
 */
 
 // CPU macros
#define  F_CPU	 16000000UL

// Serial Transmitter Macros
#define  BAUD	 9600
#define  BRC	 (F_CPU/BAUD/16-1)
#define  BUFSIZE 128

// Pin Macros
#define  ON		 1
#define  OFF     0
#define  INPUT   0
#define  OUTPUT  1
#define  LED_PORT 'D'
#define  LED_PIN_1 PIND7
#define  LED_PIN_2 PIND6
#define  LED_PIN_3 PIND5
#define  LED_PIN_4 PIND4
#define  LED_PIN_5 PIND3
#define  LED_PIN_6 PIND2
#define  SWITCH_PORT 'B'
#define  SWITCH_PIN PINB0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdarg.h>

/***************************************************************************************
 * Global Variables
 **************************************************************************************/
// buffer variables
char	serialBuf[BUFSIZE];
uint8_t bufInsertIndex = 0;
uint8_t bufSendIndex = 0;

//timer variables
uint8_t waitSec = 12;
volatile uint8_t curSec  = 0;
int milliSeconds = 0;

// interrupt variables
volatile uint8_t extIntFlag = ON;

/***************************************************************************************
 * Function Prototypes
 **************************************************************************************/
// buffer functions
void bufLoadChar(char);
void serialSendString(char *);
void serialSendInt(int);

// timer functions
void milliTimerSetup();
void milliTimerOn();
void milliTimerOff();

// utility functions
uint8_t getState();

// pin functions
void setPinDir(char, uint8_t , int, ...);
void setPinVal(char, uint8_t , int, ...);

int main(void)
{
	uint8_t curState = getState();
	uint8_t timerMaxedFlag = OFF;
	
    // initialize serial transmitter
	UCSR0B = (1 << TXCIE0) | (1 << TXEN0);	// enable transmitter and transmit complete interrupt
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // asynchronous, no parity bits, 1 stop bit, 8-bit data
	UBRR0H = (BRC >> 8);					// apply baud rate high byte
	UBRR0L = BRC;							// apply baud rate low byte
	
	// initialize pins
	setPinDir(LED_PORT, OUTPUT, 6,	LED_PIN_1, 
									LED_PIN_2, 
									LED_PIN_3, 
									LED_PIN_4, 
									LED_PIN_5, 
									LED_PIN_6);
	setPinDir(SWITCH_PORT, INPUT, 1, SWITCH_PIN); 
	
	// initialize external interrupts for tilt switch pin 8
	PCICR  = (1 << PCIE0);
	PCMSK0 = (1 << PCINT0);
	
	// initialize timer
	milliTimerSetup();

	
	// set interrupts and start timer
	sei();	// enable global interrupts
	if (curState == ON)
		milliTimerOn();
	
    while (1) 
    {
		if (extIntFlag == OFF)
		{
			curState = getState();
			
			if (curState == ON)
			{
				milliTimerOn();
			}
			else
			{
				curSec = 0;
				milliTimerOff();
				timerMaxedFlag = OFF;
				setPinVal('D', OFF, 6, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5, LED_PIN_6);
			}
			
			// reinitialize external interrupt
			extIntFlag = ON;
			PCMSK0 = (1 << PCINT0);
		}
		
		if(curState == ON)
		{
			if (curSec >= waitSec)
			{
				if (timerMaxedFlag == OFF)
				{
					setPinVal('D', ON, 6, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5, LED_PIN_6);				
					timerMaxedFlag = ON;
					milliTimerOff();
					_delay_ms(500);
				}
				else
				{
					setPinVal('D', OFF, 6, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5, LED_PIN_6);
					_delay_ms(500);
					setPinVal('D', ON, 6, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5, LED_PIN_6);
					_delay_ms(500);
				}

			}
			else if (curSec >= waitSec / 6 * 5)
				setPinVal('D', ON, 5, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5);
			else if (curSec >= waitSec / 6 * 4)
				setPinVal('D', ON, 4, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4);
			else if (curSec >= waitSec / 6 * 3)
				setPinVal('D', ON, 3, LED_PIN_1, LED_PIN_2, LED_PIN_3);	
			else if (curSec >= waitSec / 6 * 2)
				setPinVal('D', ON, 2, LED_PIN_1, LED_PIN_2);
			else if (curSec >= waitSec / 6 * 1)
				setPinVal('D', ON, 1, LED_PIN_1);	
			else
				setPinVal('D', OFF, 6, LED_PIN_1, LED_PIN_2, LED_PIN_3, LED_PIN_4, LED_PIN_5, LED_PIN_6);
		}
    }
}

/***************************************************************************************
 * bufLoadChar
 * Description: adds character to serial buffer
 **************************************************************************************/
void bufLoadChar(char myChar)
{
	uint8_t bufNextInsertIndex = (bufInsertIndex + 1) % BUFSIZE;
	
	if (bufNextInsertIndex != bufSendIndex)
	{
		serialBuf[bufInsertIndex] = myChar;
		bufInsertIndex = bufNextInsertIndex;
	}
}

/***************************************************************************************
 * serialSendString
 * Description: adds a null terminated string to the serial buffer and initiates the
 *		sending of the buffer contents across the serial line
 **************************************************************************************/
void serialSendString(char * myString)
{
	char * 	curCharPtr = myString;
	
	while(*curCharPtr != '\0')
	{
		bufLoadChar(*curCharPtr);
		curCharPtr += sizeof(char);
	}
	bufLoadChar('\n');
	
	if ( (UCSR0A & (1 << UDRE0)) && (bufSendIndex != bufInsertIndex) )
	{
		UDR0 = serialBuf[bufSendIndex];
		bufSendIndex = (bufSendIndex + 1) % BUFSIZE;
	}
}

/***************************************************************************************
 * serialSendInt
 * Description: adds an integer to the serial buffer serial buffer and initiates the
 *		sending of the buffer contents across the serial line
 **************************************************************************************/
void serialSendInt(int myInt)
{
	char	intString[7];
	int		curInt = myInt;
	int		sign;
	uint8_t i;
	
	if (myInt < 0)
	{
		intString[0] = '-';
		sign = -1;
	}
	else
	{
		intString[0] = ' ';
		sign = 1;
	}
		
	for (i = 5; i > 0; i--)
	{
		intString[i] = sign * (curInt % 10) + 0x30;
		curInt /= 10;
	}
	
	intString[6] = '\0';
	
	serialSendString(intString);
}

/***************************************************************************************
 * getState
 * Description: bounce handling algorithm to determine the true state
 **************************************************************************************/
uint8_t getState()
{
	uint8_t lowBits = 0xAA;
	uint8_t highBits = 0xAA;
	
	// protect from switch bounce (all 16 bits need to be all 1s or all 0s)
	while( highBits != lowBits || ((highBits != 0x00) && (highBits != 0xFF)) )
	{
		highBits = (highBits << 1) | (lowBits >> 7);
		if (PINB & (1 << SWITCH_PIN))
			lowBits = (lowBits << 1) | 1;
		else
			lowBits = (lowBits << 1);
			
		_delay_ms(1);
	}
	
	if (highBits == 0x00)
		return OFF;
	else
		return ON;	
}

/***************************************************************************************
 * milliTimerSetup
 * Description: set up timer 0 to count milliseconds
 **************************************************************************************/
void milliTimerSetup()
{
	TCCR0A = (1 << WGM01); // enable CTC mode
	OCR0A  = 249;          // 1000 Hz = 16000000 Hz / (64 * (249 + 1))
	TIMSK0 = (1 << OCIE0A); //turn on interrupt on compare match with OCR0A	
}

/***************************************************************************************
 * milliTimerOn
 * Description: set prescalar on timer 0 to 64, the required prescalar to get milliseconds
 *		and start timer
 **************************************************************************************/
void milliTimerOn()
{
	TCCR0B |= (1 << CS01) | (1 << CS00);	// set timer prescalar to 64 and start timer
}

/***************************************************************************************
 * milliTimerOff
 * Description: stop timer 0 and reset the counted milliseconds to 0
 **************************************************************************************/
void milliTimerOff()
{
	TCCR0B &= ~((1 << CS01) | (1 << CS00)); // stop clock
	TCNT0 = 0x00;							// reset timer
}

/***************************************************************************************
 * setPinDir
 * Description: Set the pin direction to either INPUT or OUTPUT. Must select a port
 *		and the desired pins in that port.
 **************************************************************************************/
void setPinDir(char port, uint8_t pinDir, int argCount, ...)
{
	va_list argPointer;
	int i;
	uint8_t dirResult = 0x00;
	
	va_start(argPointer, argCount);
	for (i = 0; i < argCount; i++)
	{
		dirResult |= (1 << va_arg(argPointer, int));
	}
	
	va_end(argPointer);
	
	if (pinDir == INPUT)
	{
		switch(port)
		{
			case 'B':
				DDRB &= ~dirResult;
				break;
			case 'C':
				DDRC &= ~dirResult;
				break;
			case 'D':
				DDRD &= ~dirResult;
				break;			
			default:
				break;
		}
	}
	else if (pinDir == OUTPUT)
	{
		switch(port)
		{
			case 'B':
				DDRB |= dirResult;
				break;
			case 'C':
				DDRC |= dirResult;
				break;
			case 'D':
				DDRD |= dirResult;
				break;
			default:
				break;
		}		
	}
}

/***************************************************************************************
 * setPinVal
 * Description: Set the pin vlue to either ON or OFF. Must select a port
 *		and the desired pins in that port.
 **************************************************************************************/
void setPinVal(char port, uint8_t pinVal, int argCount, ...)
{
	va_list argPointer;
	int i;
	uint8_t valResult = 0x00;
	
	va_start(argPointer, argCount);
	for (i = 0; i < argCount; i++)
	{
		valResult |= (1 << va_arg(argPointer, int));
	}

	va_end(argPointer);
	
	if (pinVal == OFF)
	{
		switch(port)
		{
			case 'B':
				PORTB &= ~valResult;
				break;
			case 'C':
				PORTC &= ~valResult;
				break;
			case 'D':
				PORTD &= ~valResult;
				break;
			default:
				break;
		}
	}
	else if (pinVal == ON)
	{
		switch(port)
		{
			case 'B':
				PORTB |= valResult;
				break;
			case 'C':
				PORTC |= valResult;
				break;
			case 'D':
				PORTD |= valResult;
				break;
			default:
				break;
		}
	}
}

/***************************************************************************************
 * Interrupt Service Routines
 **************************************************************************************/

// Serial Transmission Complete: Sends another character in buffer
ISR(USART_TX_vect)
{
	if ( bufSendIndex != bufInsertIndex )
	{
		UDR0 = serialBuf[bufSendIndex];
		bufSendIndex = (bufSendIndex + 1) % BUFSIZE;
	}
}

// PCINT0 signal change detected: turn off the interrupt and disable external interrupt flag
ISR(PCINT0_vect)
{
	PCMSK0 = 0x00;
	extIntFlag = OFF;
}

// Timer0 compar match: increment the number of milliseconds that have passed, for every 1000
//		milliseconds, increase the number of seconds.
ISR(TIMER0_COMPA_vect)
{
	milliSeconds = (milliSeconds + 1) % 1000;
	
	if (milliSeconds == 0)
	{
		curSec++;
	}
}

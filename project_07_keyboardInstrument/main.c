/*
 * ArduinoProject7_keyboard.c
 *
 * Created: 9/5/2019 11:41:27 PM
 * Author : ms111ds 
 * serial send code copied from https://www.youtube.com/watch?v=3_omxGIL0kw
 */ 

#define F_CPU	16000000UL
#define BAUD	9600
#define BRC		(F_CPU/16/BAUD-1)
#define BUFSIZE 128
#define TIMERCOMPVAL(freq, prescaler) (F_CPU/(freq)/(prescaler)/2-1)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// serial buffer global variables
char serialBuf[BUFSIZE];
uint8_t insertIndex	= 0;
uint8_t sendIndex	= 0;

 // frequencies in Hertz
unsigned int notes[4] = {262, 294, 330, 349};

// global values
volatile unsigned int curAnalogVal	  = 0;
volatile unsigned int curFreq		  = 0;
volatile unsigned int curTimerCompVal = 0;

// serial functions
void loadSerialBuf(char);
void serialSendNext();
void serialSendString(char *);
void serialSendInt(int);

// ADC functions
void startAnalogConversion();

int main(void)
{
    // USART initialization
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // asynchronous mode, no parity bits, 1 stop bit, 8 bit frame data
	UCSR0B = (1 << TXCIE0) | (1 << TXEN0);	// enable transmitter and transmitter interrupts
	
	// analog pin 0 (PC0) initialization
	ADMUX	= (1 << REFS0);					// 5V reference, connect analog pin 0 to ADC
	ADCSRA	= (1 << ADEN) | (1 << ADIE);	// enable ADC and ADC interrupts
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC prescaler at 128 
	DIDR0	= (1 << ADC0D);					// disable digital input of analog port 0
	
	// digital pin 8 (PB0) initialization
	DDRB	= (1 << DDB0);
	
	// timer 1 initialization
	TCCR1B = (1 << WGM12);						// enable CTC mode
	OCR1AH = 0;
	OCR1AL = 0xFF;
	TIMSK1 = (1 << OCIE1A);		// enable interrupts on compare output A
	
	sei();
	startAnalogConversion();
	
	// set prescaler to turn on timer
	TCCR1B |= (1 << CS11);								  // timer prescaler of 8
	
    while (1) 
    {
		if (curAnalogVal > 1010)
		{
			curFreq = notes[0];
		}
		else if (curAnalogVal >= 900)
		{
			curFreq = notes[1];
		}
		else if (curAnalogVal >= 450)
		{
			curFreq = notes[2];
		}
		else if (curAnalogVal >= 4)
		{
			curFreq = notes[3];
		}
		else
		{
			curFreq = 0;
		}
		
		if (curFreq == 0)
		{
			curTimerCompVal = 0;
		}
		else
		{
			curTimerCompVal = TIMERCOMPVAL(curFreq, 8);			
		}



		//serialSendString("ADC Val");
		//serialSendInt(ADC);
		//serialSendString("curAnalogVal");
		//serialSendInt(curAnalogVal);
		//serialSendString("curFreq");
		//serialSendInt(curFreq);		
		//serialSendString("curtimercompval");
		//serialSendInt(curTimerCompVal);
		//serialSendString("OCR1A");
		//serialSendInt(OCR1A);
		//_delay_ms(750);	
    }
}

/***********************************************************************
 * loadSerialBuf
 * Description: adds a character to the serial buffer
 **********************************************************************/
void loadSerialBuf(char myChar)
{
	uint8_t nextInsertIndex = (insertIndex + 1) % BUFSIZE;
	
	if (nextInsertIndex != sendIndex)
	{
		serialBuf[insertIndex] = myChar;
		insertIndex = nextInsertIndex;
	}
}

/***********************************************************************
 * serialSendNext
 * Description: sends next character stored in buffer if there is data
 *      to send. 
 **********************************************************************/
void serialSendNext()
{
	if ( UCSR0A & (1 << UDRE0) && sendIndex != insertIndex) // can send and there is data to send
	{
		UDR0 = serialBuf[sendIndex];
		sendIndex = (sendIndex + 1) % BUFSIZE;
	}
}


/***********************************************************************
 * serialSendString
 * Description: sends a null terminated string through the serial line.
 *      String must fit in serial buffer (serial buffer size - 2).
 *      Minus 2 since one space is reserved for newline character,
 *      and second space reserved so that serialSendNext()'s
 *      sendIndex != insertIndex condition is not tripped making algorithm 
 *      think there is no data to send.
 **********************************************************************/
void serialSendString(char * myString)
{
	char *	ptrNextChar	= myString;
	uint8_t charsLeft	= BUFSIZE - 2;
	
	while (*ptrNextChar != '\0' && charsLeft != 0)
	{
		loadSerialBuf(*ptrNextChar);
		ptrNextChar += sizeof(char);
		charsLeft--;
	}
	loadSerialBuf('\n');
	serialSendNext();
}


/***********************************************************************
 * serialSendInt
 * Description: sends a signed integer throught the serial line as a
 *      string
 **********************************************************************/
void serialSendInt(int myInt)
{
	uint8_t	curDigit;
	int8_t  sign;
	int curNum = 10;	
	char intString[7];
	
    // add sign to beginning
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
	
    // convert int to string
	curNum = myInt;
	for (curDigit = 0; curDigit < 5; curDigit++)
	{
		intString[5 - curDigit] = sign * (curNum % 10) + 0x30;
		curNum /= 10;
	}
	
    // add null terminator to end of string and send
	intString[6] = '\0';
	serialSendString(intString);
}


/***********************************************************************
 * startAnalogConversion
 * Description: begin a single analog to digital conversion
 **********************************************************************/
void startAnalogConversion()
{
	ADCSRA |= (1 << ADSC);
}


/***********************************************************************
 * tranmission complete interrupt
 * Description: if transmission is complete send next available character
 *      in serial buffer.
 **********************************************************************/
ISR(USART_TX_vect)
{
	serialSendNext();
}


/***********************************************************************
 * analog to digital conversion complete interrupt
 * Description: store value of analog to digital conversion and
 *      immediately start a new conversion.
 **********************************************************************/
ISR(ADC_vect)
{
	curAnalogVal = ADCL;			// for some reason ADCL must be read first
	curAnalogVal += (ADCH << 8);
	
	startAnalogConversion();
}


/***********************************************************************
 * timer 1 compare interrupt
 * Description: turn pin 8 on and off. Off if any switch is not pressed. 
 *      If a switch is pressed, changes timer compare value and is turned
 *      on and off depending on the timer in order to generate tone.
 **********************************************************************/
ISR(TIMER1_COMPA_vect)
{
	if (curFreq == 0)					// if no switch is pressed, off
	{
		OCR1AH = 0;
		OCR1AL = 0xFF;					// set to 255 so that interrupt doesn't occur too frequently
		PORTB &= ~(1 << PORTB0);
	}
	else
	{
		OCR1AH = (curTimerCompVal >> 8);
		OCR1AL = curTimerCompVal;
		PORTB ^= (1 << PORTB0);
	}
}




/*
 * ArduinoProject9_motorizedPinwheel.c
 *
 * Created: 10/7/2019 6:24:30 PM
 * Author : Diego
 */ 

// CPU Macros
#define F_CPU 16000000UL

// Serial Transmission Macros
#define BAUD	9600
#define BUFSIZE	128 // MUST BE A POWER OF 2, otherwise circular buffer functions won't work!!!!

// Value Macros
#define TRUE	1
#define FALSE	0
#define ON		1
#define OFF		0
#define ERROR	0xFF
#define OK		0
#define EVEN	2
#define ODD		1
#define OUTPUT	1
#define INPUT	0

#include <avr/io.h>
#include <util/delay.h>
#include <limits.h>

// global button variables
uint8_t buttonPressed = FALSE;
uint8_t buttonSample  = OFF;
uint8_t prevButtonSample = OFF;

//serial transmission definitions and variables
struct circularBuffer
{
	char buf[BUFSIZE];
	uint8_t size;
	uint8_t readIndex;
	uint8_t writeIndex;
};

struct asyncTxInfo
{
	uint8_t interrupts;
	uint8_t payloadSize;
	uint8_t parityMode;
	uint8_t stopBits;
	unsigned long baudRate;
};

struct circularBuffer serialBuf = { .size		= BUFSIZE,
									.readIndex  = 0,
									.writeIndex = 0 };
									


//serial transmission functions
uint8_t	asyncTxSetup(struct asyncTxInfo *);
uint8_t cBufDataLength(struct circularBuffer *);
uint8_t cBufWrite(struct circularBuffer *, char);
uint8_t cBufRead(struct circularBuffer *, char *);
void	sendString(char *);
void	sendInt(int);

// pin functions
uint8_t	digitalPinSetup(uint8_t, uint8_t);
uint8_t	digitalPinSetPortVal(uint8_t, uint8_t);
uint8_t digitalPinRead(uint8_t);
uint8_t digitalPinTogglePortVal(uint8_t);


int main(void)
{
	// set up USART transmitter for debugging
	struct asyncTxInfo serialInfo = { .baudRate	   = 9600,
									  .interrupts  = ON,
									  .parityMode  = OFF,
									  .payloadSize = 8,
									  .stopBits    = 1 };
	
    asyncTxSetup(&serialInfo);
	
	// set up pins 3 and 9
	digitalPinSetup(3, INPUT);
	digitalPinSetup(9, OUTPUT);
	digitalPinSetPortVal(3, OFF);
	digitalPinSetPortVal(9, OFF);
	
	
    while (1) 
    {
		buttonSample = digitalPinRead(3);	// sample pin 3
		
		if (buttonSample != prevButtonSample)
		{
				// if button pressed, set state to the "button has been pressed"
				if (buttonSample == ON)
				{
					buttonPressed = TRUE;
					prevButtonSample = ON;
				}
				// if the button is not pressed, toggle port 9 if "button has been pressed" was previous state
				// set state to "button has not been pressed"
				else if (buttonSample == OFF)
				{
					if (buttonPressed == TRUE)
						digitalPinTogglePortVal(9);
					buttonPressed = FALSE;
					prevButtonSample = OFF;
				}
		}
    }
}

/******************************************************************************************
 * asyncTxSetup
 * description: sets up USART for asynchronous normal speed transmission based on the
 *				information provided in the asyncTxInfo struct.
 * return value: OK (0x00) if everything went well
 *				 ERROR (0xFF) if a value was out of range
 *****************************************************************************************/
uint8_t	asyncTxSetup(struct asyncTxInfo * serialInfo)
{
	unsigned int brrValue;	// baud rate register value
	
	// check value ranges
	if (serialInfo->interrupts != ON && serialInfo->interrupts != OFF)
		return ERROR;
	if (serialInfo->payloadSize < 5 || serialInfo->payloadSize > 9)
		return ERROR;
	if (serialInfo->parityMode != EVEN && serialInfo->parityMode != ODD && serialInfo->parityMode != OFF)
		return ERROR;
	if (serialInfo->stopBits != 1 && serialInfo->stopBits != 2)
		return ERROR;
	if (serialInfo->stopBits != 1 && serialInfo->stopBits != 2)
		return ERROR;
	if (serialInfo->baudRate < 245 || serialInfo->baudRate > 1000000)
		return ERROR;
	
	brrValue = F_CPU/(serialInfo->baudRate)/16 - 1;
	if (brrValue >= 4096)	// 2 to the 12th - 1 is limit of UBRR0 register
		return ERROR;
		
	UCSR0B =  (1 << TXEN0);			// turn on transmitter
	UCSR0C =  0x00;					// asynchronous mode

	// turn on transmission complete interrupt	
	if (serialInfo->interrupts == ON)
		UCSR0B |= (1 << TXCIE0);
		
	// set payload data size in bits		
	switch (serialInfo->payloadSize)
	{
		case 5:
			break;
		case 6:
			UCSR0C |= (1 << UCSZ00);
			break;
		case 7:
			UCSR0C |= (1 << UCSZ01);
			break;
		case 8:
			UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
			break;
		case 9:
			UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
			UCSR0B |= (1 << UCSZ02);
			break;
		default:	// 8 bit payload
			UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	}

	// set parity check
	if (serialInfo->parityMode == EVEN)
		UCSR0C |= (1 << UPM01);
	else if (serialInfo->parityMode == ODD)
		UCSR0C |= (1 << UPM01) | (1 << UPM00);
	
	// set stop bits
	if (serialInfo->stopBits == 2)
		UCSR0C |= (1 << USBS0);
		
	// set baud rate
	UBRR0L = brrValue;
	UBRR0H = (brrValue >> 8);
		
	return OK;
}

/******************************************************************************************
 * cBufDataLength
 * description: calculates number of elements in circular buffer awaiting to be sent by
 *				USART.
 * return value: the calculated number of elements in buffer
 * WARNING: the capacity of the circular buffer MUST be a power of 2 for this function to
 *			work.
 *****************************************************************************************/
uint8_t cBufDataLength(struct circularBuffer * cBuf)
{
	return ((cBuf->writeIndex - cBuf->readIndex) & (cBuf->size - 1));
}

/******************************************************************************************
 * cBufWrite
 * description: Adds a character to serial circular buffer if it is not full. 
 * return value: OK (0x00) if value added
 *				 ERROR (0xFF) if buffer was full
 * WARNING: the capacity of the circular buffer MUST be a power of 2 for this function to
 *			work.
 *****************************************************************************************/
uint8_t cBufWrite(struct circularBuffer * cBuf, char myChar)
{
	if (cBufDataLength(cBuf) < cBuf->size - 1)
	{
		cBuf->buf[cBuf->writeIndex] = myChar;
		cBuf->writeIndex = (cBuf->writeIndex + 1) & (cBuf->size - 1);
		return OK;
	}
	return ERROR;
}

/******************************************************************************************
 * cBufRead
 * description: Reads a character from serial circular buffer if it is not empty. The value
 *				read is stored in memory pointed at by myCharPtr
 * return value: OK (0x00) if value read
 *				 ERROR (0xFF) if buffer was empty
 * WARNING: the capacity of the circular buffer MUST be a power of 2 for this function to
 *			work.
 *****************************************************************************************/
uint8_t cBufRead(struct circularBuffer * cBuf, char * myCharPtr)
{
	if ( cBufDataLength(cBuf) > 0 )
	{
		*myCharPtr = cBuf->buf[cBuf->readIndex];
		cBuf->readIndex = (cBuf->readIndex + 1) & (cBuf->size - 1);
		return OK;
	}
	return ERROR;
}

/******************************************************************************************
 * sendString
 * description: Inserts a string into the serial buffer and transmits all content stored in
 *				it.
 * return value: None
 * WARNING: the capacity of the circular buffer MUST be a power of 2 for this function to
 *			work.
 *****************************************************************************************/
void sendString(char * myString)
{
	char * curStringChar = myString;
	char   curBufChar;
	
	while (*curStringChar != '\0' && cBufWrite(&serialBuf, *curStringChar) == OK)
	{
		// send all unsent letters already in buffer
		while ( cBufRead(&serialBuf, &curBufChar) == OK )
		{
			while ( (UCSR0A & (1 << UDRE0)) == FALSE ) {}	// wait until USART data register is empty
			UDR0 = curBufChar;
		}
		curStringChar += sizeof(char);
	}
}

/******************************************************************************************
 * sendInt
 * description: Converts and integer into a string, inserts this string into the serial
 *				buffer and transmits all content stored in the buffer.
 * return value: None
 * WARNING: the capacity of the circular buffer MUST be a power of 2 for this function to
 *			work as it calls sendString()
 *****************************************************************************************/
void sendInt(int myInt)
{
	char intString[8];
	int8_t isInd1 = 0;
	int8_t isInd2 = 0;
	char temp;
	int tempInt;
	int8_t sign = 1;
	
	// handle negative numbers
	if (myInt < 0)
	{
		intString[isInd1++] = '-';
		isInd2++;
		sign = -1;
	}
	
	if (myInt == 0)
	{
		intString[isInd1++] = '0';
	}
	else
	{
		// insert numbers in reverse order
		for(tempInt = myInt; tempInt != 0; tempInt /= 10) 
		{
			intString[isInd1++] = (char)( sign * (tempInt % 10) + 0x30 );
		}
	}
	
	// add newline and null terminator. Set isInd equal to the index of the last number
	intString[isInd1++] = '\n';
	intString[isInd1--] = '\0';
	isInd1--;
	
	// swap order of numbers into correct order
	while (isInd1 > isInd2)
	{
		temp = intString[isInd2];
		intString[isInd2++] = intString[isInd1];
		intString[isInd1--] = temp;
	}
	
	sendString(intString);
}

/******************************************************************************************
 * digitalPinSetup
 * description: Sets up one of Arduino digital pins 1-13 (ports B and D) to be either an
 *				input or output pin
 * return value: OK (0x00) if setup ok
 *				 ERROR (0xFF) if pin value out of range or data direction not valid
 *****************************************************************************************/
uint8_t digitalPinSetup(uint8_t pinNumber, uint8_t dataDirection)
{
	volatile uint8_t * ddrPtr;
	uint8_t			   bitNum;
	
	if ( pinNumber > 13 || (dataDirection != INPUT && dataDirection != OUTPUT) )
	{
		return ERROR;
	}
	
	if (pinNumber <= 7)
	{
		ddrPtr = &DDRD;
		bitNum = pinNumber;
	}
	else
	{
		ddrPtr = &DDRB;
		bitNum = pinNumber - 8;		
	}
	
	if (dataDirection == INPUT)
		*ddrPtr &= ~(1 << bitNum);
	else
		*ddrPtr |= (1 << bitNum);
	
	return OK;
}

/******************************************************************************************
 * digitalPinSetPortVal
 * description: Changes behaviour of digital pin to one of the following
 *				INPUT - ON:   enables pullup resistor
 *				INPUT - OFF:  disables pullup resistor
 *				OUTPUT - ON:  set pin to high
 *				OUTPUT - OFF: set pin to low			
 * return value: OK (0x00) if setup ok
 *				 ERROR (0xFF) if pin value out of range or pin state not valid
 *****************************************************************************************/
uint8_t digitalPinSetPortVal(uint8_t pinNumber, uint8_t state)
{
	volatile uint8_t * portPtr;
	uint8_t			   bitNum;
	
	if ( pinNumber > 13 || (state != ON && state != OFF) )
	{
		return ERROR;
	}
	
	if (pinNumber <= 7)
	{
		portPtr = &PORTD;
		bitNum = pinNumber;
	}
	else
	{
		portPtr = &PORTB;
		bitNum = pinNumber - 8;
	}
	
	if (state == OFF)
		*portPtr &= ~(1 << bitNum);
	else
		*portPtr |= (1 << bitNum);
	
	return OK;
}

/******************************************************************************************
 * digitalPinRead
 * description: reads high or low value in pin		
 * return value: ON (0x01) if pin reads high value
 *				 OFF (0x00) if pin reads low value
 *				 ERROR (0xFF) if pin number not valid
 *****************************************************************************************/
uint8_t digitalPinRead(uint8_t pinNumber)
{
	volatile uint8_t * pinPtr;
	uint8_t bitNum;
	uint8_t readings = 0xAA;
		
	if ( pinNumber > 13 )
	{
		return ERROR;
	}
		
	if (pinNumber <= 7)
	{
		pinPtr = &PIND;
		bitNum = pinNumber;
	}
	else
	{
		pinPtr = &PINB;
		bitNum = pinNumber - 8;
	}
	
	// debouncing code. 0xFF for consecutive ON readings and 0x00 for consecutive off readings 
	while (readings != 0xFF && readings != 0x00)
	{
		if ( (*pinPtr & (1 << bitNum)) == 0x00)
			readings = (readings << 1);
		else
			readings = (readings << 1) | 0x01;
		_delay_ms(2);
	}	
	
	if (readings == 0xFF)
		return ON;
	else
		return OFF;
}

/******************************************************************************************
 * digitalPinTogglePortVal
 * description: toggles behaviour of digital pin to one of the following
 *				INPUT pin:   either enables or disables pullup resistor
 *				OUTPUT pin:  sets pin to either high or low		
 * return value: OK (0x00) if setup ok
 *				 ERROR (0xFF) if pin value out of range or pin state not valid
 *****************************************************************************************/
uint8_t digitalPinTogglePortVal(uint8_t pinNumber)
{
	volatile uint8_t * portPtr;
	uint8_t			   bitNum;
	
	if ( pinNumber > 13 )
	{
		return ERROR;
	}
	
	if (pinNumber <= 7)
	{
		portPtr = &PORTD;
		bitNum = pinNumber;
	}
	else
	{
		portPtr = &PORTB;
		bitNum = pinNumber - 8;
	}

	*portPtr ^= (1 << bitNum);
	
	return OK;	
}
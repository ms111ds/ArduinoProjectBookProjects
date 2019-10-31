/*
 * ArduinoProject_11_CrystalBall.c
 *
 * Created: 10/26/2019 10:04:57 PM
 * Author : ms111ds 
 * Help from:	https://protostack.com.au/2010/03/character-lcd-displays-part-1/
 *				https://protostack.com.au/2010/03/character-lcd-displays-part-2/
 */ 

// CPU clock speed
#define F_CPU 16000000UL

// included libraries
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

// pin macros
#define _RS		12
#define _E		11
#define _D4		5
#define _D5		4
#define _D6		3
#define _D7		2
#define SWITCH	6
#define ANALOG	5
 
// logic and state macros
#define HIGH	1
#define LOW		0
#define INPUT	0
#define OUTPUT	1
#define TRUE	1
#define FALSE	0

// LCD command macros
#define RS_DATA	1
#define RS_INST	0

// 8ball replies
#define NUM_REPLIES 8
char * replies[NUM_REPLIES] = {	"Yes",
								"Most likely",
								"Certainly",
								"Outlook good",
								"Unsure",
								"Ask Again",
								"Doubtful",
								"No" };
								
// program state control variables
volatile uint8_t switchStateChanged = FALSE;

// digital pin functions
void digitalPinInit(uint8_t pinNum, uint8_t dataDirection);
void digitalPinWrite(uint8_t pinNum, uint8_t writeVal);
uint8_t digitalPinRead(uint8_t pinNum);

// analog pin functions
void analogPinSetup(uint8_t pinNum);
uint16_t analogPinRead(uint8_t pinNum);
void analogPinDisable(uint8_t pinNum);

// external interrupt functions
void enableExternalInterrupt(uint8_t pinNum);

// lcd functions
void lcdInit4Bit();
void lcdWriteNibble(char value);
void lcdWrite(uint8_t rsCommand, char value);
void lcdGoToLine(uint8_t lineNum);
void lcdClearDisplay();
void lcdWriteString(char *);


int main(void)
{
	uint8_t seed;
	uint8_t switchPressed = FALSE;
	uint8_t pinReading;
	
	// digital pin setup
	digitalPinInit(_RS,	OUTPUT);
	digitalPinInit(_E,	OUTPUT);
	digitalPinInit(_D4,	OUTPUT);
	digitalPinInit(_D5,	OUTPUT);
	digitalPinInit(_D6,	OUTPUT);
	digitalPinInit(_D7,	OUTPUT);
	digitalPinInit(SWITCH, INPUT);
	
	// get random seed
	analogPinSetup(ANALOG);
	_delay_ms(50); // delay a bit to let pin's value float around a bit
	seed = (uint8_t)analogPinRead(ANALOG);
	analogPinDisable(ANALOG);
	srand(seed);
	
	// initialize LCD display
	lcdInit4Bit();
	
	// Display Greeting Message
	lcdWriteString("Ask the");
	lcdGoToLine(2);
	lcdWriteString("Crystal Ball!");

	// initialize external interrupts for pin connected to the switch
	enableExternalInterrupt(SWITCH);
	
    while (1) 
    {
		if (switchStateChanged == TRUE) // if interrupt has triggered to to switch pin state change
		{
			pinReading = digitalPinRead(SWITCH);
			if ( (switchPressed == FALSE) && (pinReading == HIGH) )
			{
				switchPressed = TRUE;
			}
			
			// get new fortune after switch released
			else if ( (switchPressed == TRUE) && (pinReading == LOW) )
			{
				switchPressed = FALSE;
				lcdClearDisplay();
				lcdWriteString("The ball says:");
				lcdGoToLine(2);
				lcdWriteString( replies[ rand() % NUM_REPLIES ] );
				
			}
			// reset switch state flag and re-enable global interrupts
			switchStateChanged = FALSE;
			sei();
		}
    }
}


/******************************************************************************
 * digitalPinInit
 * description: Sets up an Arduino Uno digital pin to be either an input or
 *				output pin. Works for digital pins 0 - 13.
 *****************************************************************************/
void digitalPinInit(uint8_t pinNum, uint8_t dataDirection)
{
	volatile uint8_t * pinReg;
	uint8_t bitmask;
	
	// check for invalid parameters
	if ( (dataDirection != INPUT) & (dataDirection != OUTPUT) )
		return;
	if (pinNum > 13)
		return;
	
	// get pin register
	if (pinNum <= 7)
	{
		pinReg = &DDRD;
		bitmask = (1 << pinNum);
	}
	else
	{
		pinReg = &DDRB;
		bitmask = (1 << ( pinNum - 8 ));
	}
		
	// set data direction
	if (dataDirection == OUTPUT)
		*pinReg |= bitmask;
	else
		*pinReg &= ~bitmask; 
}


/******************************************************************************
 * digitalPinWrite
 * description: Sets an Arduino Uno digital pin to HIGH or LOW voltage.
 *				INPUT pin  -> HIGH: enables pullup
 *				INPUT pin  -> LOW : disables pullup
 *				OUTPUT pin -> HIGH: sets voltage to high value
 *				OUTPUT pin -> LOW : sets voltage to low value
 *****************************************************************************/
void digitalPinWrite(uint8_t pinNum, uint8_t writeVal)
{
	volatile uint8_t * pinReg;
	uint8_t bitmask;
	
	// check for invalid parameters
	if ( (writeVal != HIGH) & (writeVal != LOW) )
		return;
	if (pinNum > 13)
		return;
	
	// get pin register
	if (pinNum <= 7)
	{
		pinReg = &PORTD;
		bitmask = (1 << pinNum);
	}
	else
	{
		pinReg = &PORTB;
		bitmask = (1 << ( pinNum - 8 ));
	}
		
	// set pin state
	if (writeVal == HIGH)
		*pinReg |= bitmask;
	else
		*pinReg &= ~bitmask;
}

/******************************************************************************
 * digitalPinRead
 * description: Reads voltage on an Arduino Uno digital pin. This function
 *				debounces the reading and takes at least 60 milliseconds.
 *				Returns LOW if incorrect pin is provided.
 *****************************************************************************/
uint8_t digitalPinRead(uint8_t pinNum)
{
	volatile uint8_t * pinReg;
	uint8_t bitmask;
	int8_t  readCount;
	
	// check for invalid parameters
	if (pinNum > 13)
		return LOW;	

	// get pin register
	if (pinNum <= 7)
	{
		pinReg = &PIND;
		bitmask = (1 << pinNum);
	}
	else
	{
		pinReg = &PINB;
		bitmask = (1 << ( pinNum - 8 ));
	}
	
	// wait for 50 ms for bouncing to end and obtain 3 back to back readings of
	// the same value
	_delay_ms(50);
	readCount = 0;
	while ( (readCount < 3) && (readCount > -3) )
	{
		if (*pinReg & bitmask)
			readCount++;
		else
			readCount--;
		_delay_ms(5);
	}
	
	// return read value
	if ( readCount >= 3 )
		return HIGH;
	else
		return LOW;
}

/******************************************************************************
 * analogPinSetup
 * description: Sets up pin for analog to digital conversion. Does the following,
 *				1) Connects pin to ADC
 *				2) Selects 5 volt reference voltage
 *				3) Enables 128-bit prescaler
 *				4) Disables digital input register
 *****************************************************************************/
void analogPinSetup(uint8_t pinNum)
{
	if (pinNum > 5)
		return;
		
	ADMUX	= (1 << REFS0) | pinNum; // use 5 V, select pin
	ADCSRA	= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// enable ADC, 128 bit prescaler
	DIDR0   = (1 << pinNum);	// disable digital input register
}


/******************************************************************************
 * analogPinRead
 * description: Reads voltage on analog pin and starts an analog to digital 
 *				conversion. returns a value between 0 and 1024 which has a
 *				linear relationship to a read voltage between 0V and 5V
 *****************************************************************************/
uint16_t analogPinRead(uint8_t pinNum)
{
	uint16_t result;
	
	// get reading
	ADCSRA |= (1 << ADSC);
	while ( ((1 << ADIF) & ADCSRA) == 0 ) {} // loop while conversion is not ready
	
	// store reading result
	result = ADCL; // need to read low byte first, then high byte
	result |= (ADCH << 8);
	
	// clear reading complete flag
	ADCSRA |= (1 << ADIF); // clear the flag (for some reason need to write one)
	
	return result;
}

/******************************************************************************
 * analogPinDisable
 * description: Disables the ADC and enables the digital input to the analog
 *				pin.
 *****************************************************************************/
void analogPinDisable(uint8_t pinNum)
{
	ADCSRA &= ~(1 << ADEN);
	DIDR0  &= ~(1 << pinNum);
}

/******************************************************************************
 * enableExternalInterrupt
 * description: Turns on external interrupts to selected pin. Works only for
 *				Arduino digital pins 0 - 13. Also enables global interrupts.
 *				Interrupt triggered with any detected logical change to pin
 *				voltage.
 *****************************************************************************/
void enableExternalInterrupt(uint8_t pinNum)
{
	// check for invalid value
	if (pinNum > 13)
		return;
	else if (pinNum >= 8)
	{
		// setup for PCINT 0 - 5 (digital pins 8 - 13)
		PCICR  |= (1 << PCIE0);
		PCMSK0 |= ( 1 << (pinNum - 8) );
	}
	else
	{
		// setup for PCINT 16 - 23 (digital pins 0 - 7)
		PCICR  |= (1 << PCIE2);
		PCMSK2 |= ( 1 << pinNum);		
	}
	sei();	// enable global interrupts
}


/******************************************************************************
 * lcdInit4Bit
 * description: Initialization sequence for 4-bit operation of TC1602A-21T LCD
 *				(uses Hitachi HD44780 controller). Sets up the following,
 *				1) 4-bit operation
 *				2) 2 line mode
 *				3) display on
 *				4) cursor off
 *				5) clear the display
 *				6) Cursor moves to the right after character entered
 *****************************************************************************/
void lcdInit4Bit()
{
	// initial instruction, needs to be sent 3 times
	_delay_ms(16);	// delay after power on
	digitalPinWrite(_RS, LOW);
	digitalPinWrite(_E,  LOW);
	lcdWriteNibble(0x03); // send pulse with pins 4 and 5 on
	_delay_ms(6); // delay after initial instruction
	lcdWriteNibble(0x03);
	_delay_ms(2);
	lcdWriteNibble(0x03);
	
	// set parameters for lcd operation
	lcdWriteNibble(0x02);		// set to 4 bit operation
	lcdWrite(RS_INST, 0x28);	// for some reason set to 4-bit again, and 2 line mode
	lcdWrite(RS_INST, 0x0C);	// display on, cursor off, cursor blink off
	lcdWrite(RS_INST, 0x01);	// clear display (executed in 1.53 microseconds)
	_delay_ms(3);
	lcdWrite(RS_INST, 0x06);	// entry mode: increment (move right) cursor position
}

/******************************************************************************
 * lcdWriteNibble
 * description: sends 4-bit value to TC1602A-21T LCD
 *****************************************************************************/
void lcdWriteNibble(char value)
{
	// start with enable on low
	digitalPinWrite(_E,  LOW);
		_delay_us(500);
	
	// set up the high/low values on pins (high if bit == 1, low if bit == 0)
	digitalPinWrite(_D7, ( ( value & 0x08 ) >> 3));
	digitalPinWrite(_D6, ( ( value & 0x04 ) >> 2));
	digitalPinWrite(_D5, ( ( value & 0x02 ) >> 1));
	digitalPinWrite(_D4, (value & 0x01) );
	
	// send pulse to get lcd to read command
	digitalPinWrite(_E,  HIGH);
		_delay_us(500);
	digitalPinWrite(_E,  LOW);
		_delay_us(500);
}

/******************************************************************************
 * lcdWrite
 * description: sends full 8-bit commands or data to TC1602A-21T LCD. Has two
 *				required fields,
 *				rsCommand: RS_INST if sending instruction
 *						   RS_DATA if sending character data
 *				value: 8-bit command for LCD or the 8-bit character to write.
 *****************************************************************************/
void lcdWrite(uint8_t rsCommand, char value)
{
	// check _RS value
	if ( (rsCommand != RS_INST) && (rsCommand != RS_DATA) )
		return;
		
	digitalPinWrite(_E,  LOW);
	_delay_us(1000);
	
	// select to send instruction or data
	digitalPinWrite(_RS, rsCommand); // must be set high/low 30 ns before _E
	
	// send instruction/data high nibble
	lcdWriteNibble(value >> 4);

	// send instruction/data low nibble
	lcdWriteNibble(value);

}

/******************************************************************************
 * lcdGoToLine
 * description: moves the TC1602A-21T LCD cursor to the beginning (first address) 
 *				of the selected line.
 *****************************************************************************/
void lcdGoToLine(uint8_t lineNum)
{
	switch (lineNum)
	{
		case 1:
			lcdWrite(RS_INST, 0x80);	// move cursor to first address of first line
			break;
		case 2:
			lcdWrite(RS_INST, 0xC0);	// move cursor to first address of second line
			break;
		default:
			break;
	}
}

/******************************************************************************
 * lcdWriteString
 * description: writes a null terminated string to the TC1602A-21T LCD display.
 * Note: LCD must have already been initialized.
 *****************************************************************************/
void lcdWriteString(char * myString)
{
	char * curCharPtr = myString;
	char curChar = *myString;
	
	while (curChar != '\0')
	{
		lcdWrite(RS_DATA, curChar);
		curCharPtr++;
		curChar = *curCharPtr;
	}
}

/******************************************************************************
 * lcdClearDisplay
 * description: clears TC1602A-21T LCD display and returns cursor to the
 *				beginning (first address) of the first line.
 * Note: LCD must have already been initialized.
 *****************************************************************************/
void lcdClearDisplay()
{
	lcdWrite(RS_INST, 0x01);	// clear display (executed in 1.53 microseconds)
	_delay_ms(3);
}

/******************************************************************************
 * PCINT2 interrupt vector
 * description: Interrupt handler for when the switch is pressed or released.
 *				Raises the switchStateChanged flag and disables interrupts.
 *				Interrupts will be re-enabled in the main loop after handling
 *				the state change.
 * Note: PCINT2 interrupts and global interrupts must be enabled.
 *****************************************************************************/
ISR(PCINT2_vect)
{
	switchStateChanged = TRUE;
	cli();
}

/*
 * project_10_zoetrope.c
 *
 * Created: 10/17/2019 4:25:03 PM
 * Author : Diego
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Macros
#define ON		1
#define OFF		0
#define TRUE	1
#define FALSE	0
#define OUTPUT	1
#define INPUT	0
#define SWITCH_PIN	4
#define DIR_PIN		5

// global variables
uint8_t devicePower = OFF;

// digital pin functions
void digitalPinSetup(uint8_t pinNum, uint8_t direction);
void digitalPinWrite(uint8_t pinNum, uint8_t pinVal);
uint8_t digitalPinRead(uint8_t pinNum);
void digitalPinToggle(uint8_t pinNum);

// analog pin functions
void analogPinSetup(uint8_t pinNum);
uint16_t analogPinRead();
uint16_t calibrateAnalogVal(uint16_t analogVal);

// timer functions
void setupTimer1();
void startTimer1(uint8_t pinNum);
void stopTimer1(uint8_t pinNum);
void setCompTimer1(uint8_t pinNum, int16_t compVal);

int main(void)
{
	uint8_t		powerSwitchState = OFF;
	uint8_t		prevPowerSwitchState = OFF;
	uint8_t		dirSwitchState = OFF;
	uint8_t		prevDirSwitchState = OFF;
	uint16_t	analogVal;
	
	// set up timers and pins
	setupTimer1();
	
	digitalPinSetup(SWITCH_PIN, INPUT);
	digitalPinSetup(DIR_PIN, INPUT);
	digitalPinSetup(9, OUTPUT);
	digitalPinWrite(9, OFF);
	digitalPinSetup(2, OUTPUT);
	digitalPinWrite(2, ON); // must be in opposite state of pin 3
	digitalPinSetup(3, OUTPUT);
	digitalPinWrite(3, OFF); // must be in opposite state of pin 2
	
	analogPinSetup(0);
	
    while (1) 
    {
		powerSwitchState = digitalPinRead(SWITCH_PIN);
		dirSwitchState   = digitalPinRead(DIR_PIN);
		
		// ON-OFF switch toggle mechanism
		if (powerSwitchState == ON)
		{
			prevPowerSwitchState = ON;
		}
		else if (powerSwitchState == OFF)
		{
			// turn device power on or off once button is released
			if (prevPowerSwitchState == ON)
			{
				// turn on if previously off
				if (devicePower == OFF)
				{
					analogVal = calibrateAnalogVal(analogPinRead(0));
					setCompTimer1(9, analogVal);
					startTimer1(9);
					devicePower = ON;
				}
				else // turn off if previously on
				{
					stopTimer1(9);
					digitalPinWrite(9, OFF);
					devicePower = OFF;					
				}
				prevPowerSwitchState = OFF;
			}
		}
		
		// direction change mechanism, depends on ports 2 and 3 starting at
		// opposite states states (e.g. if port 2 is on, port 3 must be off)
		if (dirSwitchState == ON)
		{
			prevDirSwitchState = ON;
		}
		else
		{
			// toggle direction change once button is released
			if (prevDirSwitchState == ON)
			{
				digitalPinToggle(2);
				digitalPinToggle(3);
				prevDirSwitchState = OFF;
			}
		}
		
		
		// keep checking for the current analog value if device turned on
		if (devicePower == ON)
		{
			analogVal = calibrateAnalogVal(analogPinRead(0));
			setCompTimer1(9, analogVal);
		}
    }
}

/******************************************************************************
 * digitalPinSetup
 * Description: Sets up a digital pin as an input or output pin
 *****************************************************************************/
void digitalPinSetup(uint8_t pinNum, uint8_t direction)
{
	volatile uint8_t * dirRegPtr;
	uint8_t mask;
	
	// ensure valid pin and assign data direction register
	if (pinNum <= 7)
	{
		dirRegPtr = &DDRD;
	}
	else if (pinNum <= 13)
	{
		dirRegPtr = &DDRB;
	}
	else
		return;
	
	// create mask for selected pin
	mask =  (1 << (pinNum & 0x07)); // the pin number if <= 7,
									// or the pin number - 8 if >= 8
	
	// set data direction of pin
	if (direction == INPUT)
		*dirRegPtr &= ~mask;
	else if (direction == OUTPUT)
		*dirRegPtr |= mask;
}

/******************************************************************************
 * digitalPinWrite
 * Description: writes to the digital pin. Effect dependent on status of pin
 *		(being an input or output pin)
 * Effects: Output Pin - OFF, sets pin to low voltage state
 *					   - ON,  turns pin to high voltage state
 *			Input Pin  - OFF, disables pull-up resistor
 *					   - ON,  enables pullup resistor
 *****************************************************************************/
void digitalPinWrite(uint8_t pinNum, uint8_t pinVal)
{
	volatile uint8_t * curPortPtr;
	uint8_t mask;
	
	// ensure valid pin and assign port register
	if (pinNum <= 7)
	{
		curPortPtr = &PORTD;
	}
	else if (pinNum <= 13)
	{
		curPortPtr = &PORTB;
	}
	else
		return;
	
	// create mask for selected pin
	mask =  (1 << (pinNum & 0x07)); // the pin number if <= 7,
									// or the pin number - 8 if >= 8
	
	// set pin value
	// if output pin, turns it off or on
	// if input pin , disables or enables pull-up
	if (pinVal == OFF)
		*curPortPtr &= ~mask;
	else if (pinVal == ON)
		*curPortPtr |= mask;
}

/******************************************************************************
 * digitalPinRead
 * Description: Reads value of digital pin. NOTE that it incorporates a switch
 *		debouncing algorithm.
 *****************************************************************************/
uint8_t digitalPinRead(uint8_t pinNum)
{
	volatile uint8_t * curPinPtr;
	uint8_t mask;
	uint8_t readValues = 0xAA;
	
	// ensure valid pin and assign pin register
	if (pinNum <= 7)
	{
		curPinPtr = &PIND;
	}
	else if (pinNum <= 13)
	{
		curPinPtr = &PINB;
	}
	else
		return 0xFF;
	
	// create mask for selected pin
	mask =  (1 << (pinNum & 0x07)); // the pin number if <= 7,
									// or the pin number - 8 if >= 8
	
	// debouncing algorithm. Read pin register and shift in
	// either a 1 or a 0 to readValues depending on the reading
	while (readValues != 0xFF && readValues != 0x00)
	{
		if (*curPinPtr & mask)
			readValues = (readValues << 1) | 0x01;
		else
			readValues = (readValues << 1);
		
		_delay_ms(2);
	}
	
	// return that the pin was read as on or off
	if (readValues == 0xFF)
		return ON;
	else
		return OFF;
}

/******************************************************************************
 * digitalPinToggle
 * Description: toggles high and low voltage on pin. Effect dependent on status 
 *		of pin being an input or output pin.
 * Effects: Output Pin - OFF, sets pin to low voltage state
 *					   - ON,  turns pin to high voltage state
 *			Input Pin  - OFF, disables pull-up resistor
 *					   - ON,  enables pullup resistor
 *****************************************************************************/
void digitalPinToggle(uint8_t pinNum)
{
	volatile uint8_t * curPortPtr;
	uint8_t mask;
	
	// ensure valid pin and assign port register
	if (pinNum <= 7)
	{
		curPortPtr = &PORTD;
	}
	else if (pinNum <= 13)
	{
		curPortPtr = &PORTB;
	}
	else
		return;
	
	// create mask for selected pin
	mask =  (1 << (pinNum & 0x07)); // the pin number if <= 7,
									// or the pin number - 8 if >= 8
	
	// toggle pin value
	*curPortPtr ^= mask;
}

/******************************************************************************
 * analogPinSetup
 * Description: Connects analog pin to ADC with the following settings
 *		Reference voltage: Vcc
 *		Running mode: Free Running
 *		Prescaler: 128
 *		Result size: 10-bit
 *****************************************************************************/
void analogPinSetup(uint8_t pinNum)
{
	if (pinNum > 5)
		return;
	
	ADMUX |= (1 << REFS0) | pinNum; // set reference voltage to 5V,
									// turn on input in selected pin
	ADCSRA |= (1 << ADEN)  | (1 << ADATE) // turn on ADC and enable auto trigger
			| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler to 128
	
	ADCSRB &= ~( (1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0) ); // free running mode
	DIDR0 &= ~(1 << pinNum);
	
	ADCSRA |= (1 << ADSC); // start first conversion
}

/******************************************************************************
 * analogPinRead
 * Description: returns the latest result produced by the ADC
 *****************************************************************************/
uint16_t analogPinRead()
{
	uint16_t readVal = ADCL;
	
	readVal |= (ADCH << 8);
	return readVal;
}

/******************************************************************************
 * calibrateAnalogVal
 * Description: Returns an analog read value determined by previous calibration
 *		to the system.
 * Calibration Details: Reading linearly transformed between max and min 
 *      Max ADC value: Around 1010
 *      Min ADC value: Around 50
 *****************************************************************************/
uint16_t calibrateAnalogVal(uint16_t analogVal)
{
	if (analogVal > 1010)
	{
		return 1024;	
	}
	else if (analogVal > 50)
	{
		return (analogVal - 50) * 16 / 15; // (analogVal - 50) * 1024 / (1010 - 50)
	}
	else
	{
		return 0;
	}
}

/******************************************************************************
 * setupTimer1
 * Description: configures timer 1 to 10-bit fast PWM mode
 *****************************************************************************/
void setupTimer1()
{
	// set up 10-bit fast PWM mode		
	TCCR1A |= (1 << WGM11) | (1 << WGM10);
	TCCR1B |= (1 << WGM12);
}

/******************************************************************************
 * startTimer1
 * Description: Enables PWM on either pin 9 or pin 10. Then starts to produce
 *		the waveform.
 *****************************************************************************/
void startTimer1(uint8_t pinNum)
{
		if (pinNum < 9 || pinNum > 10)
			return;
		
	// set pin to high on bottom of time
	// set pin to low on compare match
		if (pinNum == 9)
			TCCR1A |= (1 << COM1A1);
		else
			TCCR1A |= (1 << COM1B1);

		// set prescaler to start timer
		TCCR1B |= (1 << CS10);
}

/******************************************************************************
 * stopTimer1
 * Description: Stops PWM on pin 9 or 10. Returns these pins to normal
 *		operation.
 *****************************************************************************/
void stopTimer1(uint8_t pinNum)
{
	if (pinNum < 9 || pinNum > 10)
		return;

	// disable timer		
	TCCR1B &= ~(1 << CS10);
	
	// disconnect pin from timer control
	if (pinNum == 9)
		TCCR1A &= ~( (1 << COM1A1) | (1 << COM1A0) );
	else
		TCCR1A &= ~( (1 << COM1B1) | (1 << COM1B0) );
}

/******************************************************************************
 * setCompTimer
 * Description: Sets the duty cycle of PWM wave. Duty cycle = compVal / 1024
 *****************************************************************************/
void setCompTimer1(uint8_t pinNum, int16_t compVal)
{
	if (pinNum < 9 || pinNum > 10)
		return;
	
	if (pinNum == 9)
	{
		OCR1AH = (compVal >> 8);
		OCR1AL = (compVal);
	}
	else
	{
		OCR1BH = (compVal >> 8);
		OCR1BL = (compVal);
	}		
}

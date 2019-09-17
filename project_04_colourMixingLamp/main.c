/*
 * ArduinoProject4.c
 *
 * Created: 8/22/2019 11:33:43 PM
 * Author : ms111ds 
 * PWM set up with great help from: https://www.youtube.com/watch?v=ZhIRRyhfhLM&t=3s
 * interrupts created with great help from: https://www.youtube.com/watch?v=-8BNeGIAiR8
 */ 
# define F_CPU 16000000UL
# define BAUD 9600
# define BRC ((F_CPU/BAUD/16)-1)


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

uint8_t PWMVal(uint8_t);
void	setupPWM();
void	setupADC();
void	startPWM();
void	getConversion();
void	eightBitAdd(uint8_t *,uint8_t *, uint8_t);
void	sendNum(uint8_t);

uint8_t curAnalogIn;

uint8_t dutyCycle09 = 0;
uint8_t dutyCycle10 = 0;
uint8_t dutyCycle11 = 0;

unsigned int pwmTempVal;
unsigned int newDutyCycle;

int main(void)
{
    // setup serial transmission (debugging only)
	UBRR0H = BRC >> 8;		// baud rate setup
	UBRR0L = BRC;			// baud rate setup
	
	UCSR0B = (1 << TXEN0);						// enable transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);		// 8-bit data, no parity bit, 1 stop bit

	// set up PWM on pins 9, 10, and 11
	setupPWM();				// set up PWM on pins 9, 10, and 11
	
	// set up analog to digital converter
	curAnalogIn = 0;		// first do conversion on analog port 0.
	setupADC();

	sei();				// enable global interrupts

	startPWM();			// begin timers
	getConversion();	// get first conversion
	//int i;
    while (1) 
    {
		//sendNum(9);
		//sendNum(dutyCycle09);
		//sendNum(10);
		//sendNum(dutyCycle10);	
		//sendNum(11);
		//sendNum(dutyCycle11);	
		//for(i = 0; i < 5; i++)
		//{
		//	UDR0 = '-';
		//	_delay_ms(100);
		//}
		
    }
}

/************************************************************************
 * PWMVal
 * Description: takes in the duty cycle (number between 0 and 100), 
 * multiplies it by 255, and divides it by 100. Used to get a good compare
 * value for PWM timers.
 ************************************************************************/
uint8_t PWMVal(uint8_t dutyCycle)
{
	uint8_t multHighByte = 0;						// upper byte of the product
	uint8_t multLowByte = 0;						// lower byte of the product
	uint8_t dutyLowNibble = dutyCycle & 0x0F;		// lower nibble of the duty cycle
	uint8_t dutyHighNibble = (dutyCycle >> 4);		// upper nibble of the duty cycle
	uint8_t operationTemp;							// value used to store results of operations
	uint8_t remainder;								// remainder of a division
	uint8_t divResult = 0;							// a quotient of a division operation
	
	int8_t curNibble;								// current nibble we are working with when dividing
	int8_t i;										// used as a counter
	
	// first we multiply 255 (0xFF) by the duty cycle
	// to do this, we multiply like we do by hand but using hexadecimal
	
	// First, multiply lower nibble of duty cycle by lower nibble of 255 (F)
	// result entirely in low byte
	multLowByte += dutyLowNibble * 0x0F;
	// Second, multiply lower nibble of duty cycle by higher nibble of 255 (also F)
	// result must be multiplied by 16, so
	// place lower nibble of result in higher nibble of low byte
	// place higher nibble of result in low nibble of high byte
	operationTemp = dutyLowNibble * 0x0F;
	eightBitAdd(&multHighByte, &multLowByte, (operationTemp << 4));
	multHighByte += operationTemp >> 4;
	// third, multiply higher nibble of duty cycle by lower nibble of 255 (also F)
	// result must be multiplied by 16, so
	// place lower nibble of result in higher nibble of low byte
	// place higher nibble of result in low nibble of high byte
	operationTemp = dutyHighNibble * 0x0F;
	eightBitAdd(&multHighByte, &multLowByte, (operationTemp << 4));
	multHighByte += operationTemp >> 4;
	// third, multiply higher nibble of duty cycle by higher nibble of 255 (also F)
	// result must be multiplied by 256, so entire result in high byte
	multHighByte += dutyHighNibble * 0x0F;	
	
	
	// divide by 100, to avoid potential overflow when trying to divide by 100 (0x64), e.g. 0x30
	// cant be divided by 0x64 so then it tries to divide by 0x30X, we divide by 10 twice
	//nibbles 3, 2, 1, 0 represent the 4 hex digits in the multiplication high and low bytes
	for (i = 2; i > 0; i--)
	{
		remainder = 0;
		for(curNibble = 3; curNibble >= 0; curNibble--)
		{
			switch(curNibble)
			{
				case 3:
					operationTemp = (multHighByte >> 4) + (remainder * 0x10);	// dividend = value to be divided + remainder * 16
					divResult =		operationTemp / 0x0A;
					remainder =		operationTemp % 0x0A;
					multHighByte =	(multHighByte & 0x0F) | (divResult << 4);	// low nibble + quotient
					remainder =		operationTemp % 0x0A;
					break;
				case 2:
					operationTemp = (multHighByte & 0x0F) + (remainder * 0x10);	// dividend = value to be divided + remainder * 16
					divResult =		operationTemp / 0x0A;
					multHighByte =	(multHighByte & 0xF0) | divResult;			// high nibble + quotient
					remainder =		operationTemp % 0x0A;
					break;
				case 1:
					operationTemp = (multLowByte >> 4) + (remainder * 0x10);	// dividend = value to be divided + remainder * 16
					divResult =		operationTemp / 0x0A;
					multLowByte =	(multLowByte & 0x0F) | (divResult << 4);	// low nibble + quotient
					remainder =		operationTemp % 0x0A;
					break;
				case 0:
					operationTemp = (multLowByte & 0x0F) + (remainder * 0x10);	// dividend = value to be divided + remainder * 16
					divResult =		operationTemp / 0x0A;
					multLowByte =	(multLowByte & 0xF0) | divResult;			// high nibble + quotient
					remainder =		operationTemp % 0x0A;
					break;
				default:
					break;
			}
		}
	}
	// the answer should be now stored in the low byte that stored the answer to our multiplication
	// as it is not larger than 255
	return multLowByte;
}

/************************************************************************
 * setupPWM
 * Description: set up required parameters for timers and pulse width
 *		modulation of digital ports 9, 10, and 11. Also enable interrupts
 *		for timers.
 ************************************************************************/
void setupPWM()
{
	// make pins 9, 10, and 11 output pins
	DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
	
	// ensure PWM is high starting at minimum and low after clock compare
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);	// Pin 9 and 10 (OC1A and OC1B)
	TCCR2A |= (1 << COM1A1);					// Pin 11 (OC2A)
	
	// set up 8 bit fast PWM
	TCCR1A |= (1 << WGM10);					// Pin 9 and 10 (OC1A and OC1B) 
	TCCR1B |= (1 << WGM12);					// Pin 9 and 10 (OC1A and OC1B)
	TCCR2A |= (1 << WGM21) | (1 << WGM20);	// Pin 11 (OC2A)
	
	// set up initial compare values
	OCR1AH = OCR1BH = 0x00;							// pin 9 and pin 10 upper byte (16-bit timer)
	OCR1AL = PWMVal(dutyCycle09);	// pin 9
	OCR1BL = PWMVal(dutyCycle10);	// pin 10
	OCR2A  = PWMVal(dutyCycle11);	// pin 11

	// enable timer interrupts
	TIMSK1 = (1 << TOIE1);
	TIMSK2 = (1 << TOIE1);
}

/************************************************************************
 * setupADC
 * Description: set up required registers for analog digital conversion.
 *		The analog pin determined by "curAnalogIn" is originally connected
 *		to the analog digital converter. Also enable interrupts for ADC.
 ************************************************************************/
void setupADC()
{
	ADMUX = (1 << REFS0) | (1 << ADLAR) | curAnalogIn;		// use internal 5V source, left adjust conversion result, hook up analog pin curAnalogIn to ADC
	ADCSRA =	(1 << ADEN)  | (1 << ADIE)					// turn on converter and enable ADC interrupts
	|	(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);			// set prescalar to 128 bits for good analog signal capture

	DIDR0 = (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);		// disable digital input buffer of analog pins
}

/************************************************************************
 * startPWM
 * Description: set timer prescalars to begin the timers (and the pulse
 * width modulation that they control) on pins 9, 10, and 11.
 ************************************************************************/
void startPWM()
{
	// set prescaler to 1024 (otherwise interrupts run too frequently and don't let anything else get done)
	// and start clock
	TCCR1B |= (1 << CS12) | (1 << CS10);		// pins 9 (green ) and 10 (blue)
	TCCR2B |= (1 << CS22) | (1 << CS21 ) | (1 << CS20);		// pin 11 (red)
}

/************************************************************************
 * getConversion
 * Description: Start Analog to digital Conversion
 ************************************************************************/
void getConversion()
{
	ADCSRA |= (1 << ADSC);
}

/************************************************************************
 * sendNum
 * Description: Send unsigned 8 bit integer through the Serial Connection
 ************************************************************************/
void sendNum(uint8_t myNum)
{
	uint8_t curNum;
	uint8_t remainder;
	uint8_t i;
	char numString[5];
	

	numString[4] = '\0';
	numString[3] = '\n';
	
	curNum = myNum;
	for( i = 2; i <= 2; i--)	// overflow to end loop
	{
		remainder = curNum % 10;
		numString[i] = (char)(remainder + 0x30);
		curNum /= 10;
	}

	for(i = 0; i < 5; i++)
	{
		UDR0 = numString[i];
		_delay_ms(100);
	}
}

/************************************************************************
 * eightBitAdd
 * Description: takes the memory locations of 2 8 bit buffers representing
 *		a 16 bit number, and adds "addVal" to them. Unsigned addition.
 ************************************************************************/
void	eightBitAdd(uint8_t * highByte, uint8_t * lowByte, uint8_t addVal)
{

	if ((uint8_t)(* lowByte + addVal) < * lowByte)	// check for overflow
	{
		* highByte += 1;
		* lowByte += addVal;
	}
	else
	{
		* lowByte += addVal;
	}
};

/********* Analog to Digital Conversion Complete Vector *************/
ISR(ADC_vect)
{
	newDutyCycle = ADCH * 100 / 255;
		
	switch(curAnalogIn)
	{
		case 0:
			dutyCycle11 = (uint8_t)newDutyCycle;
			break;
		case 1:
			dutyCycle10 = (uint8_t)newDutyCycle;
			break;
		case 2:
			dutyCycle09 = (uint8_t)newDutyCycle;
			break;
		default:
			break;
	}
	// switch to the next analog input channel.
	curAnalogIn = (curAnalogIn + 1) % 3;
	ADMUX = (ADMUX & 0xF0) | curAnalogIn;
			
	getConversion();
	
}

/********* timer 1 overflow vector *************/
ISR(TIMER1_OVF_vect)
{
	pwmTempVal = dutyCycle09 * 255 / 100;
	OCR1AL = (uint8_t)pwmTempVal;
	pwmTempVal = dutyCycle10 * 255 / 100;
	OCR1BL = (uint8_t)pwmTempVal;

}

/********* timer 2 overflow vector *************/
ISR(TIMER2_OVF_vect)
{
	pwmTempVal = dutyCycle11 * 255 / 100;
	OCR2A = (uint8_t)pwmTempVal;
}



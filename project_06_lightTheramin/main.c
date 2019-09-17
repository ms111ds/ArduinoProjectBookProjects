/*
 * ArduinoProject6_PotentiometerTheramin.c
 *
 * Created: 8/31/2019 11:30:27 PM
 * Author : ms111ds 
 * timer help from: https://www.youtube.com/watch?v=cAui6116XKc
 * NOTE: works much better with a potentiometer instead of a phototransistor!!!!!
 */ 
#define F_CPU	16000000UL
#define BAUD	9600
#define BRC		((F_CPU/BAUD/16)-1)
#define BUFSIZE 128


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*****************************************************************************************************
 * Global Variables
 ****************************************************************************************************/
// serial buffer global variables
char serialBuf[BUFSIZE];
volatile uint8_t insertIndex	= 0;
volatile uint8_t sendIndex		= 0;

// other global variables
int analogHigh				= INT16_MIN;
int analogLow				= INT16_MAX;
volatile int8_t calibTimer	= 7;			// seconds for calibration
volatile int adcVal			= 0;
int curPeriod				= 0;
volatile int targetPeriods	= 1;



/*****************************************************************************************************
 * Function Prototypes
 ****************************************************************************************************/
// serial functions
void sendChar(char);
void sendString(char *);
void sendInt(int);

// Analog Digital Converter Functions
void initADC();
void singleConvADC();

// Port 8 and timer Functions
void port8Init();
void timer0Init();
void timer0Start();
void timer1Init();
void timer1Start();
void timer1Off();

// calibration function
void calibrateAnalog();


int main(void)
{
	// local variables
	int calibAdcVal	= 0;
	int tempTargetPeriods;
	
	// USART setup
	UBRR0H = (BRC >> 8);					// upper 4 bits of baud rate prescaler
	UBRR0L = BRC;							// lower 8 bits of baud rate prescaler
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);	// asynchronous mode, no parity bits, 1 stop bit, 8 bit data size
	UCSR0B = (1 << TXCIE0) | (1 << TXEN0);	// enable transmitter interrupts and turn on transmitter
	
	// initialization functions
	initADC();
	port8Init();
	timer0Init();
	timer1Init();
	sei();									// enable global interrupts
	
	// start and calibrate ADC
	singleConvADC();
	timer1Start();
	calibrateAnalog();
	timer1Off();
	
	// start timer 0
	timer0Start();

    while (1) 
    {
		// the code below might be better done in the interrupt handlers, but since the multiplications with longs
		// requires many instructions, it was placed here to allow the interrupt handlers to execute as fast as
		// possible (especially the updating of the on/off status of pin 8)
			
		// convert ADC result to value in calibrated range
		calibAdcVal = adcVal;
		if (calibAdcVal > analogHigh)
		{
			calibAdcVal = analogHigh;
		}
		else if (calibAdcVal < analogLow)
		{
			calibAdcVal = analogLow;
		}
				
		// convert calibrated ADC value to the target number of periods (2 - 160)
		tempTargetPeriods = (long)(calibAdcVal - analogLow) * (160 - 2) / (analogHigh - analogLow) + 2;
		targetPeriods = tempTargetPeriods / 2 * 2;			// target periods needs to be divisible by 2 to
															// produce duty cycle of 50
		//sendInt(adcVal);
		//_delay_ms(1);
    }
}

/**************************************************************************************************************
 * sendChar
 * Description: Send a single character to serial buffer if there is room in buffer.
 *************************************************************************************************************/
void sendChar(char myChar)
{
	if( (insertIndex + 1) % BUFSIZE != sendIndex)
	{
		serialBuf[insertIndex] =  myChar;
		insertIndex = (insertIndex + 1) % BUFSIZE;		
	}

}

/**************************************************************************************************************
 * sendString
 * Description: Sends an entire string to serial buffer. Adds a newline character.
 * Note: myString must be null terminated
 *************************************************************************************************************/
void sendString(char * myString)
{
	char *	curChar = myString;
	uint8_t prevSendIndex;
	
	// send all characters in string and add newline character at end
	while(*curChar != '\0')
	{
		sendChar(*curChar);
		curChar += sizeof(char);
	}
	sendChar('\n');
	
	// start sending information if nothing is being sent and if there is stuff to send
	if( (UCSR0A & (1 << UDRE0) ) && (insertIndex != sendIndex) )
	{
		prevSendIndex = sendIndex;
		sendIndex = (sendIndex + 1) % BUFSIZE;
		UDR0 = serialBuf[prevSendIndex];	
	}
 
}

/**************************************************************************************************************
 * sendInt
 * Description: Sends a 16-bit integer to serial buffer. Adds newline character.
 *************************************************************************************************************/
void sendInt(int myInt)
{
	char stringInt[7]; // 1 sign, 5 digits, 1 null terminator
	int8_t i;
	int8_t signAdjust;
	int tempInt;
	
	// sign
	if (myInt < 0)
	{
		stringInt[0] = '-';
		signAdjust = -1;
	}
	else
	{
		stringInt[0] = ' ';
		signAdjust = 1;
	}
		
	// digits
	tempInt = myInt;
	for(i = 5; i > 0; i--)
	{
		stringInt[i] = (char)((tempInt % 10 * signAdjust) + 0x30);
		tempInt /= 10;
	}
	
	// null terminator	
	stringInt[6] = '\0';
	
	sendString(stringInt);
}

/**************************************************************************************************************
 * initADC
 * Description: initializes analog to digital converter on port A0.
 *************************************************************************************************************/
void initADC()
{
	ADMUX	= (1 << REFS0);									// use internal 5V, use analog pin 0
	ADCSRA	= (1 << ADEN)  | (1 << ADIE)  |					// turn on ADC, enable ADC interrupts
			  (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// use prescalar of 128
	DIDR0	= (1 << ADC0D);									// disable digital input
}

/**************************************************************************************************************
 * singleConvADC
 * Description: convert the current analog reading at port A0 to a digital value.
 *************************************************************************************************************/
void singleConvADC()
{
	ADCSRA |= (1 << ADSC);
}


/**************************************************************************************************************
 * port8Init
 * Description: initializes port 8. Starts with low volatage.
 *************************************************************************************************************/
void port8Init()
{
	DDRB |= (1 << DDB0);								// turn pin to output mode
}

/**************************************************************************************************************
 * timer0Init
 * Description: initializes timer 0. Need to obtain 8000 Hz since 2 periods at 8000 Hz is 4000 Hz and 160 
 *     periods at 8000 Hz is 50 Hz. CTC should increment counter that turns on port at every halfway point
 *		between 2 and 160. So we will use a prescalar of 8 and a compare value of 124 to obtain the desired 
 *		frequency. Freq_desired = Freq_clock / ( 2 * prescaler * ( 1 + compare value) ) 
 *************************************************************************************************************/
void timer0Init()
{
	TCCR0A = (1 << WGM01);								// use Clear Timer on Compare mode
	OCR0A  = 124;										// timer compare value of 124
	TIMSK0 = (1 << OCIE0A);								// enable interrupt for timer on compare match
}

/**************************************************************************************************************
 * timer0Start
 * Description: sets the timer prescaler to 8 and starts timer 0
 *************************************************************************************************************/
void timer0Start()
{
		TCCR0B = (1 << CS01);								// prescaler of 8 and start timer
}

/**************************************************************************************************************
 * timer1Init
 * Description: initializes timer 1 for calibration.
 *************************************************************************************************************/
void timer1Init()
{
	TCCR1A = 0x0;										// use normal port operations
	TCCR1B = (1 << WGM12);								// clear timer on compare
	OCR1A  = 15625;										// compares at 1 sec with prescaler = 1024 and F_CPU = 16000000
	TIMSK1 = (1 << OCIE1A);								// enable interrupt for timer on compare match
}

void timer1Start()
{
	TCCR1B |= (1 << CS12) | (1 << CS10);					// set prescaler to 1024 and start timer
}

void timer1Off()
{
	TCCR1B = 0x0;											// stop timer
}

/**************************************************************************************************************
 * calibrateAnalog
 * Description: desired function to calibrate the analog readings in A0.
 *************************************************************************************************************/
void calibrateAnalog()
{
	DDRB  |= (1 << DDB5);								// turn pin 13 to output mode
	PORTB |= (1 << PORTB5);								// turn on pin 13
	
	while (calibTimer > 0)
	{
		if (adcVal > analogHigh)
		{
			analogHigh = adcVal;
		}
		if (adcVal < analogLow)
		{
			analogLow = adcVal;
		}
	}
	
	PORTB &= ~(1 << PORTB5);							// turn pin 13 off
}

ISR(USART_TX_vect)
{
	if(sendIndex != insertIndex)
	{
		UDR0 = serialBuf[sendIndex];
		sendIndex = (sendIndex + 1) % BUFSIZE;
	}
}

ISR(ADC_vect)
{
	adcVal = ADC;
	singleConvADC();
}

ISR(TIMER1_COMPA_vect)
{
	calibTimer--;
	PORTB ^= (1 << PORTB5);
}

ISR(TIMER0_COMPA_vect)
{
	// next period
	curPeriod++;
	if (curPeriod >= targetPeriods)
	{
		curPeriod = 0;
	}
	
	// toggle port 8 when timer curPeriod resets or at the halfway point to targetPeriods
	if (curPeriod == 0)
	{
		PORTB &= ~(1 << PORTB0);						// set pin 8 to off
	}
	else if (curPeriod == targetPeriods / 2)
	{
		PORTB |= (1 << PORTB0);							// set pin 8 to on
	}

}

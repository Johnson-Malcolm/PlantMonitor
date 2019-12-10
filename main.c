
/*
 * FinalProject.c
 *
 * Created: 11/24/2019 2:44:23 PM
 * Author : majoh
 */ 

#include <avr/io.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <AVRXlib/AVRXClocks.h>
#include <AVRXlib/AVRXSerial.h>

/** Port-B bitmask of all four LEDs. */
#define F_CPU   2000000UL
#define width_14seg 7    //as total segments are 7 from A6 to A0
#define first_14seg 12   //use 12 for left to right as A6-h has first position at 12
//use 36 for right to left as A0-h has first position at 36
#define direction_14seg 0 //as direction is incrementing from left to right
//use one for right to left i.e. decrementing

//The following set of macros is for Numeric Segments
#define width_7seg 5       //as total segments are 5 from D4 to D0.... D4 is a 1 by default
#define first_7seg 10      //use 2 for right to left as D0-a has first position at 2
//use 10 for left to right as D4-a has first position at 10
#define direction_7seg 1   //as direction is decrementing from left to right
//use zero for right to left i.e. incrementing

/** Port-B bitmask of all four LEDs. */
#define LEDS_MASK ((uint8_t)0xF0)
/** The number of less-significant bits before the LEDs in their port. */
#define LEDS_OFFSET ((uint8_t)4)
#define TIMER_PERIOD ((uint16_t)(0.1 * F_CPU / 8 + 0.5))
/** Timer zero prescaler setting to divide the system clock by 8. */
#define TIMER_PRESCALE_8 0x04

/** USART rate in bits per second. */
#define USART_BAUD ((unsigned long)57600)

/** Circular receive buffer used by AVRXSerial. */
volatile char usart_rx_buffer[60];
/** Circular transmit buffer used by AVRXSerial. */
volatile char usart_tx_buffer[60];

/** semaphore for ADC sampling **/
volatile char bADCready = 1;
volatile char aADCready = 0;

/** AVRXSerial structure containing the state of USART C0. */
volatile XUSARTst usartC0;

//Value of the sensor
volatile uint16_t moisture;
volatile uint16_t voltage;

//Global variable for use on the LCD
unsigned char text[2][10]={"moist  ","sleep   "};
unsigned char ticker[] = "     ";
int count []= {8,12,14,15};
int place = 0;
int compare = 0;
int consumption;


void clock_init(void);
void lcd_init(void);
void display_alphanumeric(unsigned char *pattern);
void display_numeric(unsigned char *pattern);

/******************************************************************************************/
/************************************ISR******************************************************/
ISR(USARTC0_TXC_vect)
{
	Tx_Handler(&usartC0);
}

ISR(USARTC0_RXC_vect)
{
	Rx_Handler(&usartC0);
}

ISR(TCC0_OVF_vect) // Use of timer to slowdown the incoming amount of ADC values
{
	bADCready = 1;
}

ISR(ADCA_CH0_vect)
{
	moisture = ADCA_CH0_RES;
}

ISR(ADCB_CH0_vect)
{
	voltage = ADCB_CH0_RES;
}

ISR(RTC_OVF_vect){
	//bADCready = 1;
	sleep_disable();		
}

/******************************************************************************************/
/************************************  Setup  ******************************************************/
void init_lights()
{
	PORTB_DIR = 0xf1; // Sets the direction of the port B
	PORTB_OUT = 0xfe; // Turns off all the on board led lights
}


void USARTC0_initialize()
{
	unsigned long peripheralClock;
	GetSystemClocks(NULL, &peripheralClock);

	usartC0.RxCB = usart_rx_buffer;
	usartC0.TxCB = usart_tx_buffer;

	PMIC_CTRL |= PMIC_LOLVLEN_bm;  // Enable low priority interrupts

	if (USART_init(&usartC0,
	0xC0, peripheralClock, _USART_TXCIL_LO | _USART_RXCIL_LO,
	USART_BAUD / 100UL, -4,
	_USART_CHSZ_8BIT, _USART_PM_DISABLED, _USART_SM_1BIT) < 0)
	{
		//Error: usartC0 not instantiated
		while (1) {  // Freeze
		}
	}

	if (USART_buffer_init(&usartC0, 60, 60) < 0) {
		// Error: usartC0 not initialized
		while (1) {  // Freeze
		}


	}

	usartC0.fInMode = _INPUT_LF | _INPUT_ECHO;
	usartC0.fOutMode = _OUTPUT_CRLF;

	if (USART_enable(&usartC0, USART_TXEN_bm | USART_RXEN_bm) < 0) {
		// Error: invalid USART ID
		while (1) {  // Freeze
		}
	}
}


/** Initializes analog-to-digital B hardware, preparing it for other function calls.
*/
void init_ADCA()
{
	ADCA_CTRLA = ADC_ENABLE_bm;	// turns ADC system on
	ADCA_CTRLB = ADC_RESOLUTION_12BIT_gc; //| ADC_FREERUN_bm;	//sets resolution - 12-bit result, right adjusted
	ADCA_REFCTRL = ADC_REFSEL_INTVCC_gc; //selects the reference for the ADC INTVCC2 VCC/2
	ADCA_PRESCALER = ADC_PRESCALER_DIV512_gc; //defines the ADC clock relative to the peripheral clock DIV512
	
	ADCA_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;//sets channel inout and gain
	ADCA_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;//enables interrupt when ADC is complete, with HI priority
	
	PMIC_CTRL |= PMIC_HILVLEN_bm;//enables hi level interrupts to occur
}

void init_ADCB()
{
	ADCB_CTRLA = ADC_ENABLE_bm;	// turns ADC system on
	ADCB_CTRLB = ADC_RESOLUTION_12BIT_gc ;//| ADC_FREERUN_bm;	//sets resolution - 12-bit result, right adjusted
	ADCB_REFCTRL = ADC_REFSEL_INTVCC_gc; //selects the reference for the ADC INTVCC2 VCC/2
	ADCB_PRESCALER = ADC_PRESCALER_DIV512_gc; //defines the ADC clock relative to the peripheral clock DIV512
	
	ADCB_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;//sets channel inout and gain
	ADCB_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;//enables interrupt when ADC is complete, with HI priority
	
	PMIC_CTRL |= PMIC_HILVLEN_bm;//enables hi level interrupts to occur

	//ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
}
void init_TCC0()
{
	//clock running at 32MHz
	TCC0_PER = 0xF61C; // 
	TCC0_CTRLA = 0x07; // set the pre-scaler to 64


	TCC0_INTCTRLA = PMIC_LOLVLEX_bm; //set a medium priority interrup when C0 overflows
	PMIC_CTRL |= PMIC_LOLVLEN_bm;
}

void init_RTC(){
	///////////////RTC timer/////////////////////////
	CLK_RTCCTRL = CLK_RTCSRC_ULP_gc| CLK_RTCEN_bm;  //RTC setup clock running on 1Khz ULP
	RTC_PER = 0xF000; //1500 max val using the equation (1500ms/1ms =  1500)
	RTC_CTRL = 0x02;//prescaler 1 used

	RTC_INTCTRL |= PMIC_MEDLVLEN_bm; //medium priority
	PMIC_CTRL |= PMIC_MEDLVLEX_bm; // turn on medium priority interrupts
}



void init_PORTA(){
	PORTA_DIR = 0x00;
	PORTA_PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc;

}	

void init_PORTE(){
	PORTE_DIR = 0xef;
}

void Initialize_Ports(){
	init_PORTA();
	init_PORTE();
	init_lights(); //Port B
}

void Initialize_Timers(){
	init_RTC(); //used for sleep mode timing
	init_TCC0(); //used for adc readings
}

void Initialize_ADC(){
	init_ADCA();
	init_ADCB();
}

void Initialize_System(){
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	SetSystemClock(CLK_SCLKSEL_RC32M_gc, CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);
	Initialize_Ports();
	Initialize_Timers();
	Initialize_ADC();
	USARTC0_initialize();
	
	lcd_init();
	SetUpButtons();
	display_numeric(2);
	display_alphanumeric("Hello");
}
/********************************************************************************************/
/********************************* Helper Functions *****************************************************/
void enterSleep(){
	SLEEP_CTRL |= 0x01;
}

void leaveSleep(){
	SLEEP_CTRL = SLEEP_MODE_PWR_SAVE;
}

void init_sleep(){
	SLEEP_CTRL = SLEEP_MODE_PWR_SAVE;
}

void motorON(){
	PORTB_OUT |= 0x01;
}

void motorOFF(){
	PORTB_OUT = PORTB_OUT&~1;
}
void motorRun(){
	motorON();
	_delay_ms(100000);		//22ml/3s -------> 7.3 ml/s
	motorOFF();
	consumption+=22;
}
char getMoistureVal(){
	if(moisture < 2200){
		consumption = 0;
		return 'w';
	}else if(moisture < 2800){
		consumption = 0;
		return 'm';
	}else{
		consumption = 22;
		motorRun();
		return 'd';
	}
}

void leds_set_count(uint8_t count)
{
	//LEDs are active-low
	count = ~count;

	// Don't clobber non-LED bits on the same port
	count <<= LEDS_OFFSET;
	count &= LEDS_MASK;  // Technically unnecessary when LEDS_OFFSET is 4

	//Turn on the one
	PORTB_OUT |= count;
	//Turn off the zeros
	PORTB_OUT &= count | ~LEDS_MASK;
}

void readMoisture(){
	ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;
	ADCA_CTRLA |= ADC_CH0START_bm;
}
void readTemperature(){
	voltage = 0;
	ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN8_gc;
	ADCB_CTRLA |= ADC_CH0START_bm;
}

void readRHVoltage(){
	voltage = 0;
	ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN15_gc;
	ADCB_CTRLA |= ADC_CH0START_bm;
}
uint16_t getTemperatureF(){
	return (((voltage/1000.0)-0.5)*100)*9.0/5.0+32;
}

/////////////////////////////////LCD & Buttons///////////////////////////////////////////////
void clock_init()
{
	OSC_PLLCTRL=OSC_PLLFAC3_bm; //select internal 2MHz oscillator as PLL clock source, PLL multiplication factor as 8
	
	OSC_CTRL=OSC_PLLEN_bm; //enable PLL
	
	while(!(OSC_STATUS & OSC_PLLRDY_bm)); //wait until PLL is locked to desired frequency and ready to use
	
	CCP=0xd8; //write Configuration Change Protection register
	
	CLK_CTRL=CLK_SCLKSEL2_bm; //select PLL as system clock source
	
	CCP=0xd8; //write Configuration Change Protection register
	
	CLK_PSCTRL=CLK_PSADIV0_bm; //select Prescaler A as 2, Prescaler B and Prescaler C as 1, Clksys=16MHz,   Clkper4=Clkper2=Clkper=Clkcpu=8MHz
	
	CLK_RTCCTRL=CLK_RTCEN_bm; //enable RTC clock source as 1KHz from 32KHz ULP internal oscillator
}


void lcd_init()
{
	PORTE_DIR|=0x20; //define PORTE.5 as output pin
	
	PORTE_OUT|=0x20; //set PORTE.5 to turn on LCD backlight
	
	LCD_CTRLA|=LCD_ENABLE_bm|LCD_SEGON_bm; //enable LCD module and all segments of LCD
	
	LCD_CTRLB=LCD_PRESC_bm|LCD_CLKDIV1_bm|LCD_CLKDIV0_bm|LCD_LPWAV_bm; // prescaler 16, clock divider 4 and low power consumption.
	
	LCD_CTRLC=0x3f; //as all 40 pins are used as segment pins
	
	LCD_CTRLF=0x1f; //i.e. 31 is the value of FCONT[5:0]
}


void display_alphanumeric(unsigned char *pattern)
{
	unsigned char direction,width;
	
	direction=direction_14seg;
	
	width=width_14seg;
	
	LCD_CTRLG=LCD_TDG1_bm|first_14seg;    //14 segments & 4 COM terminals, first_14seg is starting segment
	
	if(direction)
	
	direction=LCD_DEC_bm; //i.e. if set decrement, else increment
	
	for(;width!=0;width--)
	{
		if(*pattern=='\0')
		
		break;
		
		LCD_CTRLH=direction|(*pattern++);
	}
}


void display_numeric(unsigned char *pattern)
{
	unsigned char direction,width;
	
	direction=direction_7seg;
	
	width=width_7seg;
	
	LCD_CTRLG=LCD_TDG0_bm|first_7seg;    //7 segments & 4 COM terminals, first_7seg is starting segment
	
	if(direction)
	
	direction=LCD_DEC_bm; //i.e. if set decrement, else increment
	
	for(;width!=0;width--)
	{
		if(*pattern=='\0')
		
		break;
		
		LCD_CTRLH=direction|(*pattern++);
	}
}

void SetUpButtons()
{
	PORTE_DIR = 0X00;	// Set all pins on port e as input
	PORTE_INT0MASK = 0X0f;   //Enables all 4 buttons
	PORTE_INTFLAGS = 0x1; //Enable port int0 vect
	PORTE_PIN0CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	PORTE_PIN1CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	PORTE_PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	PORTE_PIN3CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	
	PORTE_INTCTRL |= PMIC_MEDLVLEX_bm;
	PMIC_CTRL |= PMIC_MEDLVLEN_bm;
	
}


ISR(PORTE_INT0_vect)
{
	uint8_t btn =  0x0f & ~PORTE_IN; //Buttons are active low
	switch (btn)
	{
		case 1: //button 0
		break;
		case 2://button 1
		break;
		case 4://button 2
		break;
	}
}

void displayTemp(unsigned char alphaN[10], uint16_t val)
{
	display_alphanumeric(alphaN);
	ticker[1] = (char)val/10 + 48;
	ticker[2] = (char)val%10 + 48;
				
	display_numeric(ticker);
}

void displayMoisture(unsigned char alphaN[10], char moistVal)
{
	ticker[1] = ' ';
	ticker[2] = ' ';
			
	display_numeric(ticker);
			
	text[0][6] = moistVal;
			
	display_alphanumeric(text[0]);
			
}





int main(void)
{
	static char input_buffer[32], output_buffer[32];
	
	char moistureVal, tempVal;
	uint16_t temperatureF, RHvoltage;
	
	cli();
	Initialize_System();
	init_sleep(); //Prepare for return after sleep
	enterSleep();
	sleep_enable();
	sei();
	
	
	
    /* Replace with your application code */
    while (1) 
    {
		
		
		
		if (bADCready==1) {
			bADCready = 0;
			readTemperature();
			_delay_ms(500);

			temperatureF = getTemperatureF();
			
			readRHVoltage();
			_delay_ms(500);
			RHvoltage = voltage;//(voltage-2500) * 5 / 1023;
			
			
			readMoisture();
			_delay_ms(2000);
			moistureVal = getMoistureVal();

			displayTemp("Temp F-", temperatureF);
			_delay_ms(100000);
			
			displayMoisture(text[0], moistureVal);
			_delay_ms(100000);
						
			sprintf(output_buffer,"push %c %d %d \r\n", moistureVal,temperatureF, consumption);

			USART_send(&usartC0, output_buffer);
			USART_RxFlush(&usartC0);
			
			display_alphanumeric(text[1]);
			_delay_ms(100000);
			
			sleep_cpu();
			init_sleep(); //Prepare for return after sleep
			enterSleep();
			sleep_enable();
		}
		
    }
}


/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/
/************************************************************************************************************************************/
/***************************************************************************************************************************************/


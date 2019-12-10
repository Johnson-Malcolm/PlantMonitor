
/*
 * FinalProject.c
 *
 * Created: 11/24/2019 2:44:23 PM
 * Author : majoh
 */ 

#include <avr/io.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <AVRXlib/AVRXClocks.h>
#include <AVRXlib/AVRXSerial.h>

/** Port-B bitmask of all four LEDs. */
#define F_CPU   2000000UL
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
	//bADCready = 1;
	//PORTE_OUT ^= 1UL << 0;
	//PORTB_OUT = 0xef;
	//_delay_ms(1000);
	//PORTB_OUT = 0xff;
	//_delay_ms(1000);	
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
	//sleep_disable();
	bADCready = 1;		
}

/******************************************************************************************/
/************************************  Setup  ******************************************************/
void init_lights()
{
	PORTB_DIR = 0xf0; // Sets the direction of the port B
	PORTB_OUT = 0xff; // Turns off all the on board led lights
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
	RTC_PER = 0x280A; //1500 max val using the equation (1500ms/1ms =  1500)
	RTC_CTRL = 0x01;//prescaler 1 used

	RTC_INTCTRL |= PMIC_MEDLVLEN_bm; //medium priority
	PMIC_CTRL |= PMIC_MEDLVLEX_bm; // turn on medium priority interrupts
}



void init_PORTA(){
	PORTA_DIR = 0x00;
	PORTA_PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	PORTA_PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;

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
	init_sleep();
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
	PORTE_OUT = 0x01;
}

void motorOFF(){
	PORTE_OUT = 0x00;
}

char getMoistureVal(){
	if(moisture < 2200){
		return 'w';
	}else if(moisture < 2800){
		return 'm';
	}else{
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
	ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN8_gc;
	ADCB_CTRLA |= ADC_CH0START_bm;
}

void readRHVoltage(){
	ADCB_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN10_gc;
	ADCB_CTRLA |= ADC_CH0START_bm;
}
uint16_t getTemperatureF(){
	return (((voltage/1000.0)-0.5)*100)*9.0/5.0+32;
}

int main(void)
{
	static char input_buffer[32], output_buffer[32];
	
	char moistureVal, tempVal;
	uint16_t temperatureF, RHvoltage;
	
	cli();
	Initialize_System();
	//init_sleep();
	//enterSleep();
	//ADCB_initialize();
	//sleep_enable();
	sei();
	//sleep_cpu();
	
	
    /* Replace with your application code */
    while (1) 
    {
		readTemperature();
		_delay_ms(500);

		temperatureF = getTemperatureF();
		
		readRHVoltage();
		_delay_ms(500);
		RHvoltage = voltage;
		
		
		readMoisture();
		_delay_ms(2000);
		
		if (bADCready==1) {
			bADCready = 0;
			moistureVal = getMoistureVal();
			
			//sprintf(output_buffer, "%d temperature Voltage", tempVoltage);


			
			USART_send(&usartC0, output_buffer);
			USART_RxFlush(&usartC0);
			
			sprintf(output_buffer, "%d temperature", temperatureF);

			USART_send(&usartC0, output_buffer);
			USART_RxFlush(&usartC0);
			
			sprintf(output_buffer, "%d RHvoltage", RHvoltage);

			USART_send(&usartC0, output_buffer);
			USART_RxFlush(&usartC0);
			
			//sprintf(output_buffer, "%d moisture", moisture);

			sprintf(output_buffer, "%c moisture\r\n", moistureVal);
		
	
			USART_send(&usartC0, output_buffer);
			USART_RxFlush(&usartC0);

			
			
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


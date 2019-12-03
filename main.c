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
volatile char bADCready = 0;
volatile char aADCready = 0;

/** AVRXSerial structure containing the state of USART C0. */
volatile XUSARTst usartC0;

//Value of the sensor
volatile int gForce;
volatile int gSample;
volatile uint16_t sample;
volatile uint16_t force;

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
	if(bADCready){
		sample = ADCA_CH0_RES;
	}
}

ISR(RTC_OVF_vect){
	sleep_disable();
	PORTB_OUT = 0xef;
	_delay_ms(1000);
	PORTB_OUT = 0xff;
	_delay_ms(1000);			
}

void setuplights()
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
void ADCA_initialize()
{
	ADCA_CTRLA = ADC_ENABLE_bm;	// turns ADC system on
	ADCA_CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_FREERUN_bm;	//sets resolution - 12-bit result, right adjusted
	ADCA_REFCTRL = ADC_REFSEL_INTVCC_gc; //selects the reference for the ADC INTVCC2 VCC/2
	ADCA_PRESCALER = ADC_PRESCALER_DIV512_gc; //defines the ADC clock relative to the peripheral clock DIV512
	
	ADCA_CH0_CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;//sets channel inout and gain
	ADCA_CH0_INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;//enables interrupt when ADC is complete, with HI priority
	
	PMIC_CTRL |= PMIC_HILVLEN_bm;//enables hi level interrupts to occur

	ADCA_CH0_MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;
	ADCA_CTRLA |= ADC_CH0START_bm;
}


void timer_initialize()
{
	TCC0_PER = 0x280A; // set the range of the counter 15,625
	TCC0_CTRLA = 0x01;

	TCC0_INTCTRLA = PMIC_MEDLVLEX_bm;
	PMIC_CTRL = PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm;
}

void init_rtc(){
	///////////////RTC timer/////////////////////////
	CLK_RTCCTRL = CLK_RTCSRC_ULP_gc| CLK_RTCEN_bm;  //RTC setup
	RTC_PER = 0x280A; //1500 max val using the equation (1500ms/1ms =  1500)
	RTC_CTRL = 0x01;//prescaler 1 used

	RTC_INTCTRL |= PMIC_MEDLVLEN_bm; //medium priority
	PMIC_CTRL |= PMIC_MEDLVLEX_bm; // turn on medium priority interrupts
}

void init_sleep(){
	SLEEP_CTRL = SLEEP_MODE_PWR_SAVE;
}

void enterSleep(){
	SLEEP_CTRL |= 0x01;
}

void leaveSleep(){
	SLEEP_CTRL = SLEEP_MODE_PWR_SAVE;
}

void setupPORTA(){
	PORTA_DIR = 0x00;
	PORTA_PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
}

int main(void)
{
	static char input_buffer[32], output_buffer[32];
	
	//set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	SetSystemClock(CLK_SCLKSEL_RC32M_gc, CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);
	cli();
	setuplights();
	//init_rtc();
	//init_sleep();
	//enterSleep();
	setupPORTA();
	timer_initialize();
	USARTC0_initialize();
	ADCA_initialize();
	//sleep_enable();
	sei();
	//sleep_cpu();
	
	
    /* Replace with your application code */
    while (1) 
    {
		//PORTB_OUT = 0xdf;
		//_delay_ms(1000);
		//PORTB_OUT = 0xff;
		//_delay_ms(1000);
		if (bADCready==1) {
			bADCready = 0;
			sprintf(output_buffer, "%d moisture\r\n", sample );
			USART_send(&usartC0, output_buffer); // prints out light sample
			USART_RxFlush(&usartC0); // clear rx buffer
		}
    }
}


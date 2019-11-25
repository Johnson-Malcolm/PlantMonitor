/*
 * FinalProject.c
 *
 * Created: 11/24/2019 2:44:23 PM
 * Author : majoh
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

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

void timer_initialize()
{
	TCC0_PER = 0x280A; // set the range of the counter 15,625
	TCC0_CTRLA = 0x0;

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
int main(void)
{
	//set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	cli();
	setuplights();
	init_rtc();
	init_sleep();
	enterSleep();
	//sleep_enable();
	sei();
	sleep_cpu();
	
	
    /* Replace with your application code */
    while (1) 
    {
		PORTB_OUT = 0xdf;
		_delay_ms(1000);
		PORTB_OUT = 0xff;
		_delay_ms(1000);
    }
}


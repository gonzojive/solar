/*----------------------------------------------------------------------------------------------------------------------*/
/*
 * **** Solar Grid_Intertie Inverter	by Tim Nolan	copyright 2008-2009 	******
 * 
 * 	This is the software for the auxilary ATtiny45 processor in my solar inverter project.
 * 	I'm releasing this software to the public with no restrictions. If you use it in your project all I ask is a 
 *	credit and a link back to my website www.timnolan.com.	.
 *
 * Fuses should be programmed as 0xC2, 0xDC, 0xFF for low, high and extended in ISP mode.
 * So set BOD VCC=4.3V and Int. RC Osc. 8Mhz; Startup time PWRDWN/RESET 6k CK/14 CK + 0ms
 * and disable CLKDIV8. Then program DWEN to get to debug wire mode.
 *
 * v0.04 8/11/08	Moved into new hardware.
 *
 * v0.5 1/2/09		Added and updated comments for website.
 *
 */
/*----------------------------------------------------------------------------------------------------------------------*/


#include <inttypes.h>
#include <io.h>
#include <interrupt.h>
#include <delay.h>

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define DEL 1				//define time delay for spi output
#define NUM 2				//number of average
#define SAMPLES 64
#define TRUE 1
#define FALSE 0
#define SCALE 5

/*---------------------------------------------------------------------*/
// Global variables
/*---------------------------------------------------------------------*/

	uint16_t adc_result, adc_temp, rms_volts;
	uint8_t temp_lsb, temp_msb;
	uint8_t temp_in;
	uint8_t test;
	int32_t adc_sum;
	uint8_t ser_out;

/*----------------------------------------------------------------------*/
// Simple Interger square root function  				
/*----------------------------------------------------------------------*/
uint16_t int_sqrt(uint32_t a){

	uint32_t x = 1;

	while((x * x) < a) {
    	x++;
	}

	return((uint16_t)x); 
}
/*---------------------------------------------------------------------*/
// More efficient square root function.

uint16_t int_sqrt2(uint32_t a) {

	uint32_t square = 1;
	uint32_t delta = 3;
	
	while(square <= a){
    	square += delta;
    	delta += 2;
	}

    return((uint16_t)(delta/2 - 1));
} 
/*-------------------------------------------------------------------*/
/* clock serial output of 10 bits */
void send_10bits(uint16_t cc) {

	uint16_t ser_out;
	uint8_t i;
	uint8_t clock=0;

	ser_out = 0xF800 | (cc << 1);
	for (i=0; i<12; i++) {
		if(ser_out & 0x01) PORTB |= _BV(PORTB1);
		else PORTB &= ~_BV(PORTB1);
		ser_out >>= 1;
		if(clock++ & 0x01) PORTB |= _BV(PORTB0);
		else PORTB &= ~_BV(PORTB0);
		_delay_loop_2(2*48);						// hand tweaked to get 104uS bit times for 9600 baud (range 1uS to 16384uS)
		if(clock++ & 0x01) PORTB |= _BV(PORTB0);
		else PORTB &= ~_BV(PORTB0);
		_delay_loop_2(2*48);						
	}
}
/*---------------------------------------------------------------------*/
// This interrupt occurs every 260uS so we get 64 interrupts in during one AC waveform or 16.66ms.
// We get a sample from the adc every interrupt and sum them so we have 64 ac voltage samples over one complete
// ac waveform. We divide by 64 to get the average AC voltage and take the square root to get RMS AC voltage.
//
ISR(TIM1_OVF_vect) {

	uint8_t templ, temph, temp8;
	static uint8_t count = 0;
	uint16_t temp16;
	uint16_t result;
	static uint16_t prev=0;

	if (count < SAMPLES) {							//if we are still counting
		ADCSRA |= _BV(ADSC);						//start next conversion
		loop_until_bit_is_clear(ADCSRA, ADSC);		//wait until conversion is done
		templ = ADCL;
		temph = ADCH;
		temp16 = (temph << 8) | templ;				//get 16 bit adc result in temp16
		adc_sum += (int32_t)temp16 * (int32_t)temp16;	//keep a running sum over 64 ac volts samples
		count++;
	}
	else {											//when we have gotten 64 samples calculate rms ac volts
	   	PORTB |= _BV(PORTB2);
		rms_volts = int_sqrt2(adc_sum / SAMPLES);	//divide the sum by number of sample to get average and 
		count = 0;									//take the square root to get the rms voltage
		adc_sum = 0;
		result = (rms_volts / 4) + ((prev * 3) / 4);//filter the result to smooth it.
		prev = result;
		send_10bits(result);						//send out the result as a 10 bit serial stream.
	    PORTB &= ~_BV(PORTB2);		
	}
}
/*---------------------------------------------------------------------*/
/*---------------------------------------------------------------------*/
void ioinit (void) 
{
    DDRB = _BV(PORTB4) | _BV(PORTB2) | _BV(PORTB1) | _BV(PORTB0);		 
    PORTB |= _BV(PORTB4);										//turn on green led 

	ADMUX = _BV(MUX1) | _BV(MUX0); 								// set external 5v ref, use ADC3(PB3) input
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1); 	// enable A/D, start conversion, prescaler divide by 64
	DIDR0 = _BV(ADC3D);											// disable input register on ADC3
																// setup timer1
	TCCR1 = _BV(CTC1) | _BV(PWM1A) | _BV(CS12) | _BV(CS10);		// clear clock on OCR1C match, set pwm mode, clk/16
	OCR1C = 130;												// set timer period
	TIMSK = _BV(TOIE1);											// set timer overflow interrupt

    sei ();														/* enable interrupts */
}

/*----------------------------------------------------------------------*/
int main (void)  {        

	uint16_t led_count=0;

    ioinit();

    do {							/* loop forever */

       if (++led_count & 0x4000) 
          PORTB |= _BV(PORTB4);		//blink the LED
       else
          PORTB &= ~_BV(PORTB4);

	} while(1);

    return (0);
}

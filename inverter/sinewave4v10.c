//---------------------------------------------------------------------------------------------------------------------
/*
 * **** Solar Grid_Intertie Inverter	by Tim Nolan	copyright 2008-2009 	******
 * 
 * 	This is the software for the main AT90PWM3B processor in my solar inverter project.
 * 	I'm releasing this software to the public with no restrictions. If you use it in your project all I ask is a 
 *	credit and a link back to my website www.timnolan.com.	.
 *
 * Fuses should be programmed as 0xF3, 0xDA, 0xF9 for low, high and extended in ISP mode.
 * So set BOD VCC=4.2V and PLL clock 16Mhz; Startup time PWRDWN/RESET 16k CK/14 CK + 0ms
 * and disable CLKDIV. Then program DWEN to get to debug wire mode.
 *
 * v0.01 12/21/07	First try to get something to work.
 *
 * v0.04 8/11/08	Moved into new hardware.
 *
 * v0.10 1/2/09		Added and updated comments for website.
 *
 */
//---------------------------------------------------------------------------------------------------------------------

#include <inttypes.h>
#include <avr\io.h>
#include <interrupt.h>
#include <delay.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

//#define START_STR "Solar Inverter by Tim Nolan  Copyright 2008 "
//#define VERS "v0.01"									// change version number for each version of code release
#define BAUD9600  103 									/* Run the UART at 9600 baud @16MHz fosc. */
#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0
#define SETPOINT 500
#define KD 1
#define F60HZ 4166
#define F60HZ64 65
#define F60HZMAX 4267
#define F60HZMIN 4067
#define FSETPOINT 2180
#define FSETPOINT2 0
#define ASETPOINT 300
#define FREQ_SCALE 100000000
//#define FREQ_SCALE 102000000
#define USEC_INC 4
#define ADJ_START 100
#define ADJ_MIN 95
#define ADJ_MAX 205
#define AMP_START 58
#define ADJ16_START 120		
#define ADJ16_MIN 100
#define ADJ16_MAX 1000
#define INC_HI 5
#define INC_LO 2
#define MIN_DC_AMPS 10
#define MAX_DC_AMPS 500
#define FAULT_MAX_DC_AMPS 600
#define FAULT_MIN_DC_AMPS 0
#define MIN_DC_VOLTS 1200
#define MAX_DC_VOLTS 2000
#define FAULT_MIN_DC_VOLTS 1150
#define FAULT_MAX_DC_VOLTS 2200
#define MIN_ADJ16 130
#define MAX_ADJ16 1000
#define ADJ_INC_POS 10
#define ADJ_INC_NEG -10
#define MIN_FREQ 54
#define MAX_FREQ 66
#define MIN_AC_VOLTS 108
#define MAX_AC_VOLTS 132
#define FAULT_SECONDS 5

#define TURN_ON	{PORTC &= ~(_BV(PORTC4) | _BV(PORTC5)); PCTL2 = _BV(PRUN2); pp_switch = ON;} 	
															/*enable UF_EN and RC_EN by setting PC4 and PC5 low*/	
						    								/*start power state timers*/
#define TURN_OFF {PORTC |= (_BV(PORTC4) | _BV(PORTC5)); PCTL2 &= ~_BV(PRUN2); pp_switch = OFF;}	
															/*disable UF_EN and RC_EN by setting PC4 and PC5 high*/	
															/*stop power stage timers*/

//------------------------------------------------------------------------------------------
// Function prototypes
//------------------------------------------------------------------------------------------
void update_adj(int16_t);
uint8_t check_faults(void);
//int16_t fir(uint16_t);
void ioinit (void); 
uint8_t kbhit(void); 
int uart_getchar(void);
int uart_putchar(unsigned char);

//------------------------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------------------------
// 1/2 sinewave table full 8 bit swing					
 uint8_t const pwmtab[32]={25,50,74,98,120,142,162,180,197,212,	
 225,235,244,250,254,255,254,250,244,235,				
 225,212,197,180,162,142,120,98,74,50,25,0};				

enum { POS, NEG };
enum { VOLT, AMP, TEMP };
enum pp_states { WAIT, SYNC, PP_ON, PP_TRACK, PP_SWEEP, PP_DOWN, PP_OFF, PP_FAULT, NIL } state;

volatile uint8_t polarity;
volatile uint8_t adc_type;
volatile uint8_t index1;

uint16_t temp;
uint16_t bps;
uint16_t shift;
uint16_t counter;
uint16_t pwm;
uint8_t pwm_temp;
uint8_t pwm_adj;
uint8_t adj = ADJ_START;
uint16_t temp16;
uint32_t temp32;
uint8_t temp_msb;
uint8_t temp_lsb;
int16_t error;
int16_t adj16 = ADJ16_START;
int16_t error_sum;
uint8_t tcount;
uint8_t tindex;
int16_t comp1a_max = F60HZ;					
int16_t comp1b_inc;
int16_t comp1b_prev;
int16_t edge_temp;
int16_t edge1;
int16_t edge2;
int16_t edge_neg;
int16_t edge_pos;
uint8_t edgeh;
uint8_t edgel;
int16_t freq_error;
int16_t freq_error_sum;
int8_t	ac_count = 10;
uint8_t phase_lock;
uint8_t ac_good;
uint8_t sweep;
uint8_t down;
uint8_t pp_track;
uint8_t fault;
int16_t freq;
int16_t freq_dsp;
int32_t adc_result;
int16_t prev;
int32_t adc_sum;
int16_t watts;
int16_t watts_prev;
int8_t inc = INC_HI;
int8_t  delta = INC_HI;
uint8_t grid_intertie;
int16_t dc_volts;
int16_t dc_amps;
int16_t watts_filt;
uint8_t keypress;
uint16_t ext_ser_in;
uint16_t timer1;
uint16_t ac_volts;
uint16_t tenth_seconds;
uint16_t seconds;
uint16_t prev_seconds;
uint16_t wait_fault_seconds;
uint16_t wait_seconds;
uint8_t pp_switch;
int16_t temperature;


//----------------------------------------------------------------------------------------------------
// This is fixed point FIR filter routine designed by WinFilter software. It is not used currently.
//
//WinFilter version 0.8
//http://www.winfilter.20m.com
//akundert@hotmail.com
//
//Filter type: Low Pass
//Filter model: Butterworth
//Filter order: 2
//Sampling Frequency: 100 Hz
//Cut Frequency: 2.000000 Hz
//Coefficents Quantization: 16-bit
//
//Z domain Zeros
//z = -1.000000 + j 0.000000
//z = -1.000000 + j 0.000000
//
//Z domain Poles
//z = 0.911347 + j -0.081409
//z = 0.911347 + j 0.081409
//----------------------------------------------------------------------------------------------------
/*
#define Ntap 15

#define DCgain 262144

int16_t fir(uint16_t NewSample) {
    uint16_t FIRCoef[Ntap] = { 
        11143,
        12796,
        14610,
        16594,
        18754,
        21093,
        23616,
        24924,
        23616,
        21093,
        18754,
        16594,
        14610,
        12796,
        11143
    };

    static uint16_t x[Ntap]; 			//input samples
    volatile uint32_t y;		            //output sample
    uint8_t n;
    									//shift the old samples
    for(n=Ntap-1; n>0; n--) {
       x[n] = x[n-1];
	}
    									//Calculate the new output
	y = 0;
    x[0] = NewSample;
    for(n=0; n<Ntap; n++) {
        y += (uint32_t)FIRCoef[n] * (uint32_t)x[n];
	}
    
    return((int16_t)(y / DCgain));
}
*/
//----------------------------------------------------------------------------------------------------
// This interrupt is the timer1 edge capture.
// It occurrs shortly before and shortly after zero crossing.
// Zero crossing is calculated as halfway between the two interrupts.
// The hardware is set up with two opto-isolators connected to the external AC power. They are set up
// so that one is asserted during the positive half of the AC waveform and the other one is asserted 
// during the negative half of the AC waveform. Each opto is connected to an input pin (B3 and B4) so 
// the processor can tell which half of the AC waveform is occuring. At the zero crossing point the LED in the
// opto won't come on until the AC voltage is at least 2V or 3V higher than zero (or lower in the case of
// the negative half of the AC waveform). So what you have is one opto going off right before the zero crossing
// and the other opto coming on right after the zero crossing. The two signals are ORed together and 
// connected to the timer input pin ICR1. Then the time it takes from the turn off before the zero crossing
// until the turn on after the zero crossing is measured by the processor. That time is divided by two
// and that gives you the time of the zero crossing.
// Timer1 is timing the length of 1 AC cycle which is 16.66ms or about 4166 counts on timer1 the way
// the timer is set up. So the times of the zero crossing are between 0 and 4166. 
// This routine calculates the time of edge1 and edge2 the which are the two zero crossing times. The times are 
// based on the time of the AC cycle which is nominally 4166 counts of timer1. The length of the cycle is stored
// in the variable comp1a_max. 
//
//----------------------------------------------------------------------------------------------------
ISR(TIMER1_CAPT_vect) {

	edgel = ICR1L;
	edgeh = ICR1H;												//read in both halves of the timer register
	edge_temp = (edgeh << 8) | edgel;							//get 16 bit timer value in edge_temp
	if (bit_is_clear(TCCR1B,ICES1)) {							//check bit to see if we are reading negative edge of zero cross
   		_delay_loop_2(4*60);									//30us (range 1uS to 16384uS)
		if (bit_is_clear(PINB,PINB6)) edge_neg = edge_temp;		//check bit for debouncing, if low then copy time for neg edge
		else return;											//else if not low then input is bouncing so jump out.
		TCCR1B |= _BV(ICES1);									//set ICES1 bit to detect positive edge for the next interrupt
		TIFR1 |= _BV(ICF1);										//clear the Input Capture Flag for the next interrupt
	}
	else if (bit_is_set(TCCR1B,ICES1)) {						//else check bit to see if we are reading positive edge of zero cross
   		_delay_loop_2(4*60);									//30us (range 1uS to 16384uS)
		if (bit_is_set(PINB,PINB6)) edge_pos = edge_temp;		//check bit for debouncing if set then copy time for positeve edge
		else return;											//if not then bouncing so jump out
		TCCR1B &= ~_BV(ICES1);									//clear ICES1 bit to detect negative edge for next interrupt
		TIFR1 |= _BV(ICF1);										//clear the Input Capture Flag for the next interrupt	
		if (bit_is_set(PINB,PINB4)) {							//check to see if begining of negative AC wave is called edge1
			if ((edge_pos - edge_neg) >= 0) {					//make sure edge_pos comes after edge_neg
				edge1 = ((edge_pos - edge_neg) / 2) + edge_neg;	//calculate time of zero cross by dividing time between edges by two
			}													//and adding it the negative edge to get the time in the cycle
			else {												//else the zero cross ocurs during the rollover of 4166 back to zero
				edge1 = (((comp1a_max - edge_neg) + edge_pos) / 2) + edge_neg;	//so take care of it (comp1a_max is the length of the cycle = 4166)
				if (edge1 > comp1a_max) edge1 = edge1 - comp1a_max; 
			}
		}
		else if (bit_is_set(PINB,PINB3)) {						//check to see if it is the begining of the positive AC waveform called edge2
			if ((edge_pos - edge_neg) >= 0) {					//make sure edge_pos comes after edge_neg
				edge2 = ((edge_pos - edge_neg) / 2) + edge_neg;	//calculate time of zero cross by dividing time between edges by two
			}													//and adding it the negative edge to get the time in the cycle (0-4166)
			else {												//else the zero cross ocurs during the rollover of 4166 back to zero
				edge2 = (((comp1a_max - edge_neg) + edge_pos) / 2) + edge_neg;	//so take care of it (comp1a_max is the length of the cycle = 4166)
				if (edge2 > comp1a_max) edge2 = edge2 - comp1a_max; 
			}
		}
	}
	ac_count = 10;												//this counter is set each time we get a zero cross
}																//if we stop having zero crossing then we time out and say no AC
//----------------------------------------------------------------------------------------------------
// This interrupt occurs once per every 60Hz AC cycle or once every 16.66ms.
// Timer1 is set up so that it is 4166 counts for one AC cycle or 16,66ms. This is the constant F60HZ, the actual value is comp1a_max.
// To keep the internal AC waveform synced up to the external AC waveform this count can change to change the length of internal AC
// cycle. I change the count using a software phase lock loop. The phase lock loop has a software PI controller in the feedback path.
// The PI controller (proportional and integral terms not using derivitive term of PID conroller) takes the error between the external
// and internal AC waves and calculates a new wave length (count for timer1) for the internal AC to keep it synced up to the external
// AC. So the even though the length of timer1 is nominally 4166 (F60HZ) the value in comp1a_max is constantly varying to keep the 
// internally generated AC waveform synced up to the external AC waveform. 

ISR(TIMER1_COMPA_vect) {

	static uint8_t count;
	static uint8_t count_temp;
	static uint8_t count_sec;

	comp1b_prev = comp1b_inc;
	OCR1B = comp1b_inc;
   	index1 = 32;
    polarity = NEG;
	
	if (grid_intertie == TRUE) {							// this flag is true if you want the internal AC to sync to the external AC				
		TIMSK1 |= _BV(ICIE1);								//enable capture interrupt 
		if (ac_count-- <= 0) {								//ac_count if set to 10 by zero cross interrupt if it is zero then
			ac_count = 0;									//10 interal AC cycles have passed with an external zero cross so 
			ac_good = FALSE;								//that means no external AC so ac_good flag is false 
			phase_lock = FALSE;								//if no external AC can't sync to it so phase_lock is false
			comp1a_max = F60HZ;								//set comp1a_max to default F60HZ counter value for timer1
		}
		else {
			ac_good = TRUE;									//else if ac_count hasn't timed out then we are getting zero crossing so ac_good is true
			if (edge2 <= FSETPOINT) {						//so now calculate the error which is the difference between the setpoint and the actual value
				freq_error = FSETPOINT2 - edge2;			//edge2 is the time of between the end of the negative AC wave and the begining of	
			}												//of the positive AC wave, so we call this the beginning of the external AC wave so we
			else {											//make the setpoint FSETPOINT2 = 0 which means it should edge2 should occur 
				freq_error = (comp1a_max - edge2) + FSETPOINT2; //around the 0 count of timer1
			}												//so we calculate freq_error as the difference between edge2 and FSETPOINT2	
															//making sure we take into account the rollover of timer1 from comp1a_max to zero
			if (freq_error > 1000) {												
				freq_error = 1000;							//this just sets limits on the error
				phase_lock = FALSE;							//so the error can't get bigger than 1000 or -1000
			}												//if the error is this big we are not in sync so phase_lock is false
			else if (freq_error < -1000) {
				freq_error = -1000;
				phase_lock = FALSE;
			}
			else {
				phase_lock = TRUE;							//if the error is in range then we are synced so phase_lock is true
			}
															//now we calculate the P and I terms to adjust the count.
			freq_error_sum += freq_error;					//to calculate the I term we need the sum of the errors
			if (freq_error_sum > 30000) freq_error_sum = 30000; //to avoid intergral windup we need to limit the error sum
			else if (freq_error_sum < -30000) freq_error_sum = -30000; //so we limit it between -30000 and 30000

			comp1a_max = F60HZ - (freq_error / 2) - (freq_error_sum / 256); //using the P term (freq_error / 2) and the I term 
																			//(freq_error_sum / 256) we adjust the default timer1 value 
		}																	// of 4166 (F60HZ) to minimize the error and make the external zero crossing
																			// edge2 line up with the setpoint or 0 count of timer1
	}
	else {													//else if we are not syncing to the external AC waveform
		TIMSK1 &= ~_BV(ICIE1);								//disable capture interrupt 
		ac_good = FALSE;									//then no external AC
		phase_lock = FALSE;									//we are not phase locked
		comp1a_max = F60HZ;									//set comp1a_max to the default
	}

	freq = FREQ_SCALE / (comp1a_max * USEC_INC);			//calculate the frequency in hertz from the timer1 maximum count comp1a_max
	OCR1A = comp1a_max;										//write the timer1 count com1a_max out to the timer register

	if (ac_good) {											//if we have external AC
		freq_dsp = (freq + 50) / 100;						//round off the frequency to be displayed in hertz
		ac_volts = ((uint32_t)ext_ser_in * 100) / 393;		//do scaling on ac volts, no decimal places
	}
	else { 													//if we don't have external AC then zero out 
		freq_dsp = 0;										//frequency to display and ac volts to display
		ac_volts = 0;	
	}
															//now we calculate the adc results, the adc is read and summed 64 times during each AC cycle
	adc_result = adc_sum / 64;								//so we divide the adc result by 64 to give us our average adc result
	adc_sum = 0;											//zero the adc sum for next time around
    switch (adc_type) {										//adc result is rotated amoung three readings, the dc volts, dc amps and temperature
        case AMP:											//adc_results is dc amps of solar panel this time
			dc_amps = (int16_t)(((adc_result - 20) * 100) / 40);	//do scaling for dc amps, divide this by 100 to get two decimal places
			adc_type = VOLT;								//going to read dc volts next time
			ADMUX = _BV(REFS0) | _BV(MUX1);					//set ADC/DAC external reference Vcc, set mux to ADC2 for dc volts for next time
            break;
        case VOLT:											//adc_results is dc volts this time
			dc_volts = (int16_t)((adc_result * 1000) / 493);//do scaling for dc volts, divide this by 100 to get two decimal places
			adc_type = TEMP;								//going to read the temperature of the heatsink next time
			ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX1);		//set ADC/DAC external reference Vcc, set mux to ADC10 to read temperature next time
            break;
        case TEMP:											//adc_results is temperature this time
			temperature = (int16_t)(((((((adc_result*100)/102)*5)*9)/5)+320)/10); //do scaling for temperature convert to degrees F
			adc_type = AMP;									//going to read dc amps of the solar panel next time
			ADMUX = _BV(REFS0) | _BV(MUX2) | _BV(MUX1);		//set ADC/DAC external reference Vcc, set mux to ADC6 to read dc amps next time
            break;
    }
	watts = (uint16_t)(((int32_t)dc_amps * (int32_t)dc_volts) / 100);	//calculate input dc watts of the solar panel by multiplying 
															//dc volts by dc amps, do this with a 32 bit multiply but cast it to a 16 bit
															//result, this is scaled by 100 so divide by 100 to get two decimal places
	
	if (++count >= 6) {										//this section does the MPPT algorithm 10 times a second
		tenth_seconds++;									//up the tenth second counter
		count = 0;
		if(state == PP_TRACK) {								//if we are in the peak power tracking state do the MPPT algorithm
			if(watts < watts_prev) delta = -delta;			//if the current solar panel watts are less than the previously measure watts
			update_adj(delta);								//change the sign of the update value and call the routine to update the transfer
			watts_prev = watts;								//ratio of the dc/dc connverter, save the current watts as the previous watts
		}
	}

	if (++count_sec >= 60) {
		seconds++;											//update the seconds counter
		count_sec = 0;
	}
}
//----------------------------------------------------------------------------------------------------
// This timer interrupt occurs 64 times in each 16.66ms sine wave cycle or once every 260uS.
// It uses a value from a sinewave table to update the PWM output to create both halves of the sinewave
// in 16.66ms or 60Hz. Timer1 is still running at a count of 4166 for 16.66ms so this COMPB interrupt divides
// 4166 into 64 pieces so comp1b_inc = 65. This is added on every interrupt to the time in the 4166 counter for 
// the next interrupt. 
//
//----------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPB_vect) {

	uint16_t temp16;
	uint8_t templ, temph;

	temp16 = comp1b_prev + comp1b_inc;						//add on the increment to get the value of the next interrupt in the 4166 timer1 count
	OCR1B = temp16;											//write the next timer value to interrupt to OCR1B timer register
	comp1b_prev = temp16;							 		//keep track for next interrupt

    switch (polarity) {										//divide the sinewave into positive and negative halves 32 pieces each
        case POS:
            if (index1 >= 32) {								//if we've done 32 interrupt then switch to negative half
                index1 = 0;									//zero index to pwm table	
                polarity = NEG;								//switch polarity to negative
   				PORTD |= _BV(PORTD7);						//change polarity of output bridge MOSFET to switch proper polarity of 
				PORTD &= ~_BV(PORTD6);       				//sinewave to output.
			}
            break;
        case NEG:
            if (index1 >= 32) {								//if we've done 32 interrupt then switch to positive half
                index1 = 0;									//zero index to pwm table
                polarity = POS;								//switch polarity to negative
    			PORTD |= _BV(PORTD6);						//change polarity of output bridge MOSFET to switch proper polarity of
	   			PORTD &= ~_BV(PORTD7);     					//sinewave to output  
            }
            break;
    }

	pwm_temp = pwmtab[index1];	 					// get pwm value from sinewave table 
	temp32 = (uint32_t)pwm_temp * (uint32_t)adj16;	// multiply adj16 by pwm table value to vary the overall magnitude of sinewave
	pwm = (uint16_t)(temp32 >> 8);					// adj16 varies 0-1024 so divide by 256 to get back to correct range
	index1++;										// increment the index for the next sinewave table value

													//set the values for the Power Stage Controller that does actually PWM outputs

	OCR2SA = 0x0400 - pwm - 2 - 50;				 	//set time A PSC2
	OCR2RA = pwm + 50;						 		//reset time A PSC2
	OCR2SB = 0x0400 - pwm - 2 - 50;			  		//set time B PSC2
	OCR2RB = pwm + 50;							 	//reset time B end of cycle PSC2

	OCR1SA = 0x0000 + (0x0400 - pwm); 				//set time A PSC1
	OCR1RA = 0x0400 + (0x0400 - pwm); 				//reset time A PSC1
	OCR1SB = 0x0400; 				  				//set time B PSC1
	OCR1RB = 0x0800-1; 				  				//reset time B end of cycle PSC1

	OCR0SA = 0x0000 + (0x0400 - pwm);				//set time A PSC0
	OCR0RA = 0x0400 + (0x0400 - pwm);				//reset time A PSC0
	OCR0SB = 0x0400;								//set time B PSC0
	OCR0RB = 0x0800-1;								//reset time B end of cycle PSC0

	templ = ADCL;
	temph = ADCH;
	temp16 = (temph << 8) | templ;					//get the 16 bit raw value from the adc registers
	adc_sum += (int32_t)(temp16);					//add the raw adc values to the adc_sum this will divided to get the average		
	ADCSRA |= _BV(ADSC);							//start next conversion

}
//----------------------------------------------------------------------------------------------------
// This interrupt routine occurs for each bit that is recieved from AC voltage micro.
// It uses the external INT1 interrupt on PB2.
// It receives start bit plus 10 data bits and a stop bit.
// The AC voltage is read by a seperate 8 bit micro a ATtiny45. The value of the AC voltage is sent serially
// over a single line. This is sent through an opto-isolator so the low voltage main processor is isolated from
// the high voltage AC. 
//
//----------------------------------------------------------------------------------------------------
ISR(INT1_vect) {

	static uint16_t ser_in, prev_ser_in;
	static uint8_t prefix = 0x00;
	static uint8_t stop_bit = 0x01;
	static uint8_t i;
	static uint16_t prev_timer;
	uint16_t current_timer;
	uint8_t temp8;
	uint16_t temp;

	current_timer = TCNT1;							//get the raw timer value from TCNT1
	timer1 = current_timer - prev_timer;			//find time since the last external interrupt
	prev_timer = current_timer;

	if (timer1 > 100) {								//if timer difference is large then it was a long time since the last bit
		i = 0;										//so this must be the first bit so reset all the counters
		ser_in = 0;
		prefix = 0x00;
	}

	if (i < 1) {									//if index i is zero then it is first bit
		i++;
		temp8 = (PINB & _BV(PINB1)) >> 1;			//get input bit in d0	
		temp8 = temp8 ^ (prefix & 0x01);			//check start bit is zero
		if (temp8) {								//if not zero then reset everything
			i = 0;									
			ser_in = 0;								//clear all the counters
			prefix = 0x00;
		}
		else {										//this is left over from more than one start bit
			prefix = prefix >> 1;
		}
	}
	else if ((i >= 1) && (i <= 10)) {				//if index is between 1 and 10 then read in rest of bits
		i++;
		if (bit_is_clear(PINB,PINB1)) temp = 0x0000;
		else temp = 0x0200;
		ser_in = ser_in >> 1;						//read in data bits
		ser_in = ser_in | temp;						//shift into ser_in
	}
	else if (i==11) {								//if bit number 11 check for stop bit
		temp8 = (PINB & _BV(PINB1)) >> 1;
		temp8 = temp8 ^ (stop_bit & 0x01);			//check stop bit
		if (!temp8) {								//if stop bit is good then copy data
			if (ser_in == prev_ser_in) ext_ser_in = ser_in;	//for error checking the same value is sent twice
			prev_ser_in = ser_in;					//so compare current value to previous value and only copy them if they are the same
		}
		i = 0;
		ser_in = 0;										//clear everthing for next data
		prefix = 0x00;
	}
}
//----------------------------------------------------------------------------------------------------
/* lowlevel serial port driver routine */
int uart_putchar(unsigned char c)
{
   loop_until_bit_is_set(UCSRA, UDRE);
   UDR = c;
   return 0;
}
/*--------------------------------------------------------------------*/
/* lowlevel serial port driver routine */
int uart_getchar(void)
{
while (!(UCSRA & (0x01 << RXC)));
return UDR;
}
/*--------------------------------------------------------------------*/
/* lowlevel serial port driver routine */
uint8_t kbhit(void) {

    uint8_t temp;

	temp = UCSRA & _BV(RXC);			//read in and mask off correct bit
	if (temp == 0) return(FALSE);		//if bit was zero return FALSE
	else return(TRUE); 	
}
/*---------------------------------------------------------------------*/
//	This is the setup for the Power Stage Controllers.
//  I'm using PSC0 in one ramp mode to generate the phase shifted control
//	for the full-bridge DC/AC converter. PSC0 uses PSCOUT00 and PSCOUT01 to
//	drive the two IR2104s. In one ramp mode OCR0RB = 0x0200 to set the length 
//	of the ramp which is the waveform peroid. 
//	I'm using PSC2 in 4 ramp mode to generate the signal for the active rectifier
//	that flips the high voltage transformer output into a positive waveform. PSC2
//	uses PSCOUT20 and PSCOUT21 that get optically isolated before driving IR2104s.
//	In 4 ramp mode OCR2RB does not set the signal period but the sum of all 4 ramps 
//	OCR2SA + OSCR2RA + OCR2SB + OCR2RB equals the period length. Also PSC1 and PSC2 
// 	are sycned to start at the same time as PSC0.
//	I was using PSC1 and PSC2 to generate the rectifier signals but I figured out 
//	how to use just PSC2 so PSC1 is not being used for anything now.  
//
void ioinit (void) 
{
 	DDRB = _BV(DDB7);					//set MOSI(PB1) and SCK(PB7) as output,
    DDRE = _BV(PORTE1) | _BV(PORTE2);	//enable PE1 and PE2 as outputs to blink LEDs 
    PORTE |= _BV(PORTE1);				//turn on green led 
	PORTC |= _BV(PORTC1) | _BV(PORTC7) | _BV(PORTC4) | _BV(PORTC5);	//set pullups on PC1 PC7	
    																//disable UF_EN and RC_EN by PC4 and PC5 high 
	DDRC = _BV(DDC2) | _BV(DDC3) | _BV(DDC4) | _BV(DDC5);	//set PC2 PC3 PC4 PC5 to outputs
	DDRD = _BV(PORTD6) | _BV(PORTD7)| _BV(DDD3) | _BV(DDD2) | _BV(DDD1) | _BV(DDD0);	//set PD6 PD7 PD3 PD2 PD1 PD0 to outputs
	PORTD = _BV(PORTD1); 				// this turns off the relay      
	
	comp1a_max = F60HZ;					//set up variables for timer1 this is length of one AC cycle 4166 counts for 16.66ms
	comp1b_inc = F60HZ64;				//divide 4116 into 64 pieces this is equals 65
	comp1b_prev = comp1b_inc;			//
										//do not toggle outputs, pins used as general i/o
	TCCR1B = _BV(ICNC1) | _BV(WGM12) | _BV(CS01) | _BV(CS00);
										//set input capture noise filter on and set input capture to falling edge	
										//timer1 mode 4 CTC with OCR1A setting TOP and clk/64 prescale
	OCR1A = comp1a_max;					//set TOP to be 16.6ms or one period of 60Hz AC 
	OCR1B = comp1b_inc;					//divide this by 64 to tell when to update sinewave 
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);	
										//enable timer1 compare A and compare B interrupts and capture external event interrupt
	GTCCR = _BV(ICPSEL1);				// set ICP1B input capture pin B for timer1	
										// error in io90pwm3b.h file, corrected to match data sheet 
	EICRA = _BV(ISC11) | _BV(ISC10);	// enable external interrupt 1 for rising edge triggered
	EIMSK = _BV(INT1);					// enable external interrupt 1 on pin 20 PB2

	ADMUX = _BV(REFS0) | _BV(MUX2) | _BV(MUX1);					//set ADC/DAC external reference Vcc, set mux to ADC6
	ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1); 	// enable A/D, start conversion, prescaler divide by 64
	DIDR0 = _BV(ADC6D);											// disable input register on ADC6
	_delay_loop_2(4*100);				//100us (range 1uS to 16384uS) delay to setup mux

	PLLCSR = _BV(PLLF);					//set CLK PLL to 64Mhz
	PSOC2 = _BV(POEN2D);				//enable PSCOUT23
	OCR2SA = 0x0400-2;					//set time A
	OCR2RA = 0x0000;					//reset time A
	OCR2SB = 0x0400-2;					//set time B
	OCR2RB = 0x0000;					//reset time B end of cycle
	PCNF2 = _BV(PALOCK2) | _BV(POP2) | _BV(PMODE21) | _BV(POME2) | _BV(PCLKSEL2);	
										//set autolock on, output polarity high, four ramp mode, output matrix enabled, fast clock
	POM2 = 0x55;						//set output matrix so PSCOUT20 goes high in ramp 1 and PSCOUT21 goes low in ramp 1
										//disable PSC1 outputs because we're using PSC0
	//PSOC1 = _BV(POEN1A) | _BV(POEN1B);	//enable PSCOUT10 and PSCOUT11
	OCR1SA = 0x0000;					//set time A
	OCR1RA = 0x0400;					//reset time A
	OCR1SB = 0x0400;					//set time B
	OCR1RB = 0x0800-1;					//reset time B end of cycle
	PCNF1 = _BV(PALOCK1) | _BV(POP1) | _BV(PCLKSEL1);	//set autolock on, output polarity high, one ramp mode, fast clock
	PCTL1 = _BV(PARUN1);				//sync PSC1 to start when PSC0 starts

	PSOC0 = _BV(POEN0A) | _BV(POEN0B);	//enable PSCOUT00 and PSCOUT01
	OCR0SA = 0x0000;					//set time A
	OCR0RA = 0x0400;					//reset time A
	OCR0SB = 0x0400;					//set time B
	OCR0RB = 0x0800-1;					//reset time B end of cycle
	PCNF0 = _BV(PALOCK0) | _BV(POP0) | _BV(PCLKSEL0);	//set autolock on, output polarity high, one ramp mode, fast clock
	PCTL0 = _BV(PARUN0);				//sync PSC0 to start when PSC2 starts

	PCTL2 = _BV(PRUN2);					//start PSC2 				

	
    
    bps = BAUD9600;						/* enable UART default setting after reset is 8N1-format */
    UBRRH = (unsigned char)(bps>>8);
    UBRRL = (unsigned char)bps;
    UCSRB = _BV(RXEN) | _BV(TXEN);			    	// enable UART TX and RX

    stdout = fdevopen((FILE *)uart_putchar, (FILE *)uart_getchar);
  
	//printf("\r\n");
	//printf("\r\n");
	//printf("%s %s\r\n", START_STR,VERS);
	//printf("\r\n");

    sei ();								//enable interrupts 

	_delay_loop_2(4*1000);				//1000us (range 1uS to 16384uS) delay before enabling mosfet drivers

    																 
}
/*
//--------------------------------------------------------------------------------------------------------------------------
void turn_on(void) {

	PORTC &= ~(_BV(PORTC4) | _BV(PORTC5)); 
	PSOC0 = _BV(POEN0A) | _BV(POEN0B); 
	pp_switch = ON; 	

}
//--------------------------------------------------------------------------------------------------------------------------
void turn_off(void) {

	PORTC |= (_BV(PORTC4) | _BV(PORTC5)); 
	PSOC0 = ~(_BV(POEN0A) | _BV(POEN0B)); 
	pp_switch = OFF;

}
*/
//--------------------------------------------------------------------------------------------------------------------------
// This routine checks the system values and returns a fault number if any of them are of range.

uint8_t check_faults(void) {

	if(pp_switch == ON) {
		if(dc_amps > FAULT_MAX_DC_AMPS) fault = 2;
		//else if(dc_amps < FAULT_MIN_DC_AMPS) fault = 3;
		else if(dc_volts < FAULT_MIN_DC_VOLTS) fault = 4;
		else if(dc_volts > FAULT_MAX_DC_VOLTS) fault = 5;
		else if(!ac_good) fault = 6;
		else if(!phase_lock) fault = 7;
		else if (freq_dsp > MAX_FREQ) fault = 8;
		else if (freq_dsp < MIN_FREQ) fault = 9;
		//else if (ac_volts > MAX_AC_VOLTS) fault = 10;
		//else if (ac_volts < MIN_AC_VOLTS) fault = 11;
		else fault = 0;
		wait_fault_seconds = seconds + FAULT_SECONDS;	
	}
	if (fault) PORTD |= _BV(PORTD2);		// turn on red led if fault
	else PORTD &= ~_BV(PORTD2);				// turn off red led if no fault

	return(fault);
}
//--------------------------------------------------------------------------------------------------------------------------
// The varialbe adj16 controls the overall magnitude of the output AC sinewave.
// This rountine just check that it stays within limit when you adjust it.
void update_adj(int16_t adjust) {

	if(adj16 > MAX_ADJ16) adj16 = MAX_ADJ16;
	else if(adj16 < MIN_ADJ16) adj16 = MIN_ADJ16;
	else adj16 += adjust;
}
//--------------------------------------------------------------------------------------------------------------------------
// This routine sends the state and system data out to the serial port.
//
void print_results(void) {

	if (state == WAIT) printf("Wait ");
	else if (state == NIL) printf("Nil  ");
	else if (state == PP_ON) printf("On   ");
	else if (state == PP_TRACK) printf("Track");
	else if (state == PP_SWEEP) printf("Sweep");
	else if (state == PP_DOWN) printf("Down ");
	else if (state == PP_OFF) printf("Off  ");
	else if (state == PP_FAULT) printf("Fault %d", fault);
	else if (state == SYNC) printf("Sync ");
	printf("- %u %d %d %d %d %d %d %d\r\n", seconds, adj16, dc_amps, dc_volts, watts, ac_volts, freq_dsp, temperature); 

}
//-------------------------------------------------------------------------------------------------------------------------
int main (void)  {        

	uint16_t led_count = 0;
	uint8_t wait_state_count = 0;

    ioinit();
	state = WAIT;
	fault = 0;
	TURN_OFF;								

    do {											/* loop forever */
		if (++led_count & 0x8000) {					//blink green led
      	    PORTE |= _BV(PORTE2);
			}
      	else {
        	PORTE &= ~_BV(PORTE2);
		}

    	if (kbhit()) {
				keypress = uart_getchar();			// get it in keypress
		}

 		if(seconds != prev_seconds) {				//once every second check the system state

	   		switch(state) {
    		case WAIT :								//start in wait state
				adj16 = ADJ16_START;
				if (wait_state_count-- > 0) {
					state = WAIT;					//stay in wait state for 5 seconds
				}
				else if (dc_volts > 1400) {			//if the solar panel dc volts is high enough (daytime)
					state = SYNC;					//move to sync state  
					grid_intertie = TRUE;			//sync to external AC
					TURN_ON;						//turn on MOSFET drivers
				}
				else {
					state = WAIT; 					//else if not daytime yet go back to wait state
				}
	       		break;
    		case SYNC :								
				if (check_faults()) {				//check for faults
					state = PP_FAULT;				//and go to fault state if there is faults
					TURN_OFF;						//turn off MOSFET drivers
				}
				else if (ac_good & phase_lock) {	//else if we have external ac and sycned up 
					state = PP_TRACK;				//then go to tracking state
					PORTD &= ~_BV(PORTD1);       	//relay ON flipped because using hc14 for extra current to drive relay
				}
				else { 
					state = SYNC; 					//else go back to sync
				}
	        	break;
    		case PP_ON :				
				if (check_faults()) {				//check for faults
					state = PP_FAULT;				//go to fault state if faults
					TURN_OFF;						//turn off MOSFET drivers
				}									//else go to state depending on keypress input from serial port
				else if (keypress == 'i') {			
					update_adj(ADJ_INC_POS);
				}
   				else if (keypress == 'j') {
					update_adj(ADJ_INC_NEG);
				}
   				else if (keypress == 's') {
					state = PP_SWEEP;
				}
    			else if (keypress == 'd') {
					state = PP_DOWN;
				}
   				else if (keypress == 'a') {
					state = PP_TRACK;
				}
   				else if (keypress == 'z') {
					state = PP_OFF;
				}
				else {
					state = PP_ON;
				}
				break;
	    	case PP_TRACK :									// the track function is taken care of in the 60hz interrupt to get a faster update rate				
				if (check_faults()) {
					state = PP_FAULT;
					TURN_OFF;								//turn off MOSFET drivers
				}
		    	else if (keypress) {
					state = PP_ON; 
				}
				else {
					state = PP_TRACK;
				}
    	    	break;
    		case PP_DOWN :				
				update_adj(-INC_HI);						//in PP_DOWN state sweep the overall adjust value down 
				if (check_faults()) {						//this is for debugging purposes
					state = PP_FAULT;
					TURN_OFF;								//turn off MOSFET drivers
				}
	    		else if (keypress) {
					state = PP_ON; 
				}
				else {
					state = PP_DOWN;
				}
        		break;
	    	case PP_SWEEP :									//in PP_SWEEP state sweep the overall adjust value UP 
				update_adj(INC_HI);							//this is for debugging purposes
				if (check_faults()) {
					state = PP_FAULT;
					TURN_OFF;								//turn off MOSFET drivers
				}
		    	else if (keypress) {
					state = PP_ON; 
				}
				else {
					state = PP_SWEEP;
				}
    	    	break;
    		case PP_OFF :				
				TURN_OFF;								//turn off MOSFET drivers
				PORTD |= _BV(PORTD1);					//relay = OFF;
		    	if (keypress) {
					state = WAIT; 
					wait_state_count = 5; 
				}
				else {
					state = PP_OFF;
				}
        		break;
	    	case PP_FAULT :				
				TURN_OFF;								//turn off MOSFET drivers
				PORTD |= _BV(PORTD1);					//relay = OFF;			
				state = WAIT;
				wait_state_count = 5; 
				fault = FALSE;
        		break;
			default :								// keys to make them lower case so this makes	
    		break;									// ESC into 0x3B instead of 0x1B
			}  	//end case

			print_results();						//print output
			prev_seconds = seconds;
			keypress = 0;
		}


    } while(1);

    return (0);
}

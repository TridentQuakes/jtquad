/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "receiver.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	double receiver_transmit_factor = 						0;
	int receiver_data[NUMBER_OF_CHANNELS] = 				{0, 0, 0, 0, 0, 0, 0};
	int receiver_command[NUMBER_OF_CHANNELS] = 				{0, 0, 0, 0, 0, 0, 0};
	int receiver_command_smooth[NUMBER_OF_CHANNELS] = 		{0, 0, 0, 0, 0, 0, 0};
	int receiver_zero[3] = 									{0, 0, 0};
	double receiver_slope[NUMBER_OF_CHANNELS] = 			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double receiver_offset[NUMBER_OF_CHANNELS] = 			{0};//see ds_initialize_eeprom_defaults //{-30, -27, -29.5, -29, 0.0, 0.0, 0.0}; // calculated by hand offset=((max + min)/2)
	double receiver_smooth_factor[NUMBER_OF_CHANNELS] =		{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	volatile uint16_t pulse_width[NUMBER_OF_CHANNELS] =		{0, 0, 0, 0, 0, 0, 0};
	//uint8_t channel_to_pin_map[7] = 						{1, 2, 3, 0, 4, 5, 6}; 	// {XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX1, AUX2}
	
	// temporary because of ttl converter was partially broken (enable the one above later)
	uint8_t channel_to_pin_map[7] = 						{2, 3, 6, 0, 1, 4, 5}; 	// {XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX1, AUX2}
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize receiver
   ---------------------------------------------------------------------------- */
void receiver_initialize(void)
{
	receiver_command[XAXIS] = 		1500;
	receiver_command[YAXIS] = 		1500;
	receiver_command[ZAXIS] = 		1500;
	receiver_command[THROTTLE] = 	1000;
	receiver_command[MODE] = 		1000;
	receiver_command[AUX1] = 		1000;
	receiver_command[AUX2] = 		1000;
	
	for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{
		receiver_command_smooth[channel] = 1;
	}
	
	for (uint8_t channel = XAXIS; channel < THROTTLE; channel++)
	{
		receiver_zero[channel] = 1500;
	}

	for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{
		receiver_slope[channel] = 1;
	}
	
	for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{
		receiver_offset[channel] = 0;
	}
	
	for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{
		receiver_smooth_factor[channel] = 1; 
	}
	
	initialize_input_capture();
}

/* ----------------------------------------------------------------------------
   Initialize input capture for all 7 channels
   ---------------------------------------------------------------------------- */
void initialize_input_capture(void)
{
	// Set pins as inputs
	// 	(ch0, ch1, ch2, ch3, ch4, ch5, ch6) 
	// 	(PA0, PA1, PA2, PA3, PB0, PB1, PB2)
	// *** ch3, ch4, ch7 are only ones working now because of ttl ***
	//PORT_SetPinsAsInput(&PORTA, 0b00001100);
	//PORT_SetPinsAsInput(&PORTB, 0b00000100);
	PORT_SetPinsAsInput(&PORTA, PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm);
	PORT_SetPinsAsInput(&PORTB, PIN0_bm | PIN1_bm | PIN2_bm);
	
	// Input sense both edges
	//PORTA.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
	//PORTA.PIN3CTRL = PORT_ISC_BOTHEDGES_gc;
	//PORTB.PIN2CTRL = PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN0CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN1CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN2CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN0CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN1CTRL |= PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN2CTRL |= PORT_ISC_BOTHEDGES_gc;
	
	// Set pins to channels
	//EVSYS.CH0MUX = EVSYS_CHMUX_PORTA_PIN2_gc;
	//EVSYS.CH1MUX = EVSYS_CHMUX_PORTA_PIN3_gc;
	//EVSYS.CH2MUX = EVSYS_CHMUX_PORTB_PIN2_gc;
	EVSYS_SetEventSource( 0, EVSYS_CHMUX_PORTA_PIN0_gc );
	EVSYS_SetEventSource( 1, EVSYS_CHMUX_PORTA_PIN1_gc );
	EVSYS_SetEventSource( 2, EVSYS_CHMUX_PORTA_PIN2_gc );
	EVSYS_SetEventSource( 3, EVSYS_CHMUX_PORTA_PIN3_gc );
	EVSYS_SetEventSource( 4, EVSYS_CHMUX_PORTB_PIN0_gc );
	EVSYS_SetEventSource( 5, EVSYS_CHMUX_PORTB_PIN1_gc );
	EVSYS_SetEventSource( 6, EVSYS_CHMUX_PORTB_PIN2_gc );
	
	// Enable input "capture or compare" channel A,B,C and set waveform to normal
	TC0_EnableCCChannels( &TCD0, TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm );
	TC0_EnableCCChannels( &TCE0, TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm );
	
	// Pulse width capture (bit 7:5), channel 0
	// NOTE: The selected event channel n will be the event channel source for CC channel A, 
	//       and event channel (n+1)%8, (n+2)%8, and (n+3)%8 will be the event channel source
	//       for CC channel B, C, and D.
	//TCF0.CTRLD = 0b11001000;
	TCD0.CTRLD = (uint8_t) TC_EVSEL_CH0_gc | TC_EVACT_PW_gc;
	TCE0.CTRLD = (uint8_t) TC_EVSEL_CH4_gc | TC_EVACT_PW_gc;
	
	// Set input capture channels interrupt levels to low
	//TCF0.INTCTRLB = 0b00010101;
	TC0_SetCCAIntLevel( &TCD0, TC_CCAINTLVL_LO_gc );
	TC0_SetCCBIntLevel( &TCD0, TC_CCBINTLVL_LO_gc );
	TC0_SetCCCIntLevel( &TCD0, TC_CCCINTLVL_LO_gc );
	TC0_SetCCDIntLevel( &TCD0, TC_CCDINTLVL_LO_gc );
	TC0_SetCCAIntLevel( &TCE0, TC_CCAINTLVL_LO_gc );
	TC0_SetCCBIntLevel( &TCE0, TC_CCBINTLVL_LO_gc );
	TC0_SetCCCIntLevel( &TCE0, TC_CCCINTLVL_LO_gc );
	
	// Set the timer prescalers to 1
	//TCF0.CTRLA = 0b00000001;
	TC0_ConfigClockSource( &TCD0, TC_CLKSEL_DIV1_gc );
	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_DIV1_gc );
	
	// NOTE: Here we are using pulse width capture mode (CTRLD). If using input capture mode then the CCA register's
	//       MSB is reserved for edge polarity (0 = falling, 1 = rising) and so we are left with 15 bits for the time.
	//
	//       e.g. To clear MSB of PER[H:L] to allow for propagation of edge polarity (i.e. 0111 1111 1111 1111):
	//              TC_SetPeriod( &TCF0, 0x7FFF );
	//
	//            To test edge polarity in ISR:
	//              this_capture = TC_GetCaptureA( &TCF0 );
	//              if ( this_capture & 0x8000 ) // rising edge
	//                // do something
	//              else // falling edge
	//                // do something
	//
	// See application note AVR1306 for an example of this.
}

/* ----------------------------------------------------------------------------
   ISR TCD0 CCA (CH0, Thro, THROTTLE)
   ---------------------------------------------------------------------------- */
ISR(TCD0_CCA_vect)
{
	pulse_width[0] = TCD0.CCA;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[0] = pulse_width[0] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCD0 CCB (CH1, Aile, XAXIS, Roll)
   ---------------------------------------------------------------------------- */
ISR(TCD0_CCB_vect)
{
	pulse_width[1] = TCD0.CCB;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[1] = pulse_width[1] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCD0 CCC (CH2, Elev, YAXIS, Pitch)
   ---------------------------------------------------------------------------- */
ISR(TCD0_CCC_vect)
{
	pulse_width[2] = TCD0.CCC;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[2] = pulse_width[2] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCD0 CCD (CH3, Rudd, ZAXIS, Yaw)
   ---------------------------------------------------------------------------- */
ISR(TCD0_CCD_vect)
{
	pulse_width[3] = TCD0.CCD;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[3] = pulse_width[3] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCE0 CCA (CH4, Gear, MODE)
   ---------------------------------------------------------------------------- */
ISR(TCE0_CCA_vect)
{
	pulse_width[4] = TCE0.CCA;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[4] = pulse_width[4] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCE0 CCB (CH5, Aux1, AUX1)
   ---------------------------------------------------------------------------- */
ISR(TCE0_CCB_vect)
{
	pulse_width[5] = TCE0.CCB;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[5] = pulse_width[5] >> 5;
}

/* ----------------------------------------------------------------------------
   ISR TCE0 CCC (CH6, Aux2, AUX2)
   ---------------------------------------------------------------------------- */
ISR(TCE0_CCC_vect)
{
	pulse_width[6] = TCE0.CCC;
	
	// Divide by 32 to convert to microseconds (i.e. F_CPU = 32MHZ)
	pulse_width[6] = pulse_width[6] >> 5;
}

/* ----------------------------------------------------------------------------
   Read receiver
   ---------------------------------------------------------------------------- */
void receiver_read(void)
{
	// ----------------------------------------------------------------------------------
	// STEP 1: Calibrate the raw receiver values and run through low pass filter (smooth)
	// ----------------------------------------------------------------------------------
	for(uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	{
		// STEP 1a: Calibrate receiver's raw value
		receiver_data[channel] = (receiver_slope[channel] * get_raw_channel_value(channel)) + receiver_offset[channel];
		
		// STEP 1b: Smooth the flight control receiver values
		receiver_command_smooth[channel] = filter_smooth(receiver_data[channel], receiver_command_smooth[channel], receiver_smooth_factor[channel]);
	}
	
	// ----------------------------------------------------------------------------------
	// STEP 2: Reduce receiver commands using receiver_transmit_factor and center around 1500
	// ----------------------------------------------------------------------------------
	for(uint8_t channel = XAXIS; channel < THROTTLE; channel++)
	{
		receiver_command[channel] = ((receiver_command_smooth[channel] - receiver_zero[channel]) * receiver_transmit_factor) + receiver_zero[channel];
	}
	
	// No receiver_transmit_factor reduction applied for throttle, mode and AUX
	for(uint8_t channel = THROTTLE; channel < NUMBER_OF_CHANNELS; channel++)
	{
		receiver_command[channel] = receiver_command_smooth[channel];
	}
}

/* ----------------------------------------------------------------------------
   Read raw channel value from receiver
   ---------------------------------------------------------------------------- */
uint16_t get_raw_channel_value(RECEIVER_CHANNELS_t channel)
{
	uint16_t temp_pulse_width;
	
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		temp_pulse_width = pulse_width[channel_to_pin_map[channel]];
	}
	
	return temp_pulse_width;
}

/* ----------------------------------------------------------------------------
   Low pass filter for raw receiver data
   ---------------------------------------------------------------------------- */
double filter_smooth(double currentData, double previousData, double smoothFactor) 
{
	// Only apply time compensated filter if smoothFactor is applied
	if (smoothFactor != 1.0)
	{
		return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
	}

	// If smoothFactor == 1.0, do not calculate, just bypass
	return currentData; 
}

/* ----------------------------------------------------------------------------
   Get receiver data in SI units (radians)
	- return the smoothed & scaled number of radians/sec in stick movement - zero centered
   ---------------------------------------------------------------------------- */
const double receiver_get_SI_data(RECEIVER_CHANNELS_t channel)
{
	return ((receiver_command[channel] - receiver_zero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
}

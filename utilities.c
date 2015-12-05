/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "utilities.h"

/* ================================================================================================
   Externs
   ================================================================================================ */
   
	extern volatile uint32_t timer_tcc1_overflow_count;
	//extern volatile uint32_t timer_tcd0_overflow_count;

/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Micros
   
   Returns the current time in microseconds. Uses 16 bit timer TCC1 for 
   operation. 1024us per overflow of TCC1.
   
   Notes on timer_tcc1_overflow_count: 
     - will reach max after 71.5 minutes
     - micros will work even when subtracting two values 
	   (value after overflow - value before overflow)
	 - it works as long as we are not allowing two periods of 71.5 minutes
	   to go by then it will fail
   ---------------------------------------------------------------------------- */
uint32_t micros(void)
{
	uint32_t m;
	uint16_t t;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		m = timer_tcc1_overflow_count;		// Read the overflow count
		t = TCC1_CNT;						// Read the timer register
		
		// Check if the timer overflowed while reading the timer register
		if( (TCC1_INTFLAGS & TC1_OVFIF_bm) && (t < 0x7fff) )
		{
			// If the timer overflowed increment the overflow count to what it should be now
			m++;
			
			// Read the timer register again. This read's value should be less that the one
			// read previously if done quickly since the timer overflowed.
			t = TCC1_CNT;
		}
	}
	
	/* NOTE: If the timer overflowed while it was disabled then the oveflow ISR will be called
       quickly at around this point

	   Multiply m by 1024 by shifting it left 10 positions. 1024us elapse per TCC1 overflow. 
	   So m is in microseconds. (i.e. m = (m << 10))
	
	   Divide t by 32 by shifting it right 5 positions. 32 is the number of clock cycles per 
	   microsecond when F_CPU runs at 32 MHZ. (i.e. t = (t >> 5))
	   
	   NOTE: If t is not a multiple of 32 then the remainder, t mod 32, will be truncated 
	   (not rounded). So there will be some loss in accuracy (less than 1us).
	*/

	return (m << 10) + (t >> 5);
}

/* ----------------------------------------------------------------------------
   Millis
   
   Returns the current time in milliseconds. Uses 16 bit timer TCD0 for 
   operation. 1ms per overflow of TCD0.
   
   Notes on timer_tcc1_overflow_count: 
     - will reach max after 1193.04 hours (49.71 days)
	 - micros will work even when subtracting two values 
	   (value after overflow - value before overflow)
	 - it works as long as we are not allowing two periods of 1193.04 hours 
	   to go by then it will fail
   ---------------------------------------------------------------------------- */
/*uint32_t millis(void)
{
	return timer_tcd0_overflow_count;
}*/






/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __LPF_H__
#define __LPF_H__

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "common.h"
	
/* ================================================================================================
   Defines
   ================================================================================================ */
	
	/* Notes
	
		The following [b, a] coefficients were generated using the Chebyshev type II filter. It is
		implemented in matlab. It is also implemented in signal processing toolboxes.
		
		[b, a] = cheby2(n, Rs, Wc)
		
		n		nth order low pass filter
		Rs		Rs DB stopband ripple
		Wc		normalized cutoff frequency (edge frequency)
		b		filter coefficients (row vector)
		a		filter coefficients (row vector)
		
		[b, a] = cheby2(4,60,12.5/50)
		
		4		4th order low pass filter
		60		60 DB (decibels) stopband ripple 
		12.5/60	normalized cutoff frequency
	
	*/
	#define _b0  0.001893594048567
	#define _b1 -0.002220262954039
	#define _b2  0.003389066536478
	#define _b3 -0.002220262954039
	#define _b4  0.001893594048567

	#define _a1 -3.362256889209355
	#define _a2  4.282608240117919
	#define _a3 -2.444765517272841
	#define _a4  0.527149895089809

/* ================================================================================================
   Function prototypes
   ================================================================================================ */
   
	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif

	// Initialization functions
	void init_fourth_oder(void);
	
	// Calculation functions
	void compute_fourth_order(void);
	
	// FPU functions
	void write_accel_init_fourth_order_function_to_fpu_eeprom(void);
	
	#if defined(__cplusplus)
	}
	#endif
	
#endif

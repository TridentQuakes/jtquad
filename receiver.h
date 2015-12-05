/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __RECEIVER_H__
#define __RECEIVER_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"

/* ================================================================================================
   Defines
   ================================================================================================ */
	
	#define MINCOMMAND 								1000
	#define MIDCOMMAND 								1500
	#define MAXCOMMAND 								2000
	#define MIN_THRESHOLD							(MINCOMMAND + 105)
	#define MAX_THRESHOLD							(MAXCOMMAND - 105)
	#define MINTHROTTLE 							(MINCOMMAND + 105	)
	
	#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Public functions
	void 			receiver_initialize(void);
	void 			receiver_read(void);
	const double 	receiver_get_SI_data(RECEIVER_CHANNELS_t channel);
	
	// Private functions
	uint16_t 		get_raw_channel_value(RECEIVER_CHANNELS_t channel);
	void 			initialize_input_capture(void);
	double 			filter_smooth(double currentData, double previousData, double smoothFactor);
	
	
	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif
		
#endif
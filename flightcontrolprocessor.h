/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef __FLIGHTCONTROLPROCESSOR_H__
#define __FLIGHTCONTROLPROCESSOR_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#define ATTITUDE_SCALING (0.75 * PWM2RAD)

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Public functions
	void process_flight_control(void);
	
	// Private functions
	void calculate_flight_error(void);
	void process_calibrate_esc(void);
	void process_throttle_correction(void);
	void process_hard_maneuvers(void);
	void process_min_max_motor_commands(void);
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef __FLIGHTCONTROLQUADX_H__
#define __FLIGHTCONTROLQUADX_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"

/* ================================================================================================
   Defines
   ================================================================================================ */
   
	#define FRONT_LEFT  MOTOR_0
	#define FRONT_RIGHT MOTOR_1
	#define REAR_RIGHT  MOTOR_2
	#define REAR_LEFT   MOTOR_3
	#define LASTMOTOR	(MOTOR_3 + 1)
	
	#ifdef CHANGE_YAW_DIRECTION
	  #define YAW_DIRECTION -1
	#else
	  #define YAW_DIRECTION 1
	#endif

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Public functions
	void calculate_motor_commands(void);
	
	// Private functions
	
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

 
#ifndef __FLIGHTCOMMANDPROCESSOR_H__
#define __FLIGHTCOMMANDPROCESSOR_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Public functions
	void 	read_pilot_commands(void);
	
	// Private functions
	void 	process_grounded_functions(void);
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _SERIALCOMMUNICATIONS_H_
#define _SERIALCOMMUNICATIONS_H_

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"
	
/* ================================================================================================
   Function Prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	int process_serial_commands(void);
	void send_serial_telemetry(void);
	
	bool validate_calibrate_esc_command(uint8_t command);
	void fool_aq_config(void);
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
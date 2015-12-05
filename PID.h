/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __PID_H__
#define __PID_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"

/* ================================================================================================
   ENUM and Struct
   ================================================================================================ */
   
enum {
  RATE_XAXIS_PID_IDX = 0,			// rate mode
  RATE_YAXIS_PID_IDX,				// rate mode
  ZAXIS_PID_IDX,					// rate and attitude modes
  ATTITUDE_XAXIS_PID_IDX,			// attittude mode (accel)
  ATTITUDE_YAXIS_PID_IDX,			// attittude mode (accel)
  HEADING_HOLD_PID_IDX,				// heading hold feature turned ON
  ATTITUDE_GYRO_XAXIS_PID_IDX,		// attitude mode (gyro)
  ATTITUDE_GYRO_YAXIS_PID_IDX,      // attitude mode (gyro)

  LAST_PID_IDX  // keep this
};

// PID Variables
struct PIDdata {
  double P, I, D;
  double lastError;
  // AKA experiments with PID
  double previousPIDTime;
  double integratedError;
  double windupGuard;
} PID[LAST_PID_IDX];

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	double wind_up_guard; // Read in from EEPROM

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Public functions
	double PID_update(double targetPosition, double currentPosition, struct PIDdata *PIDparameters);
	void zero_integral_error(void) __attribute__ ((noinline));
	void zero_integral_error(void) ;
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
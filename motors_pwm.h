/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __MOTORS_PWM_H__
#define __MOTORS_PWM_H__

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
	void motors_initialize(void);
	void motors_command(MOTORS_t motor, uint16_t command);
	void motors_command_all(uint16_t command);
	void motors_pulse(uint8_t number_of_pulses);
	void motors_write(void);
	void motors_set_all_motors_to_command(uint16_t command);
	void motors_set_motor_command(MOTORS_t motor, uint16_t command);
	void motors_set_motors_to_commands(uint16_t command_motor0, uint16_t command_motor1, uint16_t command_motor2, uint16_t command_motor3);
	
	// Private functions
	void initialize_hardware_pwm(void);
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _JTQUAD_H_
#define _JTQUAD_H_

/* ================================================================================================
   Includes
   ================================================================================================ */	
	#include "common.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	uint32_t current_time = 0;
	uint32_t previous_time = 0;
	uint32_t delta_time = 0;
	uint32_t previous_time_400hz = 0;
	uint32_t delta_time_400hz = 0;
	uint32_t previous_time_100hz_tasks = 0;
	uint32_t previous_time_50hz_tasks = 0;
	uint32_t previous_time_10hz_tasks = 0;
	uint32_t previous_time_10hz_tasks_low_priority_1 = 0;
	
	// Global value used to calculate delta time between sensor processing in tasks
	double G_Dt = 0.002;

	// Used to implement the micros()
	volatile uint32_t timer_tcc1_overflow_count = 0;
	
	// Main application control states (timing)
	volatile uint32_t timer_tcd1_overflow_count = 0;
	volatile uint8_t app_execute = 0;
	uint8_t app_execute_buf = 0;
	
	// Application states
	uint8_t 	motors_state = 			OFF;
	uint8_t 	flight_mode = 			RATE_FLIGHT_MODE;
	uint8_t 	previous_flight_mode = 	ATTITUDE_FLIGHT_MODE;
	bool 		in_flight = 			false;	// true when the motors are armed and the user passed the min throttle threshold once
	uint8_t 	transmitter_state = 	OFF;
	int			min_armed_throttle = 	1150;
	
	// Peripheral flags
	#ifdef HMC5883L
	uint8_t mag_flags = 0;
	#endif
	uint8_t accel_flags = 0;
	uint8_t gyro_flags = 0;
	uint8_t fpu_flags = 0;
	
	uint32_t vehicle_state = 0;
	
	#ifdef HMC5883L
	VectorDouble3 mag_offset;
	#endif
	VectorDouble3 accel_scale;

	// ESC calibration process
	uint8_t calibrate_esc = ESC_CALIBRATION_IDLE;
	uint16_t esc_calibration_test_command = MINCOMMAND;
	
	// Flight control
	double rotation_speed_factor = 1.0;
	int throttle = 1000;
	
	// Heading and heading hold global declaration section
	uint8_t 	heading_hold_config 					= ON;
	uint8_t		heading_hold_state						= OFF;
	double 		heading									= 0;
	double 		relative_heading 						= 0;
	double		heading_hold							= 0;
	uint32_t 	heading_time							= 0;
	
/* ================================================================================================
   Function Prototypes
   ================================================================================================ */
   
	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif

	void initialize_clock_32mhz(void);
	void initialize_timers_32mhz(void);
	
	void initialize_io(void);
	void initialize_defaults(void);
	void enable_interrupts(void);

	void initialize_fpu(void);
	void initialize_mag(void);
	void initialize_gyro(void);
	void initialize_accel(void);

	void process_critical_sensors(void);
	void process_100hz_tasks(void);
	void process_50hz_tasks(void);
	void process_10hz_tasks_1(void);
	void process_10hz_tasks_2(void);
	
	void main_loop(void);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif
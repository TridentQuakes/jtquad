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
   
	/* Main loop timing variables. Main loop runs at 400HZ. Main loop frame size is 1s.
	   Frame counter is incremented at 100hz. Critical sensors are read at 400hz and
	   averaged at 100hz (i.e. 4 samples are averaged. Main loop is capable of running
	   at approximately 2500hz */
	uint32_t current_time = 0;
	uint32_t previous_time = 0;
	uint32_t delta_time = 0;
	uint32_t previous_time_400hz = 0;
	uint32_t delta_time_400hz = 0;
	uint32_t previous_time_100hz_tasks = 0;
	uint32_t previous_time_50hz_tasks = 0;
	uint32_t previous_time_10hz_tasks = 0;
	uint32_t previous_time_10hz_tasks_low_priority_1 = 0;
	
	#ifdef MEGANEURA_DEBUG
	uint32_t loop_time = 0;
	uint32_t loop_time_start = 0;
	
	uint32_t temp_time = 0;
	
	uint32_t time_critical_tasks = 0;
	uint32_t time_critical_tasks_start = 0;
	
	uint32_t time_100hz_tasks = 0;
	uint32_t time_100hz_tasks_start = 0;
	uint32_t time_50hz_tasks = 0;
	uint32_t time_50hz_tasks_start = 0;
	uint32_t time_10ahz_tasks = 0;
	uint32_t time_10ahz_tasks_start = 0;
	uint32_t time_10bhz_tasks = 0;
	uint32_t time_10bhz_tasks_start = 0;
	
	// 100 hz tasks
	uint32_t time_accel_eval_ms2 = 0;
	uint32_t time_fourth_order = 0;
	uint32_t time_gyro_evaluate_rate = 0;
	uint32_t time_calc_kinematics = 0;
	uint32_t time_process_flight_control = 0;
	
	uint8_t app_events = 0;
	#define APP_EVENT_400_bp	0
	#define	APP_EVENT_400_bm	0b00000001
	#define APP_EVENT_100_bp	1
	#define	APP_EVENT_100_bm	0b00000010
	#define APP_EVENT_50_bp		2
	#define	APP_EVENT_50_bm		0b00000100
	#define APP_EVENT_10A_bp	3
	#define	APP_EVENT_10A_bm	0b00001000
	#define APP_EVENT_10B_bp	4
	#define	APP_EVENT_10B_bm	0b00010000
	//uint32_t max_event_times[5] = {0};
	
	
	// debugging flight control
	double g_rate_x = 0;
	//double g_rate_y = 0;
	//double g_rate_z = 0;
	int m_axis_roll_cmd = 0;
	//int m_axis_pitch_cmd = 0;
	//int m_axis_yaw_cmd = 0;
	double r_SI_data_x = 0;
	//double r_SI_data_y = 0;
	//double r_SI_data_z = 0;
	double rot_speed_factor = 0;
	
	double pid_x_P = 0;
	double pid_x_I = 0;
	double pid_x_D = 0;
	double pid_x_lastError = 0;
	double pid_x_previousPIDTime = 0;
	double pid_x_integratedError = 0;
	double pid_x_windupGuard = 0;
	
	/*double pid_y_P = 0;
	double pid_y_I = 0;
	double pid_y_D = 0;
	double pid_y_lastError = 0;
	double pid_y_previousPIDTime = 0;
	double pid_y_integratedError = 0;
	double pid_y_windupGuard = 0;
	
	double cmd_yaw = 0;
	
	double pid_z_P = 0;
	double pid_z_I = 0;
	double pid_z_D = 0;
	double pid_z_lastError = 0;
	double pid_z_previousPIDTime = 0;
	double pid_z_integratedError = 0;
	double pid_z_windupGuard = 0;
	*/
	//int throttle_from_receiver = 0;
	//int throttle_after_correction = 0;
	
	//int m_cmd_after_calc_m_cmd[4] = {0};
	//int m_cmd_after_minmax[4] = {0};
	
	#endif
	
	// Global value used to calculate delta time between sensor processing in tasks
	double G_Dt = 0.002;

	// Used to implement the micros and millis functions
	volatile uint32_t timer_tcc1_overflow_count = 0;
	//volatile uint32_t timer_tcd0_overflow_count = 0;
	
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
	
	// Eight flags for each peripheral device
	uint8_t mn_flags = 0;
	uint8_t temp_flags = 0;
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
	//void initialize_clock_16mhz(void);
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
	//void process_10hz_tasks_3(void);
	
	void main_loop(void);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif
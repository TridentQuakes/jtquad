/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __PCPORTAL_H__
#define __PCPORTAL_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "common.h"
	
/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers:
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	/* ----------------------------------------------------------------------------
	   General reporting functions
       ---------------------------------------------------------------------------- */
	void pc_send_version(void);
	void pc_send_loop_time(void);
	void pc_send_vehicle_state(void);
	void pc_send_vehicle_configuration(void);
	
	/* ----------------------------------------------------------------------------
	   Accelerometer reporting functions
       ---------------------------------------------------------------------------- */
	void pc_send_accel_raw_vector(void);
	void pc_send_accel_ms2_vector(void);
	void pc_send_accel_filtered_vector(void);
	void pc_send_accel_all(void);
	void pc_send_accel_bias(void);
	void pc_send_accel_sample_registers(void);
	void pc_send_accel_calibration_data(void);
	
	/* ----------------------------------------------------------------------------
	   Gyroscope reporting functions
       ---------------------------------------------------------------------------- */
	void pc_send_gyro_raw_vector(void);
	void pc_send_gyro_filtered_vector(void);
	void pc_send_gyro_calibration_data(void);
	void pc_send_gyro_heading(void);
	void pc_send_gyro_sample_registers(void);
	void pc_send_gyro_temperature(void);
	
	/* ----------------------------------------------------------------------------
	   Magnetometer reporting functions
       ---------------------------------------------------------------------------- */
	void pc_send_mag_raw_data(void);
	void pc_send_mag_measured_data(void);
	void pc_send_mag_calibration_data(void);
	void pc_send_mag_absolute_heading(void);
	void pc_send_mag_offset_calculation_data(void);
	
	/* ----------------------------------------------------------------------------
	   Flight data reporting functions
       ---------------------------------------------------------------------------- */
	void pc_send_vehicle_attitude(void);
	void pc_send_all_flight_data(void);
	void pc_send_motor_commands(void);
	void get_heading(uint8_t format, char *p_str_heading);
	void get_absolute_heading(uint8_t format, char *p_str_heading);
	
	
	/* ----------------------------------------------------------------------------
	   Transmitter functions
       ---------------------------------------------------------------------------- */
	void pc_send_transmitter_data(bool send_line_end);
	void pc_send_transmitter_smoothing_values(void);
	void pc_send_transmitter_slope(void);
	void pc_send_transmitter_offset(void);
	
	/* ----------------------------------------------------------------------------
	   PID
	   ---------------------------------------------------------------------------- */
	void pc_send_PID(uint8_t IDPid);
	
	// RATE MODE - PID values
	void pc_send_rate_mode_PID(void);
	
	// ATTITUDE MODE - PID values
	void pc_send_attitude_mode_PID(void);
	
	// YAW - PID values
	void pc_send_yaw_PID(void);
	
	/* ----------------------------------------------------------------------------
	   Error reporting functions
       ---------------------------------------------------------------------------- */
	int pc_report_error(int errorNum);
	
	/* ----------------------------------------------------------------------------
	   Utility functions
       ---------------------------------------------------------------------------- */
	void pc_send_string_resource(const char *pString);
	void pc_send_int_comma(int value);
	void pc_send_double_comma(double value, uint8_t width, uint8_t precision);
	void pc_send_uint32_t_comma(uint32_t value);
	
	/* ----------------------------------------------------------------------------
	   FPU functions
       ---------------------------------------------------------------------------- */
	void pc_send_3d_vector_delimited(uint8_t register_x, uint8_t register_y, uint8_t register_z, 
						const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format);
	void pc_send_2d_vector_delimited(uint8_t register_x, uint8_t register_y, 
						const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format);
	void pc_send_eeprom_3d_vector_delimited(uint8_t slot_x, uint8_t slot_y, uint8_t slot_z, 
								const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format);

	/* ----------------------------------------------------------------------------
	   Miscellaneous configuration values
       ---------------------------------------------------------------------------- */
	void pc_send_min_armed_throttle(void);
	
	char* deblank(char* input);
	
	/* ----------------------------------------------------------------------------
	   Debug values
       ---------------------------------------------------------------------------- */
	void pc_send_flight_control_debug_values(void);
	
	/* ----------------------------------------------------------------------------
	   Read/Action command functions
       ---------------------------------------------------------------------------- */
	   void pc_read_rate_mode_PID(void);
	   void pc_read_attitude_mode_PID(void);
	   void pc_read_yaw_pid(void);
	   void pc_read_min_armed_throttle(void);
	   void pc_read_receiver_smoothing_values(void);
	   void pc_read_channel_slope(void);
	   void pc_read_channel_offset(void);
	   void pc_initialize_eeprom_to_defaults(void);
	   void pc_calibrate_gyro(void);
	   void pc_calibrate_accel(void);
	   void pc_read_mag_offset(void);
	   
	// Disable C linkage for C++ Compilers:
	#if defined(__cplusplus)
	}
	#endif
		
#endif

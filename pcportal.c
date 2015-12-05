/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "pcportal.h"

/* ================================================================================================
   Externs
   ================================================================================================ */
	
	extern const Error_t errors[NUM_ERRORS];
	extern uint8_t motors_state;
	extern uint8_t flight_mode;
	extern int receiver_command[NUMBER_OF_CHANNELS];
	extern double receiver_slope[NUMBER_OF_CHANNELS];
	extern double receiver_offset[NUMBER_OF_CHANNELS];
	extern double receiver_smooth_factor[NUMBER_OF_CHANNELS];
	extern int min_armed_throttle;
	extern double receiver_transmit_factor;
	extern uint32_t loop_time;
	extern double rotation_speed_factor;
	extern uint32_t vehicle_state;
	extern int motor_command[4];
	extern uint8_t 	heading_hold_config;
	
	#ifdef MEGANEURA_DEBUG
	extern uint32_t loop_time;
	extern uint32_t loop_time_start;
	extern uint32_t time_100hz_tasks;
	extern uint32_t time_50hz_tasks;
	extern uint32_t time_10ahz_tasks;
	extern uint32_t time_10bhz_tasks;
	extern uint32_t time_critical_tasks;
	extern uint32_t time_wait_fpu_tasks;

	extern uint32_t time_critical_tasks_start;
	extern uint32_t time_100hz_tasks_start;
	extern uint32_t time_50hz_tasks_start;
	extern uint32_t time_10ahz_tasks_start;
	extern uint32_t time_10bhz_tasks_start;
	
	extern uint32_t time_accel_eval_ms2;
	extern uint32_t time_fourth_order;
	extern uint32_t time_gyro_evaluate_rate;
	extern uint32_t time_calc_kinematics;
	extern uint32_t time_process_flight_control;
	
	
	extern double g_rate_x;
	extern double g_rate_y;
	extern double g_rate_z;
	extern int m_axis_roll_cmd;
	extern int m_axis_pitch_cmd;
	extern int m_axis_yaw_cmd;
	extern double r_SI_data_x;
	extern double r_SI_data_y;
	extern double r_SI_data_z;
	extern double rot_speed_factor;
	
	extern double pid_x_P;
	extern double pid_x_I;
	extern double pid_x_D;
	extern double pid_x_lastError;
	extern double pid_x_previousPIDTime;
	extern double pid_x_integratedError;
	extern double pid_x_windupGuard;
	
	extern double pid_y_P;
	extern double pid_y_I;
	extern double pid_y_D;
	extern double pid_y_lastError;
	extern double pid_y_previousPIDTime;
	extern double pid_y_integratedError;
	extern double pid_y_windupGuard;
	
	extern double cmd_yaw;
	
	extern double pid_z_P;
	extern double pid_z_I;
	extern double pid_z_D;
	extern double pid_z_lastError;
	extern double pid_z_previousPIDTime;
	extern double pid_z_integratedError;
	extern double pid_z_windupGuard;
	
	extern int throttle_from_receiver;
	extern int throttle_after_correction;
	
	extern int m_cmd_after_calc_m_cmd[4];
	extern int m_cmd_after_minmax[4];
	
	#endif
	
	extern VectorDouble3 			accel_scale;
	
	#ifdef HMC5883L
	extern VectorDouble3 			mag_offset;
	#endif
	
	extern uint8_t accel_flags;
	extern uint8_t gyro_flags;
	extern uint8_t mag_flags;
	extern uint8_t fpu_flags;
	
/* ================================================================================================
   General reporting functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send version number
   ---------------------------------------------------------------------------- */
void pc_send_version(void)
{
	char sTemp[25] = {0};
	
	strcpy(sTemp, STRING_WELCOME);
	strcat(sTemp, VERSION_STRING);
	strcat(sTemp, LINE_END);
	usart_tx_string(sTemp);
}

/* ----------------------------------------------------------------------------
   Send time to execute one loop in microseconds
   ---------------------------------------------------------------------------- */
#ifdef MEGANEURA_DEBUG
void pc_send_loop_time(void)
{
	pc_send_uint32_t_comma(loop_time_start);
	pc_send_uint32_t_comma(loop_time);
	pc_send_uint32_t_comma(time_critical_tasks_start);
	pc_send_uint32_t_comma(time_critical_tasks);
	pc_send_uint32_t_comma(time_100hz_tasks_start);
	pc_send_uint32_t_comma(time_100hz_tasks);
	pc_send_uint32_t_comma(time_50hz_tasks_start);
	pc_send_uint32_t_comma(time_50hz_tasks);
	pc_send_uint32_t_comma(time_10ahz_tasks_start);
	pc_send_uint32_t_comma(time_10ahz_tasks);
	pc_send_uint32_t_comma(time_10bhz_tasks_start);
	pc_send_uint32_t_comma(time_10bhz_tasks);
	pc_send_uint32_t_comma(time_accel_eval_ms2 + time_fourth_order); 	// accel
	pc_send_uint32_t_comma(time_gyro_evaluate_rate);					// gyro
	pc_send_uint32_t_comma(time_calc_kinematics);						// kinematics
	pc_send_uint32_t_comma(time_process_flight_control);				// process flight control
	
	usart_tx_string(LINE_END);
}
#endif

/* ----------------------------------------------------------------------------
   Send vehicle state 32 bit register
   ---------------------------------------------------------------------------- */
void pc_send_vehicle_state(void)
{
	usart_tx_ul(vehicle_state);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send vehicle configuration
   ---------------------------------------------------------------------------- */
void pc_send_vehicle_configuration(void)
{
	pc_send_int_comma(LASTMOTOR);		// Number of motors
	usart_tx_string("X,");				// Form
	(bit_test_(accel_flags, ACCEL_FLAG_IS_INIT)) ? usart_tx_string("Detected,") : usart_tx_string("Not Detected,"); // Accel detected
	(bit_test_(gyro_flags, GYRO_FLAG_IS_INIT)) ? usart_tx_string("Detected,") : usart_tx_string("Not Detected,"); // Gyro detected
	#ifdef HMC5883L
	(bit_test_(mag_flags, MAG_FLAG_IS_INIT)) ? usart_tx_string("Detected,") : usart_tx_string("Not Detected,"); // Mag detected
	#else
	usart_tx_string("Not Detected,");
	#endif
	(bit_test_(fpu_flags, FPU_FLAG_IS_INIT)) ? usart_tx_string("Detected,") : usart_tx_string("Not Detected,"); // Mag detected
	usart_tx_string(LINE_END);
}

/* ================================================================================================
   Accelerometer reporting functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send raw accelerometer vector
   ---------------------------------------------------------------------------- */
void pc_send_accel_raw_vector(void)
{
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_X, FPU_REG_ACCEL_Y, FPU_REG_ACCEL_Z, CSV_DELIMITER, LINE_END, 93);
}

/* ----------------------------------------------------------------------------
   Send accelerometer vector scaled to MS^2
   ---------------------------------------------------------------------------- */
void pc_send_accel_ms2_vector(void)
{
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_MS2_X, FPU_REG_ACCEL_MS2_Y, FPU_REG_ACCEL_MS2_Z, CSV_DELIMITER, LINE_END, 73);
}

/* ----------------------------------------------------------------------------
   Send filtered accelerometer vector
   ---------------------------------------------------------------------------- */
void pc_send_accel_filtered_vector(void)
{
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_FILTERED_X, FPU_REG_ACCEL_FILTERED_Y, FPU_REG_ACCEL_FILTERED_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
}

/* ----------------------------------------------------------------------------
   Send all accelerometer vectors
   ---------------------------------------------------------------------------- */
void pc_send_accel_all(void)
{
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_X, FPU_REG_ACCEL_Y, FPU_REG_ACCEL_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_MS2_X, FPU_REG_ACCEL_MS2_Y, FPU_REG_ACCEL_MS2_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
	pc_send_3d_vector_delimited(FPU_REG_ACCEL_FILTERED_X, FPU_REG_ACCEL_FILTERED_Y, FPU_REG_ACCEL_FILTERED_Z, CSV_DELIMITER, LINE_END, 73);
}

/* ----------------------------------------------------------------------------
   Send accelerometer calibration data
   ---------------------------------------------------------------------------- */
void pc_send_accel_calibration_data(void)
{
	pc_send_eeprom_3d_vector_delimited(FPU_EEPROM_SLOT_ACCEL_OFFSET_X, FPU_EEPROM_SLOT_ACCEL_OFFSET_Y, FPU_EEPROM_SLOT_ACCEL_OFFSET_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
	pc_send_eeprom_3d_vector_delimited(FPU_EEPROM_SLOT_ACCEL_SCALE_X, FPU_EEPROM_SLOT_ACCEL_SCALE_Y, FPU_EEPROM_SLOT_ACCEL_SCALE_Z, CSV_DELIMITER, LINE_END, 73);
}

/* ----------------------------------------------------------------------------
   Send the accelerometer sampling values
   ---------------------------------------------------------------------------- */
void pc_send_accel_sample_registers(void)
{
	char sTemp[11] = {0};
	usart_tx_string("x sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_ACCEL_X_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("y sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_ACCEL_Y_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("z sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_ACCEL_Z_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("number of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_ACCEL_NUMBER_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
}

/* ================================================================================================
   Gyroscope reporting functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send raw gyroscope vector
   ---------------------------------------------------------------------------- */
void pc_send_gyro_raw_vector(void)
{
	pc_send_3d_vector_delimited(FPU_REG_GYRO_X, FPU_REG_GYRO_Y, FPU_REG_GYRO_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
}

/* ----------------------------------------------------------------------------
   Send filtered gyroscope vector
   ---------------------------------------------------------------------------- */
void pc_send_gyro_filtered_vector(void)
{
	pc_send_3d_vector_delimited(FPU_REG_GYRO_FILTERED_X, FPU_REG_GYRO_FILTERED_Y, FPU_REG_GYRO_FILTERED_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
}

/* ----------------------------------------------------------------------------
   Send gyroscope bias (offset)
   ---------------------------------------------------------------------------- */
void pc_send_gyro_calibration_data(void)
{
	pc_send_eeprom_3d_vector_delimited(FPU_EEPROM_SLOT_GYRO_OFFSET_X, FPU_EEPROM_SLOT_GYRO_OFFSET_Y, FPU_EEPROM_SLOT_GYRO_OFFSET_Z, CSV_DELIMITER, CSV_DELIMITER, 73);
	usart_tx_double_from_fpu_eeprom(FPU_EEPROM_SLOT_GYRO_SCALE, 73);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send heading computed using gyroscope
   ---------------------------------------------------------------------------- */
void pc_send_gyro_heading(void)
{
	char sTemp[11] = {0};
	fpu_write2(SELECTA, FPU_REG_GYRO_HEADING);
	fpu_ftoa_reg_a(73, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send gyroscope sampling values
   ---------------------------------------------------------------------------- */
void pc_send_gyro_sample_registers(void)
{
	char sTemp[11] = {0};
	usart_tx_string("x sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_GYRO_X_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("y sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_GYRO_Y_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("z sum of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_GYRO_Z_SUM_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	usart_tx_string("number of samples: ");
	fpu_wait();
	fpu_write2(SELECTA, FPU_REG_GYRO_NUMBER_OF_SAMPLES);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send gyroscope temperature
   ---------------------------------------------------------------------------- */
#if 0
void pc_send_gyro_temperature(void)
{
	char sTemp[11] = {0};
	fpu_write2(SELECTA, FPU_REG_GYRO_TEMPERATURE);
	fpu_ftoa_reg_a(0, sTemp);
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
}
#endif


/* ================================================================================================
   Magnetometer reporting functions
   ================================================================================================ */

#ifdef HMC5883L
/* ----------------------------------------------------------------------------
   Send raw magnetometer vector
   ---------------------------------------------------------------------------- */
void pc_send_mag_raw_data(void)
{
	pc_send_3d_vector_delimited(FPU_REG_MAG_X, FPU_REG_MAG_Y, FPU_REG_MAG_Z, CSV_DELIMITER, LINE_END, 93);
}

/* ----------------------------------------------------------------------------
   Send measured magnetometer vector
   ---------------------------------------------------------------------------- */
void pc_send_mag_measured_data(void)
{
	pc_send_3d_vector_delimited(FPU_REG_MAG_MEASURED_X, FPU_REG_MAG_MEASURED_Y, FPU_REG_MAG_MEASURED_Z, CSV_DELIMITER, CSV_DELIMITER, 93);
}

/* ----------------------------------------------------------------------------
   Send magnetometer calibration data
   ---------------------------------------------------------------------------- */
void pc_send_mag_calibration_data(void)
{
	pc_send_eeprom_3d_vector_delimited(FPU_EEPROM_SLOT_MAG_OFFSET_X, FPU_EEPROM_SLOT_MAG_OFFSET_Y, FPU_EEPROM_SLOT_MAG_OFFSET_Z, CSV_DELIMITER, CSV_DELIMITER, 93);
	pc_send_eeprom_3d_vector_delimited(FPU_EEPROM_SLOT_MAG_SCALE_X, FPU_EEPROM_SLOT_MAG_SCALE_Y, FPU_EEPROM_SLOT_MAG_SCALE_Z, CSV_DELIMITER, LINE_END, 93);
}

/* ----------------------------------------------------------------------------
   Send magnetometer measured (x,y,z) and absolute heading
   ---------------------------------------------------------------------------- */
void pc_send_mag_absolute_heading(void)
{
	char sAbsoluteHeading[15] = {0};
	
	pc_send_mag_measured_data();
	get_absolute_heading(93, sAbsoluteHeading);
	usart_tx_string(sAbsoluteHeading);
	usart_tx_string(",\r\n");
}

/* ----------------------------------------------------------------------------
   Send magnetometer measured and tilt compensated x, y and z values
     - used for offset calculations by hand and not needed otherwise
   ---------------------------------------------------------------------------- */
void pc_send_mag_offset_calculation_data(void)
{
	pc_send_2d_vector_delimited(FPU_REG_MAG_X, FPU_REG_MAG_Y, CSV_DELIMITER, CSV_DELIMITER, 93);
	pc_send_2d_vector_delimited(FPU_REG_MAG_MEASURED_X, FPU_REG_MAG_MEASURED_Y, CSV_DELIMITER, CSV_DELIMITER, 93);
	pc_send_2d_vector_delimited(FPU_REG_MAG_TILT_COMPENSATED_X, FPU_REG_MAG_TILT_COMPENSATED_Y, CSV_DELIMITER, LINE_END, 93);
}
#endif

/* ================================================================================================
   Flight data reporting functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send vehicle attitude
   ---------------------------------------------------------------------------- */ 
void pc_send_vehicle_attitude(void)
{
	char sHeading[15] = {0};
	
	pc_send_2d_vector_delimited(FPU_REG_KINEMATICS_ANGLE_X, FPU_REG_KINEMATICS_ANGLE_Y, CSV_DELIMITER, CSV_DELIMITER, 73);
	get_heading(93, sHeading);
	usart_tx_string(sHeading);
	usart_tx_string(",\r\n");
	
}

/* ----------------------------------------------------------------------------
   Send all flight data
   ---------------------------------------------------------------------------- */
void pc_send_all_flight_data(void)
{
	// Motors armed state
	pc_send_int_comma(motors_state);
	
	// Vehicle attitude
	char sHeading[11] = {0};
	pc_send_2d_vector_delimited(FPU_REG_KINEMATICS_ANGLE_X, FPU_REG_KINEMATICS_ANGLE_Y, CSV_DELIMITER, CSV_DELIMITER, 72);
	get_heading(92, sHeading);
	
	usart_tx_string(sHeading);
	usart_tx_string(",");
	
	// Altitude and altitude hold state
	usart_tx_string("0,0,");
	
	// Channels (configurator expects 8 values)
	pc_send_transmitter_data(false);
	pc_send_int_comma(0);
	
	pc_send_motor_commands();
	usart_tx_string("0,0,0,0,"); // Dummy values because configurator expects 8 values
	
	// Battery monitor
	pc_send_double_comma(19.95, 5, 2);
	
	// Flight mode
	pc_send_int_comma(flight_mode);
	
	usart_tx_string(LINE_END);
}



/* ----------------------------------------------------------------------------
   Send motor commands
   ---------------------------------------------------------------------------- */
void pc_send_motor_commands(void)
{
	// Motor command
	for (MOTORS_t motor = MOTOR_0; motor < LASTMOTOR; motor++)
	{
      pc_send_int_comma(motor_command[motor]);
    }
}

/* ----------------------------------------------------------------------------
   Get heading from gyroscope or magnetometer
   ---------------------------------------------------------------------------- */
void get_heading(uint8_t format, char *p_str_heading)
{
	#ifndef HMC5883L
	// Get the heading from the gyro if a magnetometer is not present
	fpu_write2(SELECTA, FPU_REG_GYRO_HEADING);
	fpu_ftoa_reg_a(format, p_str_heading);
	#else
	// Get the heading from the magnetometer
	//   - atan2 produces results in the range (-pi, pi]
	//   - map results to [0, 2pi) by adding 2pi to the negative results
	fpu_write3(COPY, FPU_REG_MAG_TRUE_NORTH_HEADING, FPU_REG_TEMPORARY_1);
	fpu_function_call(FPU_FUNC_MAG_CALCULATE_POSITIVE_HEADING);
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);
	fpu_ftoa_reg_a(format, p_str_heading);
	#endif
	
	deblank(p_str_heading);
}

#ifdef HMC5883L
/* ----------------------------------------------------------------------------
   Get absolute heading based on measured magnetometer data
   ---------------------------------------------------------------------------- */
void get_absolute_heading(uint8_t format, char *p_str_heading)
{
	fpu_function_call(FPU_FUNC_MAG_CALCULATE_ABSOLUTE_HEADING);
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);
	fpu_ftoa_reg_a(format, p_str_heading);
}
#endif

/* ================================================================================================
   Transmitter functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send transmitter channel values
   ---------------------------------------------------------------------------- */
void pc_send_transmitter_data(bool send_line_end)
{
	for(uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	{
		pc_send_int_comma(receiver_command[channel]);
	}

	if( send_line_end == true )
	{
		usart_tx_string(LINE_END);
	}
}

/* ----------------------------------------------------------------------------
   Send transmitter smooth factor
   ---------------------------------------------------------------------------- */
void pc_send_transmitter_smoothing_values(void)
{
	pc_send_double_comma(receiver_transmit_factor, 3, 1);
	
	//for(uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	for (RECEIVER_CHANNELS_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	{
		pc_send_double_comma(receiver_smooth_factor[channel], 3, 1);
	}
	
	// Configurator expects 10 values
	pc_send_int_comma(0);
	pc_send_int_comma(0);
	pc_send_int_comma(0);
	
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send transmitter slope
   ---------------------------------------------------------------------------- */
void pc_send_transmitter_slope(void)
{
	for(uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	{
		pc_send_double_comma(receiver_slope[channel], 3, 1);
	}

	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send transmitter offset
   ---------------------------------------------------------------------------- */
void pc_send_transmitter_offset(void)
{
	for(uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++)
	{
		pc_send_double_comma(receiver_offset[channel], 8, 1);
	}
	
	usart_tx_string(LINE_END);
}

/* ================================================================================================
   PID reporting functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Send PID
		PID_idx: index of PID 
   ---------------------------------------------------------------------------- */
void pc_send_PID(uint8_t IDPid)
{
	pc_send_double_comma(PID[IDPid].P, 8, 1);
	pc_send_double_comma(PID[IDPid].I, 8, 1);
	pc_send_double_comma(PID[IDPid].D, 8, 1);
}

/* ----------------------------------------------------------------------------
   Send RATE MODE - PID values (roll and pitch)
   ---------------------------------------------------------------------------- */
void pc_send_rate_mode_PID(void)
{
	pc_send_PID(RATE_XAXIS_PID_IDX);
	pc_send_PID(RATE_YAXIS_PID_IDX);
	pc_send_double_comma(rotation_speed_factor, 8, 1);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send ATTITUDE MODE - PID values (roll and pitch)
   ---------------------------------------------------------------------------- */
void pc_send_attitude_mode_PID(void)
{
	pc_send_PID(ATTITUDE_XAXIS_PID_IDX);
	pc_send_PID(ATTITUDE_YAXIS_PID_IDX);
	pc_send_PID(ATTITUDE_GYRO_XAXIS_PID_IDX);
	pc_send_PID(ATTITUDE_GYRO_YAXIS_PID_IDX);
	pc_send_double_comma(wind_up_guard, 8, 1);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send yaw - PID values
   ---------------------------------------------------------------------------- */
void pc_send_yaw_PID(void)
{
	pc_send_PID(ZAXIS_PID_IDX);
	pc_send_PID(HEADING_HOLD_PID_IDX);
	pc_send_int_comma(heading_hold_config);
	usart_tx_string(LINE_END);
}

/* ================================================================================================
   Error reporting functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Report error
   ---------------------------------------------------------------------------- */
int pc_report_error(int errorNum)
{
	char sTemp[5] = {0};
	int index = -1;
	int ret = ERR_SUCCESS;
	
	ret = get_error_index(errorNum, &index);
	
	if(ret != ERR_SUCCESS)
	{
		usart_tx_string("error getting error message");
		return ret;
	}
	
	itoa(errorNum, sTemp, 16);
	
	usart_tx_string("error: ");
	usart_tx_string(errors[index].errorMsg);
	usart_tx_string(LINE_END);
	usart_tx_string("error num: ");
	usart_tx_string(sTemp);
	usart_tx_string(LINE_END);
	
	return ERR_SUCCESS;
}

/* ================================================================================================
   Utility functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send a string resource
   ---------------------------------------------------------------------------- */
void pc_send_string_resource(const char *pString)
{
	usart_tx_string(pString);
	usart_tx_string(LINE_END);
}

/* ----------------------------------------------------------------------------
   Send a int value comma pair
   ---------------------------------------------------------------------------- */
void pc_send_int_comma(int value)
{
	usart_tx_int(&value);
	usart_tx_string(",");
}

/* ----------------------------------------------------------------------------
   Send a double value comma pair
   ---------------------------------------------------------------------------- */
void pc_send_double_comma(double value, uint8_t width, uint8_t precision)
{
	usart_tx_double(value, width, precision);
	usart_tx_string(CSV_DELIMITER);
}

/* ----------------------------------------------------------------------------
   Send uint32_t value comma pair
   ---------------------------------------------------------------------------- */
void pc_send_uint32_t_comma(uint32_t value)
{
	usart_tx_ul(value);
	usart_tx_string(",");
}

/* ================================================================================================
   FPU functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Send a delimited 3D vector from FPU
   ---------------------------------------------------------------------------- */
void pc_send_3d_vector_delimited(uint8_t register_x, uint8_t register_y, uint8_t register_z, 
								const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format)
{
	/* 	
		Notes on format:
		
		strings length = 10 if not expressed with scientific notation
		strings length = 13 with scientific notation (e.g. " 3.5249278E-4")
		strings include '-' and '.'
		special cases are "NaN", "Infinity", "-Infinity", " 3.5249278E-4" and "-0.0"
		if the format is not 0 you can get back a string such as "*.**"
		73 is 7 digits and precision of 3 after decimal point
	*/
	char sTemp[15] = {0};
	
	fpu_write2(SELECTA, register_x);
	
	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_Delimiter);
	
	fpu_write2(SELECTA, register_y);
	
	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_Delimiter);
	
	fpu_write2(SELECTA, register_z);

	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_EndOfLine);
}

/* ----------------------------------------------------------------------------
   Send a delimited 2D vector from FPU
   ---------------------------------------------------------------------------- */
void pc_send_2d_vector_delimited(uint8_t register_x, uint8_t register_y, 
								const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format)
{
	/* 	
		Notes on format:
		
		strings length = 10 if not expressed with scientific notation
		strings length = 13 with scientific notation (e.g. " 3.5249278E-4")
		strings include '-' and '.'
		special cases are "NaN", "Infinity", "-Infinity", " 3.5249278E-4" and "-0.0"
		if the format is not 0 you can get back a string such as "*.**"
		73 is 7 digits and precision of 3 after decimal point
	*/
	char sTemp[11] = {0};
	
	fpu_write2(SELECTA, register_x);

	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_Delimiter);
	
	fpu_write2(SELECTA, register_y);

	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_EndOfLine);
}



/* ----------------------------------------------------------------------------
   Send a delimited 3D vector from FPU's EEPROM
   ---------------------------------------------------------------------------- */
void pc_send_eeprom_3d_vector_delimited(uint8_t slot_x, uint8_t slot_y, uint8_t slot_z, 
								const char *StringPtr_Delimiter, const char *StringPtr_EndOfLine, uint8_t format)
{
	/* 	
		Notes on format:
		
		strings length = 10 if not expressed with scientific notation
		strings length = 13 with scientific notation (e.g. " 3.5249278E-4")
		strings include '-' and '.'
		special cases are "NaN", "Infinity", "-Infinity", " 3.5249278E-4" and "-0.0"
		if the format is not 0 you can get back a string such as "*.**"
		73 is 7 digits and precision of 3 after decimal point
	*/
	char sTemp[15] = {0};
	
	fpu_write3(EELOAD, FPU_REG_TEMPORARY_1, slot_x);
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);

	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_Delimiter);
	
	fpu_write3(EELOAD, FPU_REG_TEMPORARY_1, slot_y);
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);

	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_Delimiter);
	
	fpu_write3(EELOAD, FPU_REG_TEMPORARY_1, slot_z);
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);
	
	fpu_ftoa_reg_a(format, sTemp);
	usart_tx_string(deblank(sTemp));
	usart_tx_string(StringPtr_EndOfLine);
}

/* ================================================================================================
   Miscellaneous configuration values
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Send min armed throttle value
   ---------------------------------------------------------------------------- */
void pc_send_min_armed_throttle(void)
{
	usart_tx_int(&min_armed_throttle);
	usart_tx_string(LINE_END);
}


char* deblank(char* input)                                         
{
    int i,j;
    char *output=input;
    for (i = 0, j = 0; i<strlen(input); i++,j++)          
    {
        if (input[i]!=' ')                           
            output[j]=input[i];                     
        else
            j--;                                     
    }
    output[j]= '\0';
    return output;
}

/* ================================================================================================
   Debug values
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Send flight control debug values
   ---------------------------------------------------------------------------- */
#ifdef MEGANEURA_DEBUG
void pc_send_flight_control_debug_values(void)
{
	pc_send_double_comma(g_rate_x, 8, 3);
	//pc_send_double_comma(g_rate_y, 8, 3);
	//pc_send_double_comma(g_rate_z, 8, 3);
	pc_send_int_comma(m_axis_roll_cmd);
	//pc_send_int_comma(m_axis_pitch_cmd);
	//pc_send_int_comma(m_axis_yaw_cmd);
	
	pc_send_double_comma(r_SI_data_x, 8, 3);
	//pc_send_double_comma(r_SI_data_y, 8, 3);
	//pc_send_double_comma(r_SI_data_z, 8, 3);
	
	pc_send_double_comma(rot_speed_factor, 6, 2);
	
	pc_send_double_comma(pid_x_P, 8, 0);
	pc_send_double_comma(pid_x_I, 8, 0);
	pc_send_double_comma(pid_x_D, 8, 0);
	pc_send_double_comma(pid_x_lastError, 8, 3);
	pc_send_double_comma(pid_x_previousPIDTime, 8, 0);
	pc_send_double_comma(pid_x_integratedError, 8, 3);
	pc_send_double_comma(pid_x_windupGuard, 8, 0);
	
	/*pc_send_double_comma(pid_y_P, 8, 3);
	pc_send_double_comma(pid_y_I, 8, 3);
	pc_send_double_comma(pid_y_D, 8, 3);
	pc_send_double_comma(pid_y_lastError, 8, 3);
	pc_send_double_comma(pid_y_previousPIDTime, 8, 3);
	pc_send_double_comma(pid_y_integratedError, 8, 3);
	pc_send_double_comma(pid_y_windupGuard, 8, 3);
	
	pc_send_double_comma(cmd_yaw, 8, 3);
	
	pc_send_double_comma(pid_z_P, 8, 3);
	pc_send_double_comma(pid_z_I, 8, 3);
	pc_send_double_comma(pid_z_D, 8, 3);
	pc_send_double_comma(pid_z_lastError, 8, 3);
	pc_send_double_comma(pid_z_previousPIDTime, 8, 3);
	pc_send_double_comma(pid_z_integratedError, 8, 3);
	pc_send_double_comma(pid_z_windupGuard, 8, 3);
	*/
	//pc_send_int_comma(throttle_from_receiver);
	//pc_send_int_comma(throttle_after_correction);
	
	/*
	pc_send_int_comma(m_cmd_after_calc_m_cmd[FRONT_LEFT]);
	pc_send_int_comma(m_cmd_after_calc_m_cmd[FRONT_RIGHT]);
	pc_send_int_comma(m_cmd_after_calc_m_cmd[REAR_LEFT]);
	pc_send_int_comma(m_cmd_after_calc_m_cmd[REAR_RIGHT]);
	
	pc_send_int_comma(m_cmd_after_minmax[FRONT_LEFT]);
	pc_send_int_comma(m_cmd_after_minmax[FRONT_RIGHT]);
	pc_send_int_comma(m_cmd_after_minmax[REAR_LEFT]);
	pc_send_int_comma(m_cmd_after_minmax[REAR_RIGHT]);
	*/
}
#endif

/* ================================================================================================
   Read functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read in PID from serial (private function)
   ---------------------------------------------------------------------------- */
void read_serial_PID(unsigned char PIDid)
{
  struct PIDdata* pid = &PID[PIDid];
  pid->P = usart_rx_double();
  pid->I = usart_rx_double();
  pid->D = usart_rx_double();
  pid->lastError = 0;
  pid->integratedError = 0;
}

/* ----------------------------------------------------------------------------
   Read roll/pitch rate mode PID
   ---------------------------------------------------------------------------- */
void pc_read_rate_mode_PID(void)
{
	read_serial_PID(RATE_XAXIS_PID_IDX);
	read_serial_PID(RATE_YAXIS_PID_IDX);
	rotation_speed_factor = usart_rx_double();
}   
   
   
/* ----------------------------------------------------------------------------
   Read roll/pitch attitude mode PID
   ---------------------------------------------------------------------------- */   
void pc_read_attitude_mode_PID(void)
{
	read_serial_PID(ATTITUDE_XAXIS_PID_IDX);
	read_serial_PID(ATTITUDE_YAXIS_PID_IDX);
	read_serial_PID(ATTITUDE_GYRO_XAXIS_PID_IDX);
	read_serial_PID(ATTITUDE_GYRO_YAXIS_PID_IDX);
	wind_up_guard = usart_rx_double();
}

/* ----------------------------------------------------------------------------
   Read yaw PID
   ---------------------------------------------------------------------------- */
void pc_read_yaw_pid(void)
{
	read_serial_PID(ZAXIS_PID_IDX);
	read_serial_PID(HEADING_HOLD_PID_IDX);
	heading_hold_config = usart_rx_double();
}  

/* ----------------------------------------------------------------------------
   Read minimum armed throttle value
   ---------------------------------------------------------------------------- */
void pc_read_min_armed_throttle(void)
{
	min_armed_throttle = usart_rx_double();
}  

/* ----------------------------------------------------------------------------
   Read receiver smoothing values
   ---------------------------------------------------------------------------- */
void pc_read_receiver_smoothing_values(void)
{
	receiver_transmit_factor = usart_rx_double();
	
	for (RECEIVER_CHANNELS_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{
		receiver_smooth_factor[channel] = usart_rx_double();
	}
}

/* ----------------------------------------------------------------------------
   Read receiver channel, slope values
   ---------------------------------------------------------------------------- */
void pc_read_channel_slope(void)
{
	int channel = (int)usart_rx_double();
    receiver_slope[channel] = usart_rx_double();
}

/* ----------------------------------------------------------------------------
   Read receiver channel, offset values
   ---------------------------------------------------------------------------- */
void pc_read_channel_offset(void)
{
	int channel = (int)usart_rx_double();
    receiver_offset[channel] = usart_rx_double();
}

/* ----------------------------------------------------------------------------
   Initialize MCU and FPU EEPROM with all default values
   ---------------------------------------------------------------------------- */
void pc_initialize_eeprom_to_defaults(void)
{
	ds_initialize_eeprom_defaults();// Init MCU EEPROM
	ds_write_eeprom_defaults();		// Write MCU EEPROM
	fpu_init_eeprom_defaults();		// Init FPU EEPROM
	fpu_load_eeprom_data();			// Write FPU EEPROM: Includes accel scale and mag offset
	
	while (!gyro_calibrate()) // this make sure the craft is still befor to continue init process
	{
	#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_GYRO_CALIBRATION_FAIL); // Gyro offset stored to FPU EEPROM
	#endif
	}
	
	accel_compute_bias(); 		// Stores new accel offset to FPU eeprom??AQ did not do. Should I?
	zero_integral_error();		// Zero PIDs
	#ifdef HMC5883L
	mag_self_test();	// Stores new mag scale to eeprom
	#endif
}

/* ----------------------------------------------------------------------------
   Calibrate gyro, write offset values to FPU EEPROM
   ---------------------------------------------------------------------------- */
void pc_calibrate_gyro(void)
{
	while (!gyro_calibrate()) // this make sure the craft is still befor to continue init process
	{
	#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_GYRO_CALIBRATION_FAIL); // Gyro offset stored to FPU EEPROM
	#endif
	}
}

/* ----------------------------------------------------------------------------
   Calibrate accel
	- first read in accel scale (calculate by hand)
	- then automatically calculate the accel offset values
	- accel offset values are written to FPU EEPROM
   ---------------------------------------------------------------------------- */
void pc_calibrate_accel(void)
{
	accel_scale.x = usart_rx_double();
	accel_scale.y = usart_rx_double();
	accel_scale.z = usart_rx_double();
	
	fpu_eeprom_write_double_to_slot(accel_scale.x, FPU_EEPROM_SLOT_ACCEL_SCALE_X);
	fpu_eeprom_write_double_to_slot(accel_scale.y, FPU_EEPROM_SLOT_ACCEL_SCALE_Y);
	fpu_eeprom_write_double_to_slot(accel_scale.z, FPU_EEPROM_SLOT_ACCEL_SCALE_Z);
	
	accel_compute_bias(); // Store new accel offset values to FPU EEPROM for us
}

/* ----------------------------------------------------------------------------
   Read mag offset values
	  - first read in mag offset values (calculated by hand)
	  - mag offset values are written to FPU EEPROM
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
void pc_read_mag_offset(void)
{
	mag_offset.x = usart_rx_double();
	mag_offset.y = usart_rx_double();
	mag_offset.z = usart_rx_double();
	
	fpu_eeprom_write_double_to_slot(mag_offset.x, FPU_EEPROM_SLOT_MAG_OFFSET_X);
	fpu_eeprom_write_double_to_slot(mag_offset.y, FPU_EEPROM_SLOT_MAG_OFFSET_Y);
	fpu_eeprom_write_double_to_slot(mag_offset.z, FPU_EEPROM_SLOT_MAG_OFFSET_Z);
}
#endif
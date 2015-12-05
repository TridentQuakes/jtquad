/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "flightcontrolprocessor.h"

/* ================================================================================================
   Externs
   ================================================================================================ */
   
    extern uint8_t 	transmitter_state;
    extern uint8_t calibrate_esc;
	extern uint16_t esc_calibration_test_command;
    extern int motor_configurator_command[4];
	extern uint8_t 	motors_state;
	extern uint8_t 	flight_mode;
	extern int receiver_command[NUMBER_OF_CHANNELS];
	extern int receiver_zero[3];
	extern double rotation_speed_factor;
	extern int throttle;
	extern int min_armed_throttle;
	extern int motor_command[4];
	extern bool in_flight;
	extern uint8_t frame_counter;
	extern int motor_axis_roll_command;
	extern int motor_axis_pitch_command;
	extern int motor_axis_yaw_command;
	extern int motor_min_command[4];
	extern int motor_max_command[4];
	extern volatile uint8_t app_execute;
	
	#ifdef MEGANEURA_DEBUG
	extern uint8_t app_execute_buf;
	
	extern double g_rate_x;
	extern double g_rate_y;
	
	extern int m_axis_roll_cmd;
	extern int m_axis_pitch_cmd;
	
	extern double r_SI_data_x;
	extern double r_SI_data_y;
	
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
	
	extern int throttle_from_receiver;
	extern int throttle_after_correction;
	
	extern int m_cmd_after_calc_m_cmd[4];
	extern int m_cmd_after_minmax[4];
	#endif
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Calculate flight error
   ---------------------------------------------------------------------------- */
void calculate_flight_error(void)
{
	double gyro_rate_x = fpu_read_double_from_register(FPU_REG_GYRO_FILTERED_X);
	double gyro_rate_y = fpu_read_double_from_register(FPU_REG_GYRO_FILTERED_Y);
	
	g_rate_x = gyro_rate_x;

	// Attitude/stable flight mode (use accelerometer and gyroscope to update PID)
	if( flight_mode == ATTITUDE_FLIGHT_MODE )
	{
		double kinematics_angle_x = fpu_read_double_from_register(FPU_REG_KINEMATICS_ANGLE_X);
		double kinematics_angle_y = fpu_read_double_from_register(FPU_REG_KINEMATICS_ANGLE_Y);
		
		// removed flip on kinematics_angle_y, now in euler_angles();
		double roll_attitude_command = PID_update((receiver_command[XAXIS] - receiver_zero[XAXIS]) * ATTITUDE_SCALING, kinematics_angle_x, &PID[ATTITUDE_XAXIS_PID_IDX]);
		double pitch_attitude_command = PID_update((receiver_command[YAXIS] - receiver_zero[YAXIS]) * ATTITUDE_SCALING, kinematics_angle_y, &PID[ATTITUDE_YAXIS_PID_IDX]);
		
		motor_axis_roll_command = PID_update(roll_attitude_command, gyro_rate_x, &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
		motor_axis_pitch_command  = PID_update(pitch_attitude_command, gyro_rate_y, &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
	}
	// Rate/acrobatic flight mode (use only gyroscope to update PID)
	else
	{
		double receiver_SI_data_x = receiver_get_SI_data(XAXIS);
		double receiver_SI_data_y = receiver_get_SI_data(YAXIS);
		motor_axis_roll_command = PID_update(receiver_SI_data_x, gyro_rate_x * rotation_speed_factor, &PID[RATE_XAXIS_PID_IDX]);
		motor_axis_pitch_command = PID_update(receiver_SI_data_y, gyro_rate_y * rotation_speed_factor, &PID[RATE_YAXIS_PID_IDX]);
		
		r_SI_data_x = receiver_SI_data_x;
		m_axis_roll_cmd = motor_axis_roll_command;
		rot_speed_factor = rotation_speed_factor;
		
		pid_x_P = PID[RATE_XAXIS_PID_IDX].P;
		pid_x_I = PID[RATE_XAXIS_PID_IDX].I;
		pid_x_D = PID[RATE_XAXIS_PID_IDX].D;
		pid_x_lastError = PID[RATE_XAXIS_PID_IDX].lastError;
		pid_x_previousPIDTime = PID[RATE_XAXIS_PID_IDX].previousPIDTime;
		pid_x_integratedError = PID[RATE_XAXIS_PID_IDX].integratedError;
		pid_x_windupGuard = PID[RATE_XAXIS_PID_IDX].windupGuard;
	}
}				

/* ----------------------------------------------------------------------------
   Process calibrate ESC
   ---------------------------------------------------------------------------- */
void process_calibrate_esc(void)
{	

	if(calibrate_esc == ESC_CALIBRATION_START_MAX_THROTTLE)
	{
		motors_set_all_motors_to_command(MAXCOMMAND);
	}
	else if(calibrate_esc == ESC_CALIBRATION_TEST)
	{
		motors_set_all_motors_to_command(constrain(esc_calibration_test_command, MINCOMMAND, 1200));
	}
	else if(calibrate_esc == ESC_CALIBRATION_SEND_INDIVIDUAL_MOTOR_COMMANDS)
	{
		uint16_t motor0, motor1, motor2, motor3;
		motor0 = constrain(motor_configurator_command[MOTOR_0], MINCOMMAND, 1200);
		motor1 = constrain(motor_configurator_command[MOTOR_1], MINCOMMAND, 1200);
		motor2 = constrain(motor_configurator_command[MOTOR_2], MINCOMMAND, 1200);
		motor3 = constrain(motor_configurator_command[MOTOR_3], MINCOMMAND, 1200);
		
		motors_set_motors_to_commands(motor0, motor1, motor2, motor3);
		
		transmitter_state = ON;
	}
	else
	{
		motors_set_all_motors_to_command(MINCOMMAND);
	}
		
		// Send calibration commands to motors
		motors_write();

}

/* ----------------------------------------------------------------------------
   Process throttle correction
   ---------------------------------------------------------------------------- */
void process_throttle_correction(void)
{
	int throttleAdjust = 0;
	
	// Limit throttle to leave some space for motor correction in max throttle maneuver
	throttle = constrain((throttle + throttleAdjust), MINCOMMAND, MAXCOMMAND - 150);
}

/* ----------------------------------------------------------------------------
   Process hard maneuvers
   ---------------------------------------------------------------------------- */
void process_hard_maneuvers(void)
{

}

/* ----------------------------------------------------------------------------
   Process min/max commands
	- This function corrects too low/max throttle when maneuvering preventing 
	  some wobbling behavior
   ---------------------------------------------------------------------------- */
void process_min_max_motor_commands(void)
{
	// Set min and max motor command values
	for(MOTORS_t motor = MOTOR_0; motor < LASTMOTOR; motor++)
	{
		motor_min_command[motor] = min_armed_throttle;
		motor_max_command[motor] = MAXCOMMAND;
	}
	
	// Find the largest motor command
	int maxMotor = motor_command[MOTOR_0];
	for (MOTORS_t motor = MOTOR_1; motor < LASTMOTOR; motor++)
	{
		if ( motor_command[motor] > maxMotor )
		{
			maxMotor = motor_command[motor];
		}
	}
     
	// IF:	 	the largest motor command is greater than MAXCOMMAND
	// THEN:	subtract the amount we overflowed from each motor
	if( maxMotor > MAXCOMMAND )
	{
		for (MOTORS_t motor = MOTOR_0; motor < LASTMOTOR; motor++)
		{
			motor_command[motor] = motor_command[motor] - (maxMotor - MAXCOMMAND);
		}
	}
	
	
	// IF: 		throttle in minimum position, don't apply yaw
	if (receiver_command[THROTTLE] < MIN_THRESHOLD)
	{
		for (MOTORS_t motor = MOTOR_0; motor < LASTMOTOR; motor++) 
		{
			motor_min_command[motor] = min_armed_throttle;
			if ( in_flight && flight_mode == RATE_FLIGHT_MODE)
			{
				motor_max_command[motor] = MAXCOMMAND;
			}
			else
			{
				motor_max_command[motor] = min_armed_throttle;
			}
		}
	}
	
	
	// Apply limits to motor commands
	for (MOTORS_t motor = MOTOR_0; motor < LASTMOTOR; motor++) 
	{
		motor_command[motor] = constrain(motor_command[motor], motor_min_command[motor], motor_max_command[motor]);
	}
}

/* ----------------------------------------------------------------------------
   Process flight control
   ---------------------------------------------------------------------------- */
void process_flight_control(void)
{
	// TASK 1: Calculate flight error (update roll and pitch)
	calculate_flight_error();
	
	// TASK 2: Process heading (update yaw)
	process_heading();
	
	// TASK 3: Adjust throttle (50hz)
	//				- process position hold or navigation (not implemented)
	//				- process altitutde hold (not implemented)
	//				- process battery monitor hold (not implemented)
	//				- process auto-descent (not implemented)
	//				- process throttle correction
	if( app_execute & APP_EXECUTE_50_THROTTLE_bm )
	{
		throttle = receiver_command[THROTTLE];
		
		process_throttle_correction();
	}
	
	// TASK 4: Calculate motor commands
	if( motors_state == ON && transmitter_state == ON )
	{
		calculate_motor_commands();
	}
	
	// TASK 5: Process min/max motor commands
	
	process_min_max_motor_commands();
	
	// TASK 8: ESC calibration
	if(motors_state == OFF)
	{
		process_calibrate_esc();
	}
	
	// TASK 9: Command motors
	if(motors_state == ON && transmitter_state == ON)
	{
		motors_write();
	}
	
}



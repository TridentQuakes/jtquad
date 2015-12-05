/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "headingholdprocessor.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	double set_heading          = 0;
   
/* ================================================================================================
   Externs
   ================================================================================================ */
   
	extern uint8_t 	heading_hold_config;
	extern uint8_t	heading_hold_state;
	extern double 	heading;
	extern double 	relative_heading;
	extern int 		receiver_command[NUMBER_OF_CHANNELS];
	extern double	heading_hold;
	extern uint32_t heading_time;
	extern uint32_t current_time;
	extern int motor_axis_yaw_command;
	
	#ifdef MEGANEURA_DEBUG
	extern double g_rate_z;
	
	extern int m_axis_yaw_cmd;
	
	extern double r_SI_data_z;
	
	extern double cmd_yaw;
	
	extern double pid_z_P;
	extern double pid_z_I;
	extern double pid_z_D;
	extern double pid_z_lastError;
	extern double pid_z_previousPIDTime;
	extern double pid_z_integratedError;
	extern double pid_z_windupGuard;
	#endif
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Process heading hold
     - Calculate heading correction based on the users command
	 - Heading correction is calculated using the gyro or a magnetometer
   ---------------------------------------------------------------------------- */
void process_heading(void)
{
	if (heading_hold_config == ON)
	{
		#if defined(HMC5883L)
			heading = degrees( fpu_read_double_from_register(FPU_REG_MAG_TRUE_NORTH_HEADING) );
		#else
			heading = degrees( fpu_read_double_from_register(FPU_REG_GYRO_HEADING) );
		#endif

		// Always center relative heading around absolute heading chosen during yaw command
		// This assumes that an incorrect yaw can't be forced on the quad >180 or <-180 degrees
		// This is done so that quad does not accidentally hit transition between 0 and 360 or -180 and 180
		// AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
		// Doubt that will happen as it would have to be uncommanded.
		relative_heading = heading - set_heading;
		if (heading <= (set_heading - 180)) 
		{
		  relative_heading += 360;
		}
		if (heading >= (set_heading + 180)) 
		{
		  relative_heading -= 360;
		}

		// Apply heading hold only when throttle high enough to start flight
		
		// heading_hold_state:	- is the heading hold state ON or OFF
		//						- heading hold state gets turned ON only after 500ms have elapsed
		//                        since the last heading hold state (softens heading hold)
		// heading_hold: 		calculated adjustment for quad to go to heading (PID output)
		// heading_time:		used to soften heading hold by making sure at least 500ms have elapsed
		//						before turning ON heading hold state (applying new heading)
		// 
		// set_heading:			heading set by pilot via yaw command
		// relative_heading:	how off we are from the pilot's set_heading
		//  
		
		// IF: throttle above the minimum threshold to start flight
		//		IF: pilot commanding yaw 
		//			THEN: turn off heading hold
		//				  store the latest heading
		// 		ELSE:	
		//			IF: relative heading in between -0.25 and 0.25
		//				THEN: heading_hold adjustment equal to zero
		//					  HEADING_HOLD_PID.intetratedError equal zero
		//			ELSE IF: heading hold state is OFF and 500ms have elapsed since the last heading hold
		//				THEN: turn heading hold ON
		//					  store the latest heading
		//			ELSE: No new yaw input (heading hold state is ON and relative heading is not in the range of -0.25 to 0.25)
		//				THEN: heading hold PID update
		//					  update heading time
		if (receiver_command[THROTTLE] > MIN_THRESHOLD ) 
		{ 
			if ( (receiver_command[ZAXIS] > (MIDCOMMAND + 25) ) || (receiver_command[ZAXIS] < (MIDCOMMAND - 25)) ) 
			{
				// If commanding yaw, turn off heading hold and store latest heading
				set_heading = heading;
				heading_hold = 0;
				PID[HEADING_HOLD_PID_IDX].integratedError = 0;
				heading_hold_state = OFF;
				heading_time = current_time;
			}
			else 
			{
				if (relative_heading < 0.25 && relative_heading > -0.25) 
				{
					heading_hold = 0;
					PID[HEADING_HOLD_PID_IDX].integratedError = 0;
				}
				else if (heading_hold_state == OFF) 
				{ 
					// quick fix to soften heading hold on new heading
					if ((current_time - heading_time) > 500000) 
					{
						heading_hold_state = ON;
						heading_time = current_time;
						set_heading = heading;
						heading_hold = 0;
					}
				}
				else 
				{
					// No new yaw input, calculate current heading vs. desired heading heading hold
					// Relative heading is always centered around zero
					heading_hold = PID_update(0, relative_heading, &PID[HEADING_HOLD_PID_IDX]);
					heading_time = current_time; // quick fix to soften heading hold, wait 100ms before applying heading hold
				}
			}
		}
		else 
		{
			// minimum throttle not reached, use off settings
			set_heading = heading;
			heading_hold = 0;
			PID[HEADING_HOLD_PID_IDX].integratedError = 0;
		}
	}
	
	const double receiverSiData = receiver_get_SI_data(ZAXIS);
	
	const double commandedYaw = constrain(receiverSiData + radians(heading_hold), -PI, PI);
	
	double gyro_rate_z = fpu_read_double_from_register(FPU_REG_GYRO_FILTERED_Z);
	motor_axis_yaw_command = PID_update(commandedYaw, gyro_rate_z, &PID[ZAXIS_PID_IDX]);
	
}
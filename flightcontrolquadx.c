/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "flightcontrolquadx.h"

/* ================================================================================================
   EXterns
   ================================================================================================ */
   
	extern int motor_command[4];
    extern int throttle;
	
/* ================================================================================================
   Globals
   ================================================================================================ */
   
	// Flight control variables
	int motor_axis_roll_command = 0;
	int motor_axis_pitch_command = 0;
	int motor_axis_yaw_command = 0;
	
	int motor_configurator_command[4] = {0,0,0,0};
	int motor_max_command[4] = {0,0,0,0};
	int motor_min_command[4] = {0,0,0,0};
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Apply motor commands
   
	Quad X Motors Layout
	
			FRONT
			0   1
	LEFT	  x		RIGHT
			2	3
			BACK
		
	Yaw is applied by firing motors 1 and 3 faster than motors 0 and 2
   ---------------------------------------------------------------------------- */
void calculate_motor_commands(void)
{
	motor_command[FRONT_LEFT]  = throttle - motor_axis_pitch_command + motor_axis_roll_command - (YAW_DIRECTION * motor_axis_yaw_command);
	motor_command[FRONT_RIGHT] = throttle - motor_axis_pitch_command - motor_axis_roll_command + (YAW_DIRECTION * motor_axis_yaw_command);
	motor_command[REAR_LEFT]   = throttle + motor_axis_pitch_command + motor_axis_roll_command + (YAW_DIRECTION * motor_axis_yaw_command);
	motor_command[REAR_RIGHT]  = throttle + motor_axis_pitch_command - motor_axis_roll_command - (YAW_DIRECTION * motor_axis_yaw_command);
}
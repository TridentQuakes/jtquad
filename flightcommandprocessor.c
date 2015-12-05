/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "flightcommandprocessor.h"

/* ================================================================================================
   EXterns
   ================================================================================================ */
   
    extern int receiver_command[NUMBER_OF_CHANNELS];
    extern uint8_t 	motors_state;
    extern bool 		in_flight;
    extern uint8_t 	transmitter_state;
    extern int min_armed_throttle;
    extern uint8_t flight_mode;
    extern uint8_t 	previous_flight_mode;
	extern int motor_command[4];
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read pilot commands
   ---------------------------------------------------------------------------- */
void read_pilot_commands(void)
{
    // STEP 1: Read receiver commands
    receiver_read();

    // STEP 2: If the throttle stick is down (zero/grounded) then process functions from receiver command
    //         Functions are: arm motors, disarm motors, calibrate gyro and accel, and initialize GPS home waypoint
    if(receiver_command[THROTTLE] < MIN_THRESHOLD)
    {
        process_grounded_functions();
    }

    // STEP 3: If not in_flight state check throttle receiver_command[throttle] > min_armed_throttle
    if(!in_flight)
    {
        if(motors_state == ON && receiver_command[THROTTLE] > min_armed_throttle)
        {
            in_flight = true;
			#ifdef MEGANEURA_DEBUG
            usart_tx_string("in_flight = true\r\n");
			#endif
        }
    }

    // STEP 4: a. Check flying mode switch on transmitter:
    //				- Acro/Attitude
    //				- Stable/Rate
    //         b. If flight mode changes then zero integral error
	if (receiver_command[MODE] > 1500) 
    {
      flight_mode = ATTITUDE_FLIGHT_MODE;
    }
    else 
    {
      flight_mode = RATE_FLIGHT_MODE;
    }
    
    if (previous_flight_mode != flight_mode) 
    {
      zero_integral_error();
      previous_flight_mode = flight_mode;
      
      #ifdef MEGANEURA_DEBUG
      flight_mode == ATTITUDE_FLIGHT_MODE ? usart_tx_string("ATTITUDE_FLIGHT_MODE (acro)\r\n") : usart_tx_string("RATE_FLIGHT_MODE (stable)\r\n");
	  #endif
    }
}

void process_grounded_functions(void)
{
    // NOTE: We know that the throttle is down (zero) when it enters this function
    
    // FUNCTION: Disarm motors
    // 			 Left stick lower left corner
    //			 Right stick centered
    if( receiver_command[ZAXIS] < MIN_THRESHOLD && motors_state == ON )
    {
        usart_tx_string("FUNCTION: Disarm motors\r\n");
        
        motors_set_all_motors_to_command(MINCOMMAND);
        motors_state = OFF;
        in_flight = false;
		
		#ifdef MEGANEURA_DEBUG
		usart_tx_string("in_flight = false\r\n");
		#endif
    }
    
    // FUNCTION: Calibrate gyro and accel
    //			 Left stick lower left corner
    //			 Right stick lower right corner
    if( receiver_command[ZAXIS] < MIN_THRESHOLD &&
        receiver_command[XAXIS] > MAX_THRESHOLD && 
        receiver_command[YAXIS] < MIN_THRESHOLD )
    {
        usart_tx_string("FUNCTION: Calibrate gyro and accel\r\n");
        
        // Calibrate gyro and accel and store values in EEPROM
        while (!gyro_calibrate()) // this make sure the craft is still befor to continue init process
		{
		#ifdef MEGANEURA_DEBUG
			pc_send_string_resource(STRING_GYRO_CALIBRATION_FAIL);
		#endif
		}
        accel_compute_bias();
        
        // This function does nothing right now
        calibrate_kinematics();
        
        zero_integral_error();
        motors_pulse(3);
    }
    
    // FUNCTION: Arm motors
    //			 Left stick lower right corner
    //			 Right stick centered
    if( receiver_command[ZAXIS] > MAX_THRESHOLD && motors_state == OFF && transmitter_state == ON )
    {
        usart_tx_string("FUNCTION: Arm motors\r\n");
        
        for (MOTORS_t motor = 0; motor < LASTMOTOR; motor++) 
        {
            motor_command[motor] = MINTHROTTLE;
        }
        
        motors_state = ON;

        zero_integral_error();
    }
    
    // SAFETY CHECK: Prevents accidental arming of motor output if no transmitter command received
    if (receiver_command[ZAXIS] > MIN_THRESHOLD) 
    {
        transmitter_state = ON;
    }
}








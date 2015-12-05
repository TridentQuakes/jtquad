/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "motors_pwm.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	int motor_command[4] = {0,0,0,0};	
	
/* ================================================================================================
   Define
   ================================================================================================ */
   
   #define PWM_PRESCALER					1
   #define PWM_PHASE_CORRECT_FREQUENCY		500
   #define PERIOD							F_CPU / (2 * PWM_PRESCALER * PWM_PHASE_CORRECT_FREQUENCY)
   #define COMMAND_TO_DUTY_FACTOR			PERIOD / MAXCOMMAND
   
/* ================================================================================================
   Public functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize motors
   ---------------------------------------------------------------------------- */
void motors_initialize(void)
{
	initialize_hardware_pwm();
	
	motors_command_all(MINCOMMAND);
}

/* ----------------------------------------------------------------------------
   Command single motor
		- uint16_t command (compare value)
		- command ranges between 1000 to 2000 (i.e. 50% to 100% duty cycle)
   ---------------------------------------------------------------------------- */
void motors_command(MOTORS_t motor, uint16_t command)
{
	uint16_t compareValue = command * 2;
	
	// Output a new duty cycle. Set CCxBUF to 50-100% duty cycle. The new value 
	// will be valid from the next UPDATE condition).
	switch(motor)
	{
		case MOTOR_0:
			TC_SetCompareA( &TCF0, compareValue );
			break;
		case MOTOR_1:
			TC_SetCompareB( &TCF0, compareValue );
			break;
		case MOTOR_2:
			TC_SetCompareC( &TCF0, compareValue );
			break;
		case MOTOR_3:
			TC_SetCompareD( &TCF0, compareValue );
			break;
		default:
			break;
	}

	#if 0
	
	do {
		/*  Wait for the new compare value to be latched
		 *  from CCxBUF[H:L] to CCx[H:L]. This happens at
		 *  TC overflow (UPDATE ).
		 */
	} while( TC_GetOverflowFlag( &TCF0 ) == 0 );

	// Clear overflow flag
	TC_ClearOverflowFlag( &TCF0 );
	
	#endif
}
   
/* ----------------------------------------------------------------------------
   Apply a command to all motors at once
	- uint16_t command 
   ---------------------------------------------------------------------------- */
void motors_command_all(uint16_t command)
{
	motors_command(MOTOR_0, command);
	motors_command(MOTOR_1, command);
	motors_command(MOTOR_2, command);
	motors_command(MOTOR_3, command);
}

/* ----------------------------------------------------------------------------
   Pulse all motors
   ---------------------------------------------------------------------------- */
void motors_pulse(uint8_t number_of_pulses)
{
	for (uint8_t i = 0; i < number_of_pulses; i++)
	{
		motors_command_all(MIN_THRESHOLD); 	// 1105
		_delay_ms(200);
		motors_command_all(MINCOMMAND); 	// 1100
		_delay_ms(600);
	}
}

/* ----------------------------------------------------------------------------
   Write values from motor_command array to motors
   ---------------------------------------------------------------------------- */
void motors_write(void)
{
	motors_command(MOTOR_0, motor_command[MOTOR_0]);
	motors_command(MOTOR_1, motor_command[MOTOR_1]);
	motors_command(MOTOR_2, motor_command[MOTOR_2]);
	motors_command(MOTOR_3, motor_command[MOTOR_3]);
}

/* ----------------------------------------------------------------------------
   Set values in motor_command array to given command
   ---------------------------------------------------------------------------- */
void motors_set_all_motors_to_command(uint16_t command)
{
	motor_command[MOTOR_0] = command;
	motor_command[MOTOR_1] = command;
	motor_command[MOTOR_2] = command;
	motor_command[MOTOR_3] = command;
}

/* ----------------------------------------------------------------------------
   Set values in motor_command array to given commands
   ---------------------------------------------------------------------------- */
void motors_set_motors_to_commands(uint16_t command_motor0, uint16_t command_motor1, uint16_t command_motor2, uint16_t command_motor3)
{
	motor_command[MOTOR_0] = command_motor0;
	motor_command[MOTOR_1] = command_motor1;
	motor_command[MOTOR_2] = command_motor2;
	motor_command[MOTOR_3] = command_motor3;
}

/* ----------------------------------------------------------------------------
   Set value in motor_command array for given motor to given command
   ---------------------------------------------------------------------------- */
void motors_set_motor_command(MOTORS_t motor, uint16_t command)
{
	motor_command[motor] = command;
}

/* ================================================================================================
   Private functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Initialize hardware PWM
			- frequency of PWM is 8000hz
			- zero throttle is 50% duty cycle
			- full throttle is 100% duty cycle
			- dual slope, update BOTTOM (i.e. PER and CCx register are updated on BOTTOM)
   ---------------------------------------------------------------------------- */
void initialize_hardware_pwm(void)
{
	// 	PF0		Motor 0
	//	PF1		Motor 1
	//	PF2		Motor 2
	// 	PF3		Motor 3

	// Enable output on PC0
	PORT_SetPinsAsOutput( &PORTF, PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm );

	// Set the TC period (takes affect righ away), use TC_SetPeriodBuffered when you want it to take place only at UPDATE conditions
	TC_SetPeriod( &TCF0, 40000 ); // 40000 is 400hz with prescaler 1

	// Configure the TC for dual slope, update BOTTOM (i.e. update PER and CCx with PERBUF and CCxBUF values) (i.e. only update on a new cycle)
	TC0_ConfigWGM( &TCF0, TC_WGMODE_DS_B_gc );

	// Enable Compare channel A
	TC0_EnableCCChannels( &TCF0, TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm | TC0_CCDEN_bm );

	// Start timer by selecting a clock source, prescaler 1
	TC0_ConfigClockSource( &TCF0, TC_CLKSEL_DIV8_gc );
}

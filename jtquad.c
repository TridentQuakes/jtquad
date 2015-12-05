/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "jtquad.h"

/* ================================================================================================
   Interrupt Routines
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Overflow Interrupt Routine: 1s (1hz)
   ---------------------------------------------------------------------------- */
ISR(TCC0_OVF_vect)
{
	// Toggle LED
	PORT_TogglePins( &PORTH, 0x01 );
}

/* ----------------------------------------------------------------------------
   Overflow Interrupt Routine: 1024us (976.5625Hzhz)
   ---------------------------------------------------------------------------- */
ISR(TCC1_OVF_vect)
{
	// Used by the micros function
	++timer_tcc1_overflow_count;
}

/* ----------------------------------------------------------------------------
   Overflow Interrupt Routine: 1ms (1000hz)
   ---------------------------------------------------------------------------- */
/*ISR(TCD0_OVF_vect)
{
	// Used by the millis function
	timer_tcd0_overflow_count++;
}
*/

/* ----------------------------------------------------------------------------
   Overflow Interrupt Routine: (2.5ms) (400hz)
   ---------------------------------------------------------------------------- */
ISR(TCD1_OVF_vect)
{
	app_execute |= APP_EXECUTE_400_bm;			// 400 hz tasks true on every overflow
	
	uint32_t temp_50hz = timer_tcd1_overflow_count % 8;
	
	if(temp_50hz == 0)							// 50 hz, inline with 100hz tasks, for throttle correction process in flight control
	{
		app_execute |= APP_EXECUTE_50_THROTTLE_bm;
	}
	
	if(timer_tcd1_overflow_count % 4 == 0)		// 100 hz tasks
	{
		app_execute |= APP_EXECUTE_100_bm;
	}
	
	if(temp_50hz == 3)		// 50 hz tasks
	{
		
		app_execute |= APP_EXECUTE_50_bm;
	}
	
	if(timer_tcd1_overflow_count % 40 == 7)		// 10 hz tasks a
	{
		
		app_execute |= APP_EXECUTE_10A_bm;
	}
	
	if(timer_tcd1_overflow_count % 40 == 15)	// 10 hz tasks b
	{
		
		app_execute |= APP_EXECUTE_10B_bm;
	}
	
	// used for testing
	if(timer_tcd1_overflow_count % 40 == 0)	// 10 hz tasks b
	{
		
		app_execute |= APP_EXECUTE_10_bm;
	}
	
	// Used to control 400hz loop
	++timer_tcd1_overflow_count;
	
	/*if(timer_tcd1_overflow_count > 39)
	{
		timer_tcd1_overflow_count = 0;
	}*/
}

/* ================================================================================================
   Initialization Routines
   ================================================================================================ */

void initialize_clock_32mhz(void)
{
	// Enable the external oscillator, 16 MHz
	CLKSYS_XOSC_Config(OSC_FRQRANGE_12TO16_gc, false, OSC_XOSCSEL_XTAL_16KCLK_gc); 
	CLKSYS_Enable(OSC_XOSCEN_bm); 
	while(CLKSYS_IsReady(OSC_XOSCRDY_bm) == 0);

	// Enable the PLL with 2x multiplier, 32 MHz
	CLKSYS_PLL_Config(OSC_PLLSRC_XOSC_gc, 2); 
	CLKSYS_Enable(OSC_PLLEN_bm); 
	while(CLKSYS_IsReady(OSC_PLLRDY_bm) == 0); 
	CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_PLL_gc);

	// Disable the built-in 2 MHz clock source
	CLKSYS_Disable(OSC_RC2MEN_bm);
}

/*void initialize_clock_16mhz(void)
{
	CLKSYS_XOSC_Config( OSC_FRQRANGE_12TO16_gc, false, OSC_XOSCSEL_XTAL_16KCLK_gc );
	CLKSYS_Enable( OSC_XOSCEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc , CLK_PSBCDIV_1_1_gc );
	do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_XOSC_gc );
}*/

void initialize_io(void)
{
	// PH0 as output pin for flashing LED
	PORT_SetDirection(&PORTH, PIN0_bm);
	PORT_SetOutputValue( &PORTH, PIN0_bm);
	
	// These internal pullups where not good enough to TWI communication with ITG3200
	//PORTE.DIRSET |= 0x03;
	//PORTE.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	//PORTE.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
}

void initialize_defaults(void)
{
	fpu_init_eeprom_defaults();
}

void enable_interrupts(void)
{
	/* Enable low interrupt level in PMIC and enable global interrupts. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
}

void initialize_timers_32mhz(void)
{
	// ************************************************************************
	// Timer: 1s (1hz)
	// ************************************************************************
	TC_SetPeriod( &TCC0, 0x7a11 );
	TC0_SetOverflowIntLevel( &TCC0, TC_OVFINTLVL_LO_gc);
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1024_gc );
	
	// ************************************************************************
	// Timer for micros function
	// ************************************************************************
	//   - Overflow every 1024us (976.5625Hz)
	//   - 1024 was chosen for efficiency since it is a power of 2, micros can
	//     simply left shift 10 positions for multiplication
	TC_SetPeriod( &TCC1, 0x7fff );
	TC1_SetOverflowIntLevel( &TCC1, TC_OVFINTLVL_LO_gc);
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_DIV1_gc );
	
	// ************************************************************************
	// Timer for millis function
	// ************************************************************************
	//   - Overflow every 1ms (1000hz)
	//TC_SetPeriod( &TCD0, 0x7cff );
	//TC0_SetOverflowIntLevel( &TCD0, TC_OVFINTLVL_LO_gc);
	//TC0_ConfigClockSource( &TCD0, TC_CLKSEL_DIV1_gc );
	
	// ************************************************************************
	// Timer for main application
	// ************************************************************************
	//   - 400hz, 2.5ms
	TC_SetPeriod( &TCD1, 0x9c3f );
	TC1_SetOverflowIntLevel( &TCD1, TC_OVFINTLVL_LO_gc);
	TC1_ConfigClockSource( &TCD1, TC_CLKSEL_DIV2_gc );
	
}

void initialize_fpu(void)
{
	int ret = fpu_initialize();
	if( ERR_SUCCESS != ret )
	{
		report_error(ret);
	}
	else
	{
		bit_set_(fpu_flags, FPU_FLAG_IS_INIT);
		vehicle_state |= FPU_DETECTED_bp;
	}
	
	ret = fpu_load_constants();
	if( ERR_SUCCESS != ret )
	{
		report_error(ret);
	}
}

#ifdef HMC5883L
void initialize_mag(void)
{
	int ret = mag_initialize();
	if( ERR_SUCCESS != ret )
	{
		report_error(ret);
	}
	else
	{
		bit_set_(mag_flags, MAG_FLAG_IS_INIT);
		vehicle_state |= MAG_DETECTED_bp;
	}
}
#endif

void initialize_accel(void)
{
	int ret = accel_initialize();
	if( ERR_SUCCESS != ret )
	{
		report_error(ret);
	}
	else
	{
		bit_set_(accel_flags, ACCEL_FLAG_IS_INIT);
		vehicle_state |= ACCEL_DETECTED_bp;
	}
}

void initialize_gyro(void)
{
	int ret = gyro_initialize();
	if( ERR_SUCCESS != ret )
	{
		report_error(ret);
	}
	else
	{
		bit_set_(gyro_flags, GYRO_FLAG_IS_INIT);
		vehicle_state |= GYRO_DETECTED_bp;
	}
}

/* ================================================================================================
   Application Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Process Critical Sensors
     - Process critical sensors 4 times per 400hz loop
   ---------------------------------------------------------------------------- */
void process_critical_sensors()
{
	// Both sensors sample the data rates are 200 hz (1/2 sample rate)
	accel_read_vector_and_sum();
	gyro_read_vector_and_sum();
}

/* ----------------------------------------------------------------------------
   Process 100hz Tasks
     - Process accelerometer and run data through the low pass filter
	 - Process the gyroscope
	 - Calculate the vehicle attitude (kinematics)
	 - Process flight control
   ---------------------------------------------------------------------------- */
void process_100hz_tasks(void)
{
	// STEP1: Time elapsed since the last time we executed this task
	G_Dt = (current_time - previous_time_100hz_tasks) / 1000000.0;
	previous_time_100hz_tasks = current_time;

	#ifdef MEGANEURA_DEBUG
	temp_time = micros();
	#endif			
	// STEP2: Use accumulated samples to calculate average of samples and then apply 1g scale factor and a runtime bias (offset) 
	accel_evaluate_ms2();
	#ifdef MEGANEURA_DEBUG
	time_accel_eval_ms2 = micros() - temp_time;
	#endif
			
	#ifdef MEGANEURA_DEBUG
	temp_time = micros();
	#endif	
	// STEP3: Run accel data through low pass filter
	compute_fourth_order();
	#ifdef MEGANEURA_DEBUG
	time_fourth_order = micros() - temp_time;
	#endif
	
	#ifdef MEGANEURA_DEBUG
	temp_time = micros();
	#endif
	// STEP4: Evaluate the rotational velocity using the gyro
	gyro_evaluate_rate();
	#ifdef MEGANEURA_DEBUG
	time_gyro_evaluate_rate = micros() - temp_time;
	#endif
	
	#ifdef MEGANEURA_DEBUG
	temp_time = micros();
	#endif	
	// STEP5: Get description of rotation of quadcopter
	calculate_kinematics();
	#ifdef MEGANEURA_DEBUG
	time_calc_kinematics = micros() - temp_time;
	#endif
	
	#ifdef MEGANEURA_DEBUG
	temp_time = micros();
	#endif
	// STEP6: Process flight control
	process_flight_control();
	#ifdef MEGANEURA_DEBUG
	time_process_flight_control = micros() - temp_time;
	#endif	
}

/* ----------------------------------------------------------------------------
   Process 50hz Tasks
     - Read pilot commands
   ---------------------------------------------------------------------------- */
void process_50hz_tasks(void)
{
	G_Dt = (current_time - previous_time_50hz_tasks) / 1000000.0;
	previous_time_50hz_tasks = current_time;
	
	// Read the pilot's commands and perform grounded functions based on stick configuration (mode 2)
	read_pilot_commands();
}

/* ----------------------------------------------------------------------------
   Process 10hz Tasks 1
     - Process the magnetometer and calculate the true north heading
	 - Magnetic declination not applied at this time so true north heading
	   represents magnetics north and not true north
   ---------------------------------------------------------------------------- */
void process_10hz_tasks_1(void)
{
	G_Dt = (current_time - previous_time_10hz_tasks) / 1000000.0;
	previous_time_10hz_tasks = current_time;
	
	#ifdef HMC5883L
	mag_measure();
	calculate_heading();
	#endif
}

/* ----------------------------------------------------------------------------
   Process 10hz Tasks 2 (low priority tasks 1)
     - Read serial commands
	 - Send serial telemetry
   ---------------------------------------------------------------------------- */
void process_10hz_tasks_2(void)
{
	G_Dt = (current_time - previous_time_10hz_tasks_low_priority_1) / 1000000.0;
	previous_time_10hz_tasks_low_priority_1 = current_time;
	
	// Read serial commands
	if( ERR_SUCCESS != process_serial_commands() )
	{
		//report_error(ret);
	}
	
	// Send telemetry data
	send_serial_telemetry();
}

/* ================================================================================================
   Main Loop
     - Max speed when processing accelerometer was approximately 2500hz
	 - Main loop runs at 400hz
	 - One frame is 1s and frame counter is incremented at 100hz
   ================================================================================================ */
void main_loop(void)
{	
	for(;;)
	{
		#ifdef MEGANEURA_DEBUG
		app_events = 0;
		uint32_t time_400=0, time_100=0, time_50=0, time_10a=0, time_10b=0;
		uint32_t time_400_ts=0, time_100_ts=0, time_50_ts=0, time_10a_ts=0, time_10b_ts=0;
		#endif
		
		current_time = micros();
		heading_time = micros();
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			app_execute_buf = app_execute;
		}
		
		// ********************************************************************
		// 400Hz task loop (2.5ms)
		// ********************************************************************
		if( app_execute_buf & APP_EXECUTE_400_bm )
		{
			//app_execute &= ~APP_EVENT_400_bm;
			
			#ifdef MEGANEURA_DEBUG
			//bit_set_(app_events, APP_EVENT_400_bp);
			app_events |= APP_EVENT_400_bm;
			#endif
			//previous_time_400hz = current_time;
			
			#ifdef MEGANEURA_DEBUG
			//usart_tx_string("400");
			//time_critical_tasks_start = micros();
			time_400_ts = micros();
			#endif
			
			process_critical_sensors();
			#ifdef MEGANEURA_DEBUG
			//time_critical_tasks = micros() - time_critical_tasks_start;
			time_400 = micros() - time_400_ts;
			#endif
		}
		
		// ********************************************************************
		// 100Hz task loop (10ms) (100 frames per second)
		// ********************************************************************
		if( app_execute_buf & APP_EXECUTE_100_bm )
		{	
			//app_execute &= ~APP_EVENT_100_bm;
			
			#ifdef MEGANEURA_DEBUG
			//bit_set_(app_events, APP_EVENT_100_bp);
			app_events |= APP_EVENT_100_bm;
			#endif
			//frame_counter++;
			
			#ifdef MEGANEURA_DEBUG
			//usart_tx_string("100");
			//time_100hz_tasks_start = micros();
			time_100_ts = micros();
			#endif
			process_100hz_tasks();
			#ifdef MEGANEURA_DEBUG
			//time_100hz_tasks = micros() - time_100hz_tasks_start;
			time_100_ts = micros() - time_100_ts;
			#endif
		}
		
		// ================================================================
		// 50Hz task loop (20ms)
		// ================================================================
		if ( app_execute_buf & APP_EXECUTE_50_bm )
		{
			//app_execute &= ~APP_EXECUTE_50_bm;
			
			#ifdef MEGANEURA_DEBUG
			//bit_set_(app_events, APP_EVENT_50_bp);
			app_events |= APP_EVENT_50_bm;
			//usart_tx_string("50");
			//time_50hz_tasks_start = micros();
			time_50_ts = micros();
			#endif
			process_50hz_tasks();
			#ifdef MEGANEURA_DEBUG
			//time_50hz_tasks = micros() - time_50hz_tasks_start;
			time_50 = micros() - time_50_ts;
			#endif
		}
			
		// ****************************************************************
		// 10Hz task loop (100ms)
		// ****************************************************************
		if ( app_execute_buf & APP_EXECUTE_10A_bm )
		{ 
			//app_execute &= ~APP_EXECUTE_10A_bm;
			
			#ifdef MEGANEURA_DEBUG
			//bit_set_(app_events, APP_EVENT_10A_bp);
			app_events |= APP_EVENT_10A_bm;
			//usart_tx_string("10a");
			//time_10ahz_tasks_start = micros();
			time_10a_ts = micros();
			#endif
			process_10hz_tasks_1();
			#ifdef MEGANEURA_DEBUG
			//time_10ahz_tasks = micros() - time_10ahz_tasks_start;
			time_10a = micros() - time_10a_ts;
			#endif
		}
		
		if( app_execute_buf & APP_EXECUTE_10B_bm )
		{
			//app_execute &= ~APP_EXECUTE_10B_bm;
			
			#ifdef MEGANEURA_DEBUG
			//bit_set_(app_events, APP_EVENT_10B_bp);
			app_events |= APP_EVENT_10B_bm;
			//usart_tx_string("10b");
			//time_10bhz_tasks_start = micros();
			time_10b_ts = micros();
			#endif
			process_10hz_tasks_2();
			#ifdef MEGANEURA_DEBUG
			//time_10bhz_tasks = micros() - time_10bhz_tasks_start;
			time_10b = micros() - time_10b_ts;
			#endif
		}
		
		
		
		// Wait until all FPU instructions have been processed over the FPU's 256 byte buffer will overflow.
		// NOTE: In the future more waits may be needed if there are more than 256 bytes of instructions per loop.

		
		#ifdef MEGANEURA_DEBUG
		// update loop times with the latest
		if( app_execute_buf & APP_EXECUTE_400_bm )
		{
			time_critical_tasks = time_400;
		}
		if( app_execute_buf & APP_EXECUTE_100_bm )
		{
			time_100hz_tasks = time_100;
		}
		if( app_execute_buf & APP_EXECUTE_50_bm )
		{
			time_50hz_tasks = time_50;
		}
		if( app_execute_buf & APP_EXECUTE_10A_bm )
		{
			time_10ahz_tasks = time_10a;
		}
		if( app_execute_buf & APP_EXECUTE_10B_bm )
		{
			time_10bhz_tasks = time_10b;
		}
		
		if( (app_execute_buf & APP_EXECUTE_400_bm) &&
		    (app_execute_buf & APP_EXECUTE_100_bm) )
		{
			loop_time = micros() - current_time;
		}
		
		#if 0
		if(app_events)
		{
			usart_tx_string((app_events & APP_EVENT_400_bm)? "X\t" : " \t");
			usart_tx_string((app_events & APP_EVENT_100_bm)? "X\t" : " \t");
			usart_tx_string((app_events & APP_EVENT_50_bm)? "X\t" : " \t");
			usart_tx_string((app_events & APP_EVENT_10A_bm)? "X\t" : " \t");
			usart_tx_string((app_events & APP_EVENT_10B_bm)? "X\t" : " \t");
			usart_tx_ul(loop_time);
			usart_tx_string("\t");
			usart_tx_ul(current_time);
			usart_tx_string(LINE_END);
		}
		#endif
		
		
		
		// Clear tasks that we executed in this loop
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			app_execute &= ~app_execute_buf;
		}
		#endif
	}
}

/* ================================================================================================
   Main
   ================================================================================================ */
int main(void)
{
	initialize_clock_32mhz();
	initialize_io();
	
	usart_initialize();
	twi_initialize();
	
	initialize_defaults();
	
	receiver_initialize();
	motors_initialize();
	
	enable_interrupts();
	
	_delay_ms(100);
	
	// ************************************************************************
	// EEPROM
	// ************************************************************************
	#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_APP_READING_EEPROM);
	#endif
	ds_initialize();
	ds_read_eeprom_defaults();
	
	// Check if it's a first time boot
	bool is_first_time_boot = false;
	if(eeprom_read_double(SOFTWARE_VERSION_ADR) != VERSION_NUMBER)
	{
		#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_APP_IS_FIRST_TIME_BOOT);
		#endif
		ds_initialize_eeprom_defaults();
		ds_write_eeprom_defaults();
		is_first_time_boot = true;
	}
	
#ifdef MEGANEURA_DEBUG
	pc_send_version();
#endif
	
	// ************************************************************************
	// Receiver
	// ************************************************************************
	
	// initialize_receiver_from_eeprom();
	
	// ************************************************************************
	// FPU
	// ************************************************************************
	initialize_fpu();
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_FPU_INIT);
#endif
	
	
	// ************************************************************************
	// FPU EEPROM
	// ************************************************************************
	if(is_first_time_boot)
	{
		fpu_load_eeprom_data();
	}
	
	// ************************************************************************
	// Gyroscope
	// ************************************************************************
	initialize_gyro();
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_GYRO_INIT);
#endif
	gyro_reset_sum();
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_GYRO_CALIBRATION_START);
#endif
	while (!gyro_calibrate()) // this make sure the craft is still befor to continue init process
	{
#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_GYRO_CALIBRATION_FAIL);
#endif
	}
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_GYRO_CALIBRATION_COMPLETE);
#endif
	
	// ************************************************************************
	// Accelerometer, EEPROM functions, low pass filter and kinematics
	// ************************************************************************
	initialize_accel();
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_ACCEL_INIT);
#endif
	accel_reset_sum();

	if(is_first_time_boot)
	{
		#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_ACCEL_CALIBRATION_START);
		#endif
		
		accel_compute_bias(); 	// take 400 samples at 400hz and then calculate bias (offset)

		#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_ACCEL_CALIBRATION_COMPLETE);
		#endif
	}
	
	init_fourth_oder();
	
	initialize_kinematics();
	
#ifdef HMC5883L
	// ************************************************************************
	// Magnetometer
	// ************************************************************************
	initialize_mag(); // Assumes the craft is flat
	initialize_heading_fusion();
	
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_MAG_INIT);
#endif	
#endif
	
	
	// ************************************************************************
	// Timers
	// ************************************************************************
	initialize_timers_32mhz();
	
	previous_time_400hz = previous_time = micros();
	
	transmitter_state = OFF;

	#ifdef HMC5883L
	if( bit_test_(fpu_flags, FPU_FLAG_IS_INIT) && 
		bit_test_(mag_flags, MAG_FLAG_IS_INIT) &&
		bit_test_(accel_flags, ACCEL_FLAG_IS_INIT) &&
		bit_test_(gyro_flags, GYRO_FLAG_IS_INIT) )
	#else
	if( bit_test_(fpu_flags, FPU_FLAG_IS_INIT) && 
		bit_test_(accel_flags, ACCEL_FLAG_IS_INIT) &&
		bit_test_(gyro_flags, GYRO_FLAG_IS_INIT) )
	#endif
	{
		main_loop();
	}
	else
	{
		// Error initializing a peripheral device
		//report_error();
#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_INIT_ERROR);
#endif
	}
}

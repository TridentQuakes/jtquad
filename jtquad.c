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
	// Toggle LED on port H
	PORT_TogglePins( &PORTH, 0x01 );
}

/* ----------------------------------------------------------------------------
   Overflow Interrupt Routine: 1024us (976.5625hz)
   ---------------------------------------------------------------------------- */
ISR(TCC1_OVF_vect)
{
	// Used by the micros function
	++timer_tcc1_overflow_count;
}

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
	
	// Increment timer overflow count
	++timer_tcd1_overflow_count;
}

/* ================================================================================================
   Initialization Routines
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize clocks
   ---------------------------------------------------------------------------- */
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

/* ----------------------------------------------------------------------------
   Initiliaze IO
   
   // NOTE: The xmega internal pullups where not good enough for TWI to ITG3200 
            communication.
   ---------------------------------------------------------------------------- */
void initialize_io(void)
{
	// PH0 as output pin for flashing LED
	PORT_SetDirection(&PORTH, PIN0_bm);
	PORT_SetOutputValue( &PORTH, PIN0_bm);
	
	
}

/* ----------------------------------------------------------------------------
   Initialize defaults
   ---------------------------------------------------------------------------- */
void initialize_defaults(void)
{
	fpu_init_eeprom_defaults();
}

/* ----------------------------------------------------------------------------
   Enable Interrupts
   ---------------------------------------------------------------------------- */
void enable_interrupts(void)
{
	// Enable low interrupt level
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	
	// Enable global interrupts
	sei();
}

/* ----------------------------------------------------------------------------
   Initialize Timers
   
     - Initialize the system timers assuming MCU running at 32mhz
   ---------------------------------------------------------------------------- */
void initialize_timers_32mhz(void)
{
	// ************************************************************************
	// Timer: 1s (1hz)
	// ************************************************************************
	TC_SetPeriod( &TCC0, 0x7a11 );
	TC0_SetOverflowIntLevel( &TCC0, TC_OVFINTLVL_LO_gc);
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1024_gc );
	
	// ************************************************************************
	// Timer: 1024us (976.5625hz)
	// ************************************************************************
	//   - Used by micros() functions
	//   - 1024us chosen for efficiency, power of 2, left/right shift for 
	//     multiplicaiton/division
	TC_SetPeriod( &TCC1, 0x7fff );
	TC1_SetOverflowIntLevel( &TCC1, TC_OVFINTLVL_LO_gc);
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_DIV1_gc );
	
	// ************************************************************************
	// Timer: 2.5 ms (400hz)
	// ************************************************************************
	//   - Used to control the main loop speed
	TC_SetPeriod( &TCD1, 0x9c3f );
	TC1_SetOverflowIntLevel( &TCD1, TC_OVFINTLVL_LO_gc);
	TC1_ConfigClockSource( &TCD1, TC_CLKSEL_DIV2_gc );
}

/* ----------------------------------------------------------------------------
   Initialize FPU
   
     - umFPU v3.1
   ---------------------------------------------------------------------------- */
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

/* ----------------------------------------------------------------------------
   Initialize Magnetometer (HMC5843 or HMC5883)
   ---------------------------------------------------------------------------- */
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

/* ----------------------------------------------------------------------------
   Initialize accelerometer (ADXL345)
   ---------------------------------------------------------------------------- */
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

/* ----------------------------------------------------------------------------
   Initialize gyroscope (ITG3200)
   ---------------------------------------------------------------------------- */
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
     - This gets called at a rate of 400hz
	 - Sum up as many sample as possible before the next round of calculations,
	   which is 4 samples since the calculations are done at a rate of 100hz
	 - NOTE: Acceleromter and gyroscope are currently sampling at 200hz
   ---------------------------------------------------------------------------- */
void process_critical_sensors()
{
	accel_read_vector_and_sum();
	gyro_read_vector_and_sum();
}

/* ----------------------------------------------------------------------------
   Process 100hz Tasks
     - Process accelerometer and run data through the low pass filter (lpf)
	 - Process the gyroscope
	 - Calculate the vehicle attitude (kinematics)
	 - Process flight control
   ---------------------------------------------------------------------------- */
void process_100hz_tasks(void)
{
	// STEP1: Time elapsed since the last time we executed this task
	G_Dt = (current_time - previous_time_100hz_tasks) / 1000000.0;
	previous_time_100hz_tasks = current_time;
		
	// STEP2: Use accumulated samples to calculate average of samples and 
	//        then apply 1g scale factor and a runtime bias (offset) 
	accel_evaluate_ms2();
	
	// STEP3: Run accel data through low pass filter
	compute_fourth_order();
	
	// STEP4: Evaluate the rotational velocity using the gyro
	gyro_evaluate_rate();
		
	// STEP5: Get description of rotation of quadcopter
	calculate_kinematics();
	
	// STEP6: Process flight control
	process_flight_control();	
}

/* ----------------------------------------------------------------------------
   Process 50hz Tasks
     - Read pilot commands
   ---------------------------------------------------------------------------- */
void process_50hz_tasks(void)
{
	G_Dt = (current_time - previous_time_50hz_tasks) / 1000000.0;
	previous_time_50hz_tasks = current_time;
	
	// Read the pilot's commands and perform grounded functions based on stick
	// configuration (mode 2)
	read_pilot_commands();
}

/* ----------------------------------------------------------------------------
   Process 10hz Tasks 1
     - Process the magnetometer and calculate the true north heading
	 - Magnetic declination not applied at this time so true north heading
	   represents magnetic north and not true north
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
     - Read user's serial commands
	 - Perform serial telemetry tasks
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
     - Loop runs at 400hz
	 - Process tasks at 400, 100, 50 and 10 hz
		- 400: process critical sensors (accelerometer and gyroscope)
		- 100: Do calculations
		- 50: Read pilot commands
		- 10: Sample magnetometer, calculate heading, send/receive serial data  
     
	 NOTE: Max speed when processing accelerometer was approximately 2500hz
   ================================================================================================ */
void main_loop(void)
{	
	for(;;)
	{
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
			process_critical_sensors();
		}
		
		// ********************************************************************
		// 100Hz task loop (10ms) (100 frames per second)
		// ********************************************************************
		if( app_execute_buf & APP_EXECUTE_100_bm )
		{	
			process_100hz_tasks();
		}
		
		// ================================================================
		// 50Hz task loop (20ms)
		// ================================================================
		if ( app_execute_buf & APP_EXECUTE_50_bm )
		{
			process_50hz_tasks();
		}
			
		// ****************************************************************
		// 10Hz task loop (100ms)
		// ****************************************************************
		if ( app_execute_buf & APP_EXECUTE_10A_bm )
		{ 
			process_10hz_tasks_1();
		}
		
		if( app_execute_buf & APP_EXECUTE_10B_bm )
		{
			process_10hz_tasks_2();
		}
		
		// Wait until all FPU instructions have been processed or the FPU's 256 byte buffer
		// will overflow. NOTE: In the future more fpu 'waits' may be required
	}
}

/* ================================================================================================
   Main
   ================================================================================================ */
int main(void)
{
	// ************************************************************************
	// General Init
	// ************************************************************************
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
	// MCU EEPROM
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

	// This makes sure the quadcopter is still before beginning any calibration
	while (!gyro_calibrate())
	{
#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_GYRO_CALIBRATION_FAIL);
#endif
	}
	
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_GYRO_CALIBRATION_COMPLETE);
#endif
	
	// ************************************************************************
	// Accelerometer, low pass filter and kinematics
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
		
		// take 400 samples at 400hz and then calculate bias (offset)
		accel_compute_bias();

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
	// Assumes the craft is flat
	initialize_mag();
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
#ifdef MEGANEURA_DEBUG
		pc_send_string_resource(STRING_INIT_ERROR);
#endif
	}
}

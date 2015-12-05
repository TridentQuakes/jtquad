/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "serialcommunications.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
	
	uint8_t queryType = 'X';

/* ================================================================================================
   Externs
   ================================================================================================ */
   
    extern uint8_t 	motors_state;
    extern uint8_t calibrate_esc;
	extern uint16_t esc_calibration_test_command;
	extern USART_data_t USART_data;
    extern int motor_configurator_command[4];
	extern uint8_t 	flight_mode;
	extern double rotation_speed_factor;
	
/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Process serial commands
   ---------------------------------------------------------------------------- */
int process_serial_commands(void)
{
	if( USART_RXBufferData_Available(&USART_data) )
	{
		queryType = USART_RXBuffer_GetByte(&USART_data);
		
		switch(queryType)
		{
			case 'A': 		// Read roll/pitch rate mode PID
				pc_read_rate_mode_PID();
				break;
			case 'B':		// Read roll/pitch attitude mode PID
				pc_read_attitude_mode_PID();
				break;
			case 'C':		// Read yaw PID
				pc_read_yaw_pid();
				break;
			case 'D':		// Altitude hold PID (not implemented)
				break;
			case 'E':		// Read sensor filtering values
				pc_read_min_armed_throttle();
				//aref = readFloatSerial();
				break;
			case 'F':		// Read transmitter smoothing values
				pc_read_receiver_smoothing_values();
				break;
			case 'G': 		// Receive transmitter calibration: slope value
				pc_read_channel_slope();
				break;
			case 'H': 		// Receive transmitter calibration: slope value
				pc_read_channel_offset();
				break;
			case 'I':		// Initialize EEPROM with default values
				pc_initialize_eeprom_to_defaults();
				break;
			case 'J':		// Calibrate gyro (writes offset to FPU EEPROM)
				pc_calibrate_gyro();
				break;
			case 'K':		// Read accel scale (x,y,z) and calculate accel offset values
				pc_calibrate_accel();
				break;
			case 'L':
				accel_compute_bias(); // Store new accel offset values to FPU EEPROM for us
				break;
			#ifdef HMC5883L
			case 'M':
				pc_read_mag_offset();
				break;
			#endif
			case 'W':		// Write all user configurable values to EEPROM
				ds_write_eeprom_defaults();
				zero_integral_error();
				break;
			case 'X':		// Stop sending messages command received
				break;
			case '1':		// Start ESC calibration by setting throttle to max on all motor channels
				validate_calibrate_esc_command(1);
				break;
			case '2':		// ESC calibration, part two, set throttle to min on all motor channels
				validate_calibrate_esc_command(2);
				break;
			case '3':		// Test ESC calibration by setting a test_command on all motor channels
				if( validate_calibrate_esc_command(3) )
				{
					esc_calibration_test_command = (int) usart_rx_double();
				}
				break;
			case '4':		// Stop ESC calibration
				if( validate_calibrate_esc_command(4) )
				{
					calibrate_esc = ESC_CALIBRATION_IDLE;
					esc_calibration_test_command = MINCOMMAND;
				}
				break;
			case '5':		// Set individual motor commands
				if( validate_calibrate_esc_command(5) )
				{
					motor_configurator_command[0] = (int) usart_rx_double();
					motor_configurator_command[1] = (int) usart_rx_double();
					motor_configurator_command[2] = (int) usart_rx_double();
					motor_configurator_command[3] = (int) usart_rx_double();
				}
				break;
		}
	}
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Send serial telemetry
   ---------------------------------------------------------------------------- */
void send_serial_telemetry(void)
{
	switch(queryType)
	{
		case '=': // Reserved debug command to view any variable from Serial Monitor
			#ifdef MEGANEURA_DEBUG
			pc_send_loop_time();
			#endif
			
			//motors_pulse(3);
			//motors_command(MOTOR_0, 1500);
			break;
		case 'a': // Send roll and pitch rate mode PID values
			pc_send_rate_mode_PID();
			queryType = 'X';
			break;
		case 'b': // Send roll and pitch attitude mode PID values
			pc_send_attitude_mode_PID();
			queryType = 'X';
			break;
		case 'c': // Send yaw PID values
			pc_send_yaw_PID();
			queryType = 'X';
			break;
		case 'e': // Miscellaneous configuration values
			// pc_send_aref();
			pc_send_min_armed_throttle();
			queryType = 'X';
			break;
		case 'f':
			pc_send_transmitter_smoothing_values();
			queryType = 'X';
			break;
		case 'g':
			pc_send_transmitter_slope();
			queryType = 'X';
			break;
		case 'h':
			pc_send_transmitter_offset();
			queryType = 'X';
			break;
		case 'i': // Send sensor data
			// format:		"gyrox,gyroy,gyroz,accelx,accely,accelz,magx,magy,magz,\r\n"
			//				usart_tx_string("0,0,0,0,0,0,0,0,0,\r\n");
			pc_send_gyro_filtered_vector();
			pc_send_accel_filtered_vector();
			#ifdef HMC5883L
			pc_send_mag_measured_data();
			#endif
			usart_tx_string(LINE_END);
			break;
		#ifdef HMC5883L
		case 'j': // Send raw mag data
			pc_send_mag_raw_data();
			break;
		#endif
		case 'k': // Send accelerometer calibration values
			pc_send_accel_calibration_data();
			queryType = 'X';
			break;
		case 'l': // Send raw accelerometer values
			pc_send_accel_raw_vector();
			break;
		#ifdef HMC5883L
		case 'm': // Send the magnetometer calibration values
			pc_send_mag_calibration_data();
			queryType = 'X';
			break;
		#endif
		case 'n': // Send the magnetometer calibration values
			pc_send_gyro_calibration_data();
			queryType = 'X';
			break;
		case 'q':
			pc_send_vehicle_state();
			queryType = 'X';
			break;
		case 'r': // Vehicle attitude
			pc_send_vehicle_attitude();
			break;
		case 's': // All flight data
			pc_send_all_flight_data();
			break;
		case 't':
			pc_send_transmitter_data(true);
			break;
		case 'x': // Stop sending messages
			break;
		case '!': // Send flight software version
			//usart_tx_string("3.2");
			usart_tx_string("3.2\r\n");
			queryType = 'X';
			break;
		case '#': // Send configuration
			//reportVehicleState();
			//fool_aq_config();
			pc_send_vehicle_configuration();
			queryType = 'X';
			break;
		case '6': // Report remote commands
			pc_send_motor_commands();
			usart_tx_string(LINE_END);
			//queryType = 'X';
			break;
		#ifdef MEGANEURA_DEBUG
		case '7': // JAT: print flight control debugging values
			pc_send_flight_control_debug_values();
			usart_tx_string(LINE_END);
			//queryType = 'X';
			break;
		case '8':
			if(flight_mode == RATE_FLIGHT_MODE)
			{
				flight_mode = ATTITUDE_FLIGHT_MODE;
			}
			else
			{
				flight_mode = RATE_FLIGHT_MODE;
			}
			queryType = 'X';
			break;
		#endif
		/*#ifdef HMC5883L
		case 't': // Send absolute heading (used for testing)
			pc_send_mag_absolute_heading();
			break;
		case 'u': // Send mag measured (x,y) and mag tilt compensated (x,y) (used for offset calculation)
			pc_send_mag_offset_calculation_data();
			break;
		#endif*/
		default:
			break;
	}

}

/* ----------------------------------------------------------------------------
   Fools AQ Configurator version 3.2
   ---------------------------------------------------------------------------- */
void fool_aq_config(void)
{
  // Tell Configurator how many vehicle state values to expect
  usart_tx_string("15\r\n");//("1\r\n");
  
  usart_tx_string("Software Version: 3.2\r\n");
  usart_tx_string("Board Type: Mega v21\r\n");
  usart_tx_string("Flight Config: Quad X\r\n");
  usart_tx_string("Receiver Channels: 8\r\n");
  usart_tx_string("Motors: 4\r\n");
  usart_tx_string("Gyroscope: Detected\r\n");
  usart_tx_string("Accelerometer: Detected\r\n");
  usart_tx_string("Barometer: Not Detected\r\n");
  usart_tx_string("Magnetometer: Not Detected\r\n");
  usart_tx_string("Heading Hold: Not Detected\r\n");
  usart_tx_string("Altitude Hold: Not Detected\r\n");
  usart_tx_string("Battery Monitor: Not Detected\r\n");
  usart_tx_string("Camera Stability: Not Detected\r\n");
  usart_tx_string("Range Detection: Not Detected\r\n");
  usart_tx_string("GPS: Not Enabled\r\n");
}

/* ----------------------------------------------------------------------------
   Validate the ESC calibration command
   ---------------------------------------------------------------------------- */
bool validate_calibrate_esc_command(uint8_t command)
{
	// Use a specific double value to validate max throttle call is being sent
	if( usart_rx_double() == 123.45 )
	{
		motors_state = OFF;
		calibrate_esc = command;
		return true;
	}
	else
	{
		calibrate_esc = ESC_CALIBRATION_IDLE;
		esc_calibration_test_command = MINCOMMAND;
		return false;
	}
}



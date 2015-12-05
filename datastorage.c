/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "datastorage.h"

/* ================================================================================================
   Extern
   ================================================================================================ */
   
   extern double 		receiver_transmit_factor;
   extern int			min_armed_throttle;
   extern double 		receiver_slope[NUMBER_OF_CHANNELS];
   extern double 		receiver_offset[NUMBER_OF_CHANNELS];
   extern double 		receiver_smooth_factor[NUMBER_OF_CHANNELS];
   extern uint8_t 		flight_mode;
   extern double 		rotation_speed_factor;
   extern uint8_t 		heading_hold_config;

/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize EEPROM
   ---------------------------------------------------------------------------- */
void ds_initialize(void)
{
	/* flush buffer just to be sure when we start.*/
	EEPROM_FlushBuffer();
	
	/* disable NVM since we are using I/O mapped access and not memory mapped access */
	EEPROM_DisableMapping();
}

/* ----------------------------------------------------------------------------
   Initialize eeprom defaults
   ---------------------------------------------------------------------------- */
void ds_initialize_eeprom_defaults(void)
{
	PID[RATE_XAXIS_PID_IDX].P = 100.0;
	PID[RATE_XAXIS_PID_IDX].I = 150.0;
	PID[RATE_XAXIS_PID_IDX].D = -350.0;
	PID[RATE_YAXIS_PID_IDX].P = 100.0;
	PID[RATE_YAXIS_PID_IDX].I = 150.0;
	PID[RATE_YAXIS_PID_IDX].D = -350.0;
	PID[ATTITUDE_XAXIS_PID_IDX].P = 3.5;
	PID[ATTITUDE_XAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_XAXIS_PID_IDX].D = 0.0;
	PID[ATTITUDE_YAXIS_PID_IDX].P = 3.5;
	PID[ATTITUDE_YAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_YAXIS_PID_IDX].D = 0.0;
	PID[ZAXIS_PID_IDX].P = 200.0;
	PID[ZAXIS_PID_IDX].I = 5.0;
	PID[ZAXIS_PID_IDX].D = 0.0;
	PID[HEADING_HOLD_PID_IDX].P = 3.0;
	PID[HEADING_HOLD_PID_IDX].I = 0.1;
	PID[HEADING_HOLD_PID_IDX].D = 0.0;
	// AKA PID experiements
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].P = 100.0;
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_GYRO_XAXIS_PID_IDX].D = -350.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].P = 100.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].I = 0.0;
	PID[ATTITUDE_GYRO_YAXIS_PID_IDX].D = -350.0;
	
	rotation_speed_factor = 1.0;
	
	// *******************************************************
	// WIND UP GUARD
	// *******************************************************
	
	wind_up_guard = 1000.0;
	
	// AKA - added so that each PID has its own windupGuard, will need to be removed once each PID's range is established and put in the eeprom
	for (uint8_t i = XAXIS; i < LAST_PID_IDX; i++ ) 
	{
		PID[i].windupGuard = wind_up_guard;
	}
	
	// Integral Limit for attitude mode
    // Set for 1/2 max attitude command (+/-0.75 radians)
    // Rate integral not used for now
	PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
	PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
	
	// *******************************************************
	// RECEIVER
	// *******************************************************
	receiver_transmit_factor = 1;
	min_armed_throttle = 1105;
	
	//accelOneG = -9.80665;
	
	for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{ 
		receiver_slope[channel] = 1;
		receiver_offset[channel] = 0;
		receiver_smooth_factor[channel] = 1;
	}
	receiver_smooth_factor[ZAXIS] = 0.5;
	
	receiver_offset[XAXIS] = 		-30;
	receiver_offset[YAXIS] = 		-27;
	receiver_offset[ZAXIS] = 		-25.5;//29.5
	receiver_offset[THROTTLE] = 	-29;
	receiver_offset[MODE] =			0;
	receiver_offset[AUX1] =			0;
	receiver_offset[AUX2] =			0;
	// ******************************************************
	
	// *******************************************************
	// GENERAL
	// *******************************************************
	flight_mode = RATE_FLIGHT_MODE;
	//flight_mode = ATTITUDE_FLIGHT_MODE;
	heading_hold_config = OFF;

}

/* ----------------------------------------------------------------------------
   Read defaults from EEPROM
   ---------------------------------------------------------------------------- */
void ds_read_eeprom_defaults(void)
{
	eeprom_read_PID(RATE_XAXIS_PID_IDX, 			ROLL_PID_GAIN_ADR);
	eeprom_read_PID(RATE_YAXIS_PID_IDX, 			PITCH_PID_GAIN_ADR);
	eeprom_read_PID(ZAXIS_PID_IDX, 					YAW_PID_GAIN_ADR);
	eeprom_read_PID(ATTITUDE_XAXIS_PID_IDX, 		LEVELROLL_PID_GAIN_ADR);
	eeprom_read_PID(ATTITUDE_YAXIS_PID_IDX, 		LEVELPITCH_PID_GAIN_ADR);
	eeprom_read_PID(HEADING_HOLD_PID_IDX, 			HEADING_PID_GAIN_ADR);
	eeprom_read_PID(ATTITUDE_GYRO_XAXIS_PID_IDX, 	LEVEL_GYRO_ROLL_PID_GAIN_ADR);
	eeprom_read_PID(ATTITUDE_GYRO_YAXIS_PID_IDX, 	LEVEL_GYRO_PITCH_PID_GAIN_ADR);
	
	rotation_speed_factor = eeprom_read_double(ROTATION_SPEED_FACTOR_ADR);
	
	wind_up_guard = eeprom_read_double(WINDUP_GUARD_ADR);
	receiver_transmit_factor = eeprom_read_double(TRANSMIT_FACTOR_ADR);
	
	// Read receiver calibration data, offset, slope and smooth factor
	for (RECEIVER_CHANNELS_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
	{ 
		receiver_slope[channel] = eeprom_read_double(RECEIVER_DATA[channel].slope);
		receiver_offset[channel] = eeprom_read_double(RECEIVER_DATA[channel].offset);
		receiver_smooth_factor[channel] = eeprom_read_double(RECEIVER_DATA[channel].smooth_factor);
	}
	
	min_armed_throttle = eeprom_read_double(MIN_ARMED_THROTTLE_ADR);
	flight_mode = eeprom_read_double(FLIGHTMODE_ADR);
	heading_hold_config = eeprom_read_double(HEADINGHOLD_ADR);
}



/* ----------------------------------------------------------------------------
   Write defaults to EEPROM
   ---------------------------------------------------------------------------- */
void ds_write_eeprom_defaults(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// ******************************************************
		// NOTE: umFPU v3.1 EEPROM stores:
		//		- accel offset (x,y,z) and scale (x,y,z)
		//		- gyro offset (x,y,z) and scale (scale)
		//		- mag offset (x,y,z) and scale (x,y,z)
		//		- const negative one G (-9.80665)
		//		- cheby2 parameters (b0,b1,b2,b3,b4,a1,a2,a3,a4)
		// ******************************************************
		
		eeprom_write_PID(RATE_XAXIS_PID_IDX, 			ROLL_PID_GAIN_ADR);
		eeprom_write_PID(RATE_YAXIS_PID_IDX, 			PITCH_PID_GAIN_ADR);
		eeprom_write_PID(ATTITUDE_XAXIS_PID_IDX, 		LEVELROLL_PID_GAIN_ADR);
		eeprom_write_PID(ATTITUDE_YAXIS_PID_IDX, 		LEVELPITCH_PID_GAIN_ADR);
		eeprom_write_PID(ZAXIS_PID_IDX, 				YAW_PID_GAIN_ADR);
		eeprom_write_PID(HEADING_HOLD_PID_IDX, 			HEADING_PID_GAIN_ADR);
		eeprom_write_PID(ATTITUDE_GYRO_XAXIS_PID_IDX,	LEVEL_GYRO_ROLL_PID_GAIN_ADR);
		eeprom_write_PID(ATTITUDE_GYRO_YAXIS_PID_IDX, 	LEVEL_GYRO_PITCH_PID_GAIN_ADR);
		
		eeprom_write_double(ROTATION_SPEED_FACTOR_ADR, rotation_speed_factor);
		
		eeprom_write_double(WINDUP_GUARD_ADR, wind_up_guard);
		eeprom_write_double(TRANSMIT_FACTOR_ADR, receiver_transmit_factor);
		
		// Write receiver calibration data, offset, slope and smooth factor
		for (uint8_t channel = XAXIS; channel < NUMBER_OF_CHANNELS; channel++) 
		{ 
			eeprom_write_double(RECEIVER_DATA[channel].slope, receiver_slope[channel]);
			eeprom_write_double(RECEIVER_DATA[channel].offset, receiver_offset[channel]);
			eeprom_write_double(RECEIVER_DATA[channel].smooth_factor, receiver_smooth_factor[channel]);
		}
		
		eeprom_write_double(MIN_ARMED_THROTTLE_ADR, min_armed_throttle);
		//writeFloat(aref, AREF_ADR);
		eeprom_write_double(FLIGHTMODE_ADR, flight_mode);
		eeprom_write_double( HEADINGHOLD_ADR, heading_hold_config);
		//writeFloat(accelOneG, ACCEL_1G_ADR);
		eeprom_write_double(SOFTWARE_VERSION_ADR, VERSION_NUMBER);
  
	}
}

/* ----------------------------------------------------------------------------
   Read int to EEPROM
   ---------------------------------------------------------------------------- */
int ds_read_eeprom_int(uint8_t page_address, uint8_t byte_address)
{
	WORD value;
	value.byte1 = EEPROM_ReadByte(page_address, byte_address);
	value.byte0 = EEPROM_ReadByte(page_address, byte_address + 1);
	
	return value._int;
}

/* ----------------------------------------------------------------------------
   Read double to EEPROM
   ---------------------------------------------------------------------------- */
double ds_read_eeprom_double(uint8_t page_address, uint8_t byte_address)
{
	DWORD value;
	value.byte3 = EEPROM_ReadByte(page_address, byte_address);
	value.byte2 = EEPROM_ReadByte(page_address, byte_address + 1);
	value.byte1 = EEPROM_ReadByte(page_address, byte_address + 2);
	value.byte0 = EEPROM_ReadByte(page_address, byte_address + 3);
	
	return value._double;
}

/* ----------------------------------------------------------------------------
   Write int to EEPROM
   ---------------------------------------------------------------------------- */
void ds_write_eeprom_int(uint8_t page_address, uint8_t byte_address, int value)
{
	WORD temp;
	temp._int = value;
	EEPROM_WriteByte(page_address, byte_address, temp.byte1);
	EEPROM_WriteByte(page_address, byte_address + 1, temp.byte0);
}

/* ----------------------------------------------------------------------------
   Write double to EEPROM
   ---------------------------------------------------------------------------- */
void ds_write_eeprom_double(uint8_t page_address, uint8_t byte_address, double value)
{
	DWORD temp;
	temp._double = value;
	EEPROM_WriteByte(page_address, byte_address, temp.byte3);
	EEPROM_WriteByte(page_address, byte_address + 1, temp.byte2);
	EEPROM_WriteByte(page_address, byte_address + 2, temp.byte1);
	EEPROM_WriteByte(page_address, byte_address + 3, temp.byte0);
}

/* ----------------------------------------------------------------------------
   Read PID to EEPROM
	- IDpid		idx in PID[] array
	- IDEeprom	eeprom address from t_NVR_Data
   ---------------------------------------------------------------------------- */
void ds_read_PID(uint8_t IDPid, int IDEeprom) 
{
	struct PIDdata* pid = &PID[IDPid];
	pid->P = ds_read_eeprom_double(IDEeprom / 32, IDEeprom % 32);
	pid->I = ds_read_eeprom_double(IDEeprom + 4 / 32, IDEeprom + 4 % 32);
	pid->D = ds_read_eeprom_double(IDEeprom + 8 / 32, IDEeprom + 8 % 32);
	pid->lastError = 0;
	pid->integratedError = 0;
}

/* ----------------------------------------------------------------------------
   Write PID to EEPROM
	- IDpid		idx in PID[] array
	- IDEeprom	eeprom address from t_NVR_Data
   ---------------------------------------------------------------------------- */
void ds_write_PID(uint8_t IDPid, int IDEeprom) 
{
	struct PIDdata* pid = &PID[IDPid];
	ds_write_eeprom_double(IDEeprom / 32, IDEeprom % 32, pid->P);
	ds_write_eeprom_double(IDEeprom + 4 / 32, IDEeprom + 4 % 32, pid->I);
	ds_write_eeprom_double(IDEeprom + 8 / 32, IDEeprom + 8 % 32, pid->D);
}
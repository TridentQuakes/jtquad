/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */


/* eeprom specifications */
/* size: 2048 bytes */
/* page size: 32 bytes */
/* page indices range from 0 to (EEPROM_SIZE / EEPROM_PAGE_SIZE ) - 1 = 63 */
/* totale number of pages: 64 */

#ifndef __DATASTORAGE_H__
#define __DATASTORAGE_H__

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "common.h"

/* ================================================================================================
   Defines
   ================================================================================================ */
   
	#define NUMBER_OF_CHANNELS						7

/* ================================================================================================
   Typedefs
   ================================================================================================ */

	typedef struct
	{
		double p;
		double i;
		double d;
	} t_NVR_PID;

	typedef struct 
	{
		double slope;
		double offset;
		double smooth_factor;
	} t_NVR_Receiver;

	// accel, gyro, mag values stored in fpu eeprom, not here
	typedef struct
	{
		t_NVR_PID ROLL_PID_GAIN_ADR;
		t_NVR_PID LEVELROLL_PID_GAIN_ADR;
		t_NVR_PID YAW_PID_GAIN_ADR;
		t_NVR_PID PITCH_PID_GAIN_ADR;
		t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
		t_NVR_PID HEADING_PID_GAIN_ADR;
		t_NVR_PID LEVEL_GYRO_ROLL_PID_GAIN_ADR;
		t_NVR_PID LEVEL_GYRO_PITCH_PID_GAIN_ADR;

		t_NVR_Receiver RECEIVER_DATA[NUMBER_OF_CHANNELS];

		
		double ROTATION_SPEED_FACTOR_ADR;
		double WINDUP_GUARD_ADR;
		double TRANSMIT_FACTOR_ADR;
		double MIN_ARMED_THROTTLE_ADR;
		//double AREF_ADR;
		double FLIGHTMODE_ADR;
		double HEADINGHOLD_ADR;
		
		double SOFTWARE_VERSION_ADR;
		
		double ACCEL_1G_ADR;
		
		// Battery Monitor
		//double BATT_ALARM_VOLTAGE_ADR;
		//double BATT_THROTTLE_TARGET_ADR;
		//double BATT_DOWN_TIME_ADR;
	} t_NVR_Data;
		
/* ================================================================================================
   Macros
   ================================================================================================ */
	
	#define GET_NVR_ADDRESS(param)				((int)&(((t_NVR_Data*) 0)->param))
	#define GET_NVR_OFFSET(param) 				((int)&(((t_NVR_Data*) 0)->param)) % 32
	#define GET_NVR_PAGE_NUMBER(param) 			((int)&(((t_NVR_Data*) 0)->param)) / 32

	#define eeprom_read_double(addr) 			ds_read_eeprom_double(GET_NVR_PAGE_NUMBER(addr), GET_NVR_OFFSET(addr))
	#define eeprom_read_int(addr) 				ds_read_eeprom_int(GET_NVR_PAGE_NUMBER(addr), GET_NVR_OFFSET(addr))
	#define eeprom_write_double(addr, value)	ds_write_eeprom_double(GET_NVR_PAGE_NUMBER(addr), GET_NVR_OFFSET(addr), value)
	#define eeprom_write_int(addr, value)		ds_write_eeprom_int(GET_NVR_PAGE_NUMBER(addr), GET_NVR_OFFSET(addr), value)
	//#define readLong(addr) nvrReadLong(GET_NVR_OFFSET(addr))
	//#define writeLong(value, addr) nvrWriteLong(value, GET_NVR_OFFSET(addr))
	#define eeprom_read_PID(IDPid, addr)		ds_read_PID(IDPid, GET_NVR_ADDRESS(addr))
	#define eeprom_write_PID(IDPid, addr)		ds_write_PID(IDPid, GET_NVR_ADDRESS(addr))
		
/* ================================================================================================
   Function prototypes
   ================================================================================================ */
   
	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif

	/* Public Interface - May be used in end-application: */

	void ds_initialize(void);
	
	void ds_initialize_eeprom_defaults(void);
	void ds_write_eeprom_defaults(void);
	void ds_read_eeprom_defaults(void);
	
	int ds_read_eeprom_int(uint8_t page_address, uint8_t byte_address);
	double ds_read_eeprom_double(uint8_t page_address, uint8_t byte_address);
	
	void ds_write_eeprom_double(uint8_t page_address, uint8_t byte_address, double value);
	void ds_write_eeprom_int(uint8_t page_address, uint8_t byte_address, int value);

	void ds_read_PID(uint8_t IDPid, int IDEeprom);
	void ds_write_PID(uint8_t IDPid, int IDEeprom);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif

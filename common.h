/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _COMMON_H_
#define _COMMON_H_

/* ================================================================================================
   Main compilation defines and version information
   ================================================================================================ */
		
	// Compilation configuration
	#define MEGANEURA_DEBUG		// serial data will be outputted
		
	// Version
	#define VERSION_STRING 		"1.0.8"
	#define VERSION_NUMBER 		1.08

/* ================================================================================================
   AVR libc
   ================================================================================================ */
   
	#include <string.h>
	#include <stdio.h>
	#include <math.h>
	#include <stdbool.h>
	#include <stdint.h>
	#include <stdlib.h>
	#include <util/atomic.h>
	#include <avr/eeprom.h>
	
/* ================================================================================================
   Typedefs
   ================================================================================================ */
	
	typedef struct MN_Time_Struct{
		uint32_t 		millisecond;
		uint32_t		microsecond;
	} MN_Time_t;

	typedef struct Vector2 { 
		int x; 
		int y; 
	} Vector2;
	
	typedef struct Vector3 { 
		int x; 
		int y;
		int z;
	} Vector3;
	
	typedef struct VectorDouble2 { 
		double x; 
		double y; 
	} VectorDouble2;
	
	typedef struct VectorDouble3 { 
		double x; 
		double y;
		double z;
	} VectorDouble3;
	
	typedef enum External_Command_Mode_enum
	{
	   INTERFACE_NONE = 0,
	   INTERFACE_PC
	} External_Command_Mode_t;
	
	typedef struct FourthOrderData_Struct
	{
	  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
	  float outputTm1, outputTm2, outputTm3, outputTm4;
	} FourthOrderData_t;
	
	typedef enum RECEIVER_CHANNELS_enum
	{
		XAXIS = 0,
		YAXIS = 1, 
		ZAXIS = 2,
		THROTTLE = 3,
		MODE = 4,
		AUX1 = 5, 
		AUX2 = 6
	} RECEIVER_CHANNELS_t;
	
	typedef enum MOTORS_enum
	{
		MOTOR_0 = 0,
		MOTOR_1 = 1,
		MOTOR_2 = 2,
		MOTOR_3 = 3
	} MOTORS_t;

/* ================================================================================================
   XMega includes - core MCU drivers (atxmega128a1)
   ================================================================================================ */

	#include "xmega/avr_compiler.h"
	#include "xmega/adc_driver.h"
	#include "xmega/clksys_driver.h"
	#include "xmega/twi_master_driver.h"
	#include "xmega/twi_slave_driver.h"
	#include "xmega/usart_driver.h"
	#include "xmega/port_driver.h"
	#include "xmega/rtc_driver.h"
	#include "xmega/spi_driver.h"
	#include "xmega/TC_driver.h"
	#include "xmega/eeprom_driver.h"
	#include "xmega/event_system_driver.h"

/* ================================================================================================
   Application includes 
   ================================================================================================ */
   
	#include "stringresources.h"
	#include "usart.h"
	#include "twi.h"
	#include "spi.h"
	#include "error.h"
	#include "fpu.h"
	#include "pcportal.h"
	#include "numeric_typedefs.h"
	#ifdef HMC5883L
	#include "hmc5883L.h"
	#endif
	#include "itg3200.h"
	#include "adxl345.h"
	#include "utilities.h"
	#include "lpf.h"
	
	#include "kinematics_arg.h"
	#include "heading_fusion_processor_marg.h"
	#include "receiver.h"
	
	
	#include "flightcommandprocessor.h"
	#include "flightcontrolprocessor.h"
	#include "headingholdprocessor.h"

	#include "motors_pwm.h"
	
	#include "flightcontrolquadx.h"
	
	#include "datastorage.h"
	#include "serialcommunications.h"
	
	#include "PID.h"
	#include "userconfiguration.h"
	
	
/* ================================================================================================
   Application defines
   ================================================================================================ */
	
	// General
	#define CSV_DELIMITER							","
	#define LINE_END 								"\x0D\x0A"
	
	#define PI          							3.1415926535897932384626433832795
	//#define HALF_PI     							1.5707963267948966192313216916398
	//#define TWO_PI      							6.283185307179586476925286766559
	#define DEG_TO_RAD  							0.017453292519943295769236907684886
	#define RAD_TO_DEG 								57.295779513082320876798154814105
	
	// Application
	#define MN_FLAG_PROCESS_EXTERNAL_COMMANDS		0
	#define MN_FLAG_SEND_EXTERNAL_DATA				1


	#define TEMP_FLAG_SEND_DATA 					0
	#define TEMP_FLAG_DATA_READY					1
	#define TEMP_FLAG_SEND_MIN_MAX_DATA				2

	#define MAG_FLAG_IS_INIT 						0
	#define MAG_FLAG_DATA_READY						1
	#define MAG_FLAG_SEND_DATA 						2
	#define MAG_FLAG_SEND_OFFSET					3
	#define MAG_FLAG_PROCESS_DATA_REQUESTS			4

	#define ACCEL_FLAG_IS_INIT 						0
	#define ACCEL_FLAG_DATA_READY					1
	#define ACCEL_FLAG_SEND_DATA 					2

	#define GYRO_FLAG_IS_INIT 						0
	#define GYRO_FLAG_DATA_READY					1
	#define GYRO_FLAG_SEND_DATA 					2

	#define FPU_FLAG_IS_INIT						0

	#define OFF										0
	#define ON										1

	//#define LASTMOTOR 								4

	#define RATE_FLIGHT_MODE 						0
	#define ATTITUDE_FLIGHT_MODE 					1
	
	#define TASK_100HZ 								1
	#define TASK_50HZ 								2
	#define TASK_10HZ 								10
	#define TASK_1HZ 								100
	#define THROTTLE_ADJUST_TASK_SPEED 				TASK_50HZ
	
	// Vehicle states (peripheral status) -- bit positions
	#define	GYRO_DETECTED_bp		 				0x001;
	#define ACCEL_DETECTED_bp		 				0x002;
	#define	MAG_DETECTED_bp			 				0x004;
	//#define	BARO_DETECTED_bp		 			0x008;
	//#define	HEADING_HOLD_ENABLED_bp 			0x010;
	//#define	ALTITUDE_HOLD_ENABLED_bp 			0x020;
	//#define	BATTERY_MONITOR_ENABLED_bp			0x040;
	//#define	CAMERA_STABLE_ENABLED_bp 			0x080;
	//#define	RANGE_ENABLED_bp		 			0x100;
	#define	FPU_DETECTED_bp		 					0x200;
	
	
	#define APP_EXECUTE_400_bp							0
	#define	APP_EXECUTE_400_bm							0b00000001
	#define APP_EXECUTE_100_bp							1
	#define	APP_EXECUTE_100_bm							0b00000010
	#define APP_EXECUTE_50_bp							2
	#define	APP_EXECUTE_50_bm							0b00000100
	#define APP_EXECUTE_10A_bp							3
	#define	APP_EXECUTE_10A_bm							0b00001000
	#define APP_EXECUTE_10B_bp							4
	#define	APP_EXECUTE_10B_bm							0b00010000
	#define APP_EXECUTE_50_THROTTLE_bp					5					// part of flight control 100hz task, every other time 50hz
	#define	APP_EXECUTE_50_THROTTLE_bm					0b00100000
	
	#define APP_EXECUTE_10_bp							6					// used for testing
	#define	APP_EXECUTE_10_bm							0b01000000			// used for testing
	
	#define 	ESC_CALIBRATION_IDLE									0
	#define		ESC_CALIBRATION_START_MAX_THROTTLE						1
	#define		ESC_CALIBRATION_MIN_THROTTLE							2
	#define		ESC_CALIBRATION_TEST									3
	#define		ESC_CALIBRATION_STOP									4
	#define		ESC_CALIBRATION_SEND_INDIVIDUAL_MOTOR_COMMANDS			5
	
/* ================================================================================================
   Macros
   ================================================================================================ */
	
		#define radians(deg)            				((deg)*DEG_TO_RAD)
		#define degrees(rad)            				((rad)*RAD_TO_DEG)
		
		#define min(a,b)                				((a)<(b)?(a):(b))
		#define max(a,b)                				((a)>(b)?(a):(b))
		#define constrain(amt,low,high) 				((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
		#define round(x)                				((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
		#define radians(deg)            				((deg)*DEG_TO_RAD)
		#define degrees(rad)            				((rad)*RAD_TO_DEG)
		#define sq(x)                   				((x)*(x))

	
		/** Macro for setting a bit.
        *
        * \param[in]  x 8 bit register.
        * \param[in]  y Bit position.
			 */
		#define   bit_set_(x,y)      x |= 1<<y
		
		/** Macro for clearing a bit.
        *
        * \param[in]  x 8 bit register.
        * \param[in]  y Bit position.
			 */
		#define   bit_clear_(x,y)      x &= ~(1<<y)
		
		/** Macro for testing a bit.
        *
        * \param[in]  x 8 bit register.
        * \param[in]  y Bit position.
        * \return     Returns 0 or 1.
        * \note       !! is used to return a 1 and not an int greater or equal to 1.
			 */
		#define   bit_test_(x,y)      (!!(x & (1<<y)))
		
		/** Macro for testing a bit.
        *
        * \param[in]  x 8 bit register.
        * \param[in]  y Bit position.
			 */
		#define bit_toggle_(x,y)			(x ^= (1<<y));

#endif
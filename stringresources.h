/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _STRINGRESOURCES_H_
#define _STRINGRESOURCES_H_

/* ================================================================================================
   Defines
   ================================================================================================ */
	
	#define STRING_WELCOME						"JT Quad Version "
	#define STRING_FPU_INIT						"um-FPU v3.1 initialization complete"
	#define STRING_ACCEL_INIT					"ADXL345 initialization complete"
	#define STRING_GYRO_INIT					"ITG3200 initialization complete"
	#define STRING_TMP102_INIT					"TMP102 initialization complete"
	#define STRING_MAG_INIT						"HMC5883L initialization complete"
	#define STRING_INIT_ERROR					"An error occurred during the initialization process"
	
	#define STRING_GYRO_CALIBRATION_START		"ITG3200 calibration start (will not complete until gyro is stable) ..."
	#define STRING_GYRO_CALIBRATION_COMPLETE	"ITG3200 calibration complete"
	#define STRING_ACCEL_CALIBRATION_START		"ADXL345 calibration start ..."
	#define STRING_ACCEL_CALIBRATION_COMPLETE	"ADXL345 calibration complete"
	
	#define STRING_GYRO_BIAS					"ITG3200 (bias x, bias y, bias z, scale factor):"
	#define STRING_GYRO_CALIBRATION_FAIL		"ITG3200 calibration function failed due to movement. Keep the board stable"
	
	#define STRING_APP_INIT_COMPLETE			"JT Quad initialization complete"
	#define STRING_APP_COMMAND_LIST				"Command List\r\ni = send sensor data (gyroxyz, accelxyz, magxyz,)\r\nj = send mag raw data\r\nk = send accelerometer calibration values\r\nl = send accelerometer raw data\r\nm = send magnetometer calibration data\r\nn = gyroscope calibration data\r\nr = send vehicle attitude\r\ns = send all flight data\r\nX = stop sending data"
	
	#define STRING_APP_READING_EEPROM			"Reading the defaults from EEPROM"
	#define STRING_APP_IS_FIRST_TIME_BOOT		"First time boot so we will write the default settings to EEPROM"

#endif

/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __ITG3200_H__
#define __ITG3200_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "common.h"
	
/* ================================================================================================
   ITG3200 defines
   ================================================================================================ */
   
	#define ITG3200_SLAVE_ADDRESS 		0x69 // or 0x69 if pin 9 (ADO) is high, solder jumper on sparkfun board
	
	// ITG3200 Register Defines
	#define WHO_AM_I				0x00
	#define	SAMPLE_RATE_DIVIDER		0x15
	#define DLPF					0x16
	#define INT_CFG					0x17
	#define INT_STATUS				0x1A
	#define	TMP_H					0x1B
	#define	TMP_L					0x1C
	#define	GX_H					0x1D
	#define	GX_L					0x1E
	#define	GY_H					0x1F
	#define	GY_L					0x20
	#define GZ_H					0x21
	#define GZ_L					0x22
	#define PWR_MGM					0x3E
	
	#define GYRO_SMOOTH_FACTOR 		1.0
	#define GYROSCOPE_GAIN 			(1.0 / 14.375)	//ITG-3200 sensitivity is 14.375 LSB/(degrees/sec)
	#define GYRO_FULL_SCALE_OUTPUT	2000.0			// +/- 2000 deg/sec

/* ================================================================================================
   ITG3200 macros
   ================================================================================================ */
   
	#define ITG3200_RATE(r) 		(1000 / r) - 1	// Sample rate macro

/* ================================================================================================
   Application defines
   ================================================================================================ */
   
	#define GYRO_CALIBRATION_TRESHOLD 		4		// Threshold of +-difference allowed in ADC gyro values during calibration process (i.e. sensitivity to movement)
	#define GYRO_NUM_SAMPLES_BIAS 			49		// Take 49 samples for each axis during calibration process

/* ================================================================================================
   Function prototypes
   ================================================================================================ */
   
	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	// Initiliazation functions
	int gyro_initialize(void);
	
	// Measurement functions
	void gyro_read_vector_and_sum(void);
	void gyro_evaluate_rate(void);
	void gyro_reset_sum(void);
	void write_gyro_reset_sum_function_to_fpu_eeprom(void);
	
	// Calibration function
	int findMedianIntWithDiff(int *data, int arraySize, int * diff);
	bool gyro_calibrate(void);
	int gyro_read_vector(void);
	int gyro_read_vector_to_local_ints(int *pX, int *pY, int *pZ);
	
	// Gyroscope temperature (not used)
	void gyro_read_temperature(void);
	
	// Data filtering (not used)
	double smooth(double current_data, double previous_data, double smooth_factor);
	
	// TWI (I2C) communication
	int gyro_twi_read(uint8_t reg, uint8_t *pValue);
	int gyro_twi_write(uint8_t reg, uint8_t value);
	
	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif
	
#endif

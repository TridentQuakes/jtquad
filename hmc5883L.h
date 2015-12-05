/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __HMC5883L_H__
#define __HMC5883L_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"
	
/* ================================================================================================
   HMC5883L Defines
   ================================================================================================ */
	
	// General
	#define HMC5883L_10HZ_WAIT 	100
	#define HMC5883L_SLAVE_ADDRESS    0x1E

	// Registers
	#define HMC5883L_CONFIG_A 0x00
	#define HMC5883L_CONFIG_B 0x01
	#define HMC5883L_MODE 0x02
	#define HMC5883L_X_MSB 0x03
	#define HMC5883L_X_LSB 0x04
	#define HMC5883L_Y_MSB 0x05
	#define HMC5883L_Y_LSB 0x06
	#define HMC5883L_Z_MSB 0x07
	#define HMC5883L_Z_LSB 0x08
	#define HMC5883L_STATUS 0x09
	#define HMC5883L_ID_A 0x0A
	#define HMC5883L_ID_B 0x0B
	#define HMC5883L_ID_C 0x0C

	// Configuration Register A Options
	#define HMC5883L_DATA_OUT_50HZ 0x18
	#define HMC5883L_DATA_OUT_20HZ 0x14
	#define HMC5883L_DATA_OUT_10HZ 0x10
	#define HMC5883L_DATA_OUT_NORMAL 0x00
	#define HMC5883L_DATA_OUT_POS_BIAS 0x01
	#define HMC5883L_DATA_OUT_NEG_BIAS 0x02

	// Configuration Register B Options
	#define HMC5883L_GAIN_0_7_GA 0x00
	#define HMC5883L_GAIN_1_GA 0x20
	#define HMC5883L_GAIN_1_5_GA 0x40
	#define HMC5883L_GAIN_2_GA 0xC0
	#define HMC5883L_GAIN_3_2_GA 0x80
	#define HMC5883L_GAIN_3_8_GA 0xA0
	#define HMC5883L_GAIN_4_5_GA 0xE0

	// Mode Register Options
	#define HMC5883L_MODE_CONTINUOUS 0x00
	#define HMC5883L_MODE_SINGLE 0x01
	#define HMC5883L_MODE_IDLE 0x02
	#define HMC5883L_MODE_SLEEP 0x03

	// Status Register Result Bits
	#define HMC5883L_REGULATOR_ENABLED_BIT 0x04
	#define HMC5883L_DATA_OUTPUT_REGISTER_LOCK 0x02
	#define HMC5883L_READY_BIT 0x01

	// Self-Test Target Value
	#define HMC5883L_SELFTEST_TARGET_1_0_GA 		715 // 715 = 0.55 ga * 1300 adc units

/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif

	int mag_initialize(void);
	int mag_selftest(void);
	int mag_measure(void);
	int mag_read_vector(void); // not called since we are using single measurement mode
	
	int mag_twi_read(uint8_t reg, uint8_t *pValue);
	int mag_twi_write(uint8_t reg, uint8_t value);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif
		
#endif

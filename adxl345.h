/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
 
#ifndef __ADXL345_H__
#define __ADXL345_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"
	
/* ================================================================================================
   Defines
   ================================================================================================ */
		
		// ADXL345 Register names
		#define ADXL345_DEVID			0x00
		#define ADXL345_RESERVED1		0x01
		#define ADXL345_THRESH_TAP		0x1d
		#define ADXL345_OFSX			0x1e
		#define ADXL345_OFSY			0x1f
		#define ADXL345_OFSZ			0x20
		#define ADXL345_DUR				0x21
		#define ADXL345_LATENT			0x22
		#define ADXL345_WINDOW			0x23
		#define ADXL345_THRESH_ACT		0x24
		#define ADXL345_THRESH_INACT	0x25
		#define ADXL345_TIME_INACT		0x26
		#define ADXL345_ACT_INACT_CTL	0x27
		#define ADXL345_THRESH_FF		0x28
		#define ADXL345_TIME_FF			0x29
		#define ADXL345_TAP_AXES		0x2a
		#define ADXL345_ACT_TAP_STATUS	0x2b
		#define ADXL345_BW_RATE			0x2c
		#define ADXL345_POWER_CTL		0x2d
		#define ADXL345_INT_ENABLE		0x2e
		#define ADXL345_INT_MAP			0x2f
		#define ADXL345_INT_SOURCE		0x30
		#define ADXL345_DATA_FORMAT		0x31
		#define ADXL345_DATAX0			0x32
		#define ADXL345_DATAX1			0x33
		#define ADXL345_DATAY0			0x34
		#define ADXL345_DATAY1			0x35
		#define ADXL345_DATAZ0			0x36
		#define ADXL345_DATAZ1			0x37
		#define ADXL345_FIFO_CTL		0x38
		#define ADXL345_FIFO_STATUS		0x39
		
		// Bit field definitions and register values

		/* register values for DEVID */
		/* The device ID should always read this value, The customer does not
		need to use this value but it can be read to check that the
		device can communicate */
		#define ADXL345_ID				0xe5

		/* Reserved soft reset value */
		#define ADXL345_SOFT_RESET		0x52

		/* Registers THRESH_TAP through TIME_INACT take only 8-bit values
		There are no specific bit fields in these registers */
		/* Bit values in ACT_INACT_CTL */
		#define ADXL345_INACT_Z_ENABLE		0x01
		#define ADXL345_INACT_Z_DISABLE		0x00
		#define ADXL345_INACT_Y_ENABLE		0x02
		#define ADXL345_INACT_Y_DISABLE		0x00
		#define ADXL345_INACT_X_ENABLE		0x04
		#define ADXL345_INACT_X_DISABLE		0x00
		#define ADXL345_INACT_AC			0x08
		#define ADXL345_INACT_DC			0x00
		#define ADXL345_ACT_Z_ENABLE		0x10
		#define ADXL345_ACT_Z_DISABLE		0x00
		#define ADXL345_ACT_Y_ENABLE		0x20
		#define ADXL345_ACT_Y_DISABLE		0x00
		#define ADXL345_ACT_X_ENABLE		0x40
		#define ADXL345_ACT_X_DISABLE		0x00
		#define ADXL345_ACT_AC				0x80
		#define ADXL345_ACT_DC				0x00

		/* Registers THRESH_FF and TIME_FF take only 8-bit values
		There are no specific bit fields in these registers */
		/* Bit values in TAP_AXES */
		#define ADXL345_TAP_Z_ENABLE		0x01
		#define ADXL345_TAP_Z_DISABLE		0x00
		#define ADXL345_TAP_Y_ENABLE		0x02
		#define ADXL345_TAP_Y_DISABLE		0x00
		#define ADXL345_TAP_X_ENABLE		0x04
		#define ADXL345_TAP_X_DISABLE		0x00
		#define ADXL345_TAP_SUPPRESS		0x08

		/* Bit values in ACT_TAP_STATUS */
		#define ADXL345_TAP_Z_SOURCE		0x01
		#define ADXL345_TAP_Y_SOURCE		0x02
		#define ADXL345_TAP_X_SOURCE		0x04
		#define ADXL345_STAT_ASLEEP			0x08
		#define ADXL345_ACT_Z_SOURCE		0x10
		#define ADXL345_ACT_Y_SOURCE		0x20
		#define ADXL345_ACT_X_SOURCE		0x40

		/* Bit values in BW_RATE */
		/* Expresed as output data rate */
		#define ADXL345_RATE_3200			0x0f
		#define ADXL345_RATE_1600			0x0e
		#define ADXL345_RATE_800			0x0d
		#define ADXL345_RATE_400			0x0c
		#define ADXL345_RATE_200			0x0b
		#define ADXL345_RATE_100			0x0a
		#define ADXL345_RATE_50				0x09
		#define ADXL345_RATE_25				0x08
		#define ADXL345_RATE_12_5			0x07
		#define ADXL345_RATE_6_25			0x06
		#define ADXL345_RATE_3_125			0x05
		#define ADXL345_RATE_1_563			0x04
		#define ADXL345_RATE__782			0x03
		#define ADXL345_RATE__39			0x02
		#define ADXL345_RATE__195			0x01
		#define ADXL345_RATE__098			0x00

		/* Expressed as output bandwidth */
		/* Use either the bandwidth or rate code,
		whichever is more appropriate for your application */
		#define ADXL345_BW_1600			0x0f
		#define ADXL345_BW_800			0x0e
		#define ADXL345_BW_400			0x0d
		#define ADXL345_BW_200			0x0c
		#define ADXL345_BW_100			0x0b
		#define ADXL345_BW_50			0x0a
		#define ADXL345_BW_25			0x09
		#define ADXL345_BW_12_5			0x08
		#define ADXL345_BW_6_25			0x07
		#define ADXL345_BW_3_125		0x06
		#define ADXL345_BW_1_563		0x05
		#define ADXL345_BW__782			0x04
		#define ADXL345_BW__39			0x03
		#define ADXL345_BW__195			0x02
		#define ADXL345_BW__098			0x01
		#define ADXL345_BW__048			0x00
		#define ADXL345_LOW_POWER		0x08
		#define ADXL345_LOW_NOISE		0x00

		/* Bit values in POWER_CTL */
		#define ADXL345_WAKEUP_8HZ				0x00
		#define ADXL345_WAKEUP_4HZ				0x01
		#define ADXL345_WAKEUP_2HZ				0x02
		#define ADXL345_WAKEUP_1HZ				0x03
		#define ADXL345_SLEEP					0x04
		#define ADXL345_MEASURE					0x08
		#define ADXL345_STANDBY					0x00
		#define ADXL345_AUTO_SLEEP				0x10
		#define ADXL345_ACT_INACT_SERIAL		0x20
		#define ADXL345_ACT_INACT_CONCURRENT	0x00

		/* Bit values in INT_ENABLE, INT_MAP, and INT_SOURCE are identical.
		Use these bit values to read or write any of these registers. */
		#define ADXL345_OVERRUN				0x01
		#define ADXL345_WATERMARK			0x02
		#define ADXL345_FREEFALL			0x04
		#define ADXL345_INACTIVITY			0x08
		#define ADXL345_ACTIVITY			0x10
		#define ADXL345_DOUBLETAP			0x20
		#define ADXL345_SINGLETAP			0x40
		#define ADXL345_DATAREADY			0x80
		
		#define ADXL345_WATERMARK_MASK		1
		#define ADXL345_DATAREADY_MASK		7

		// Bit values in DATA_FORMAT
		/* Register values read in DATAX0 through DATAZ1 are dependent on the
		value specified in data format. Customer code will need to interpret
		the data as desired. */
		#define ADXL345_RANGE_2G			0x00
		#define ADXL345_RANGE_4G			0x01
		#define ADXL345_RANGE_8G			0x02
		#define ADXL345_RANGE_16G			0x03
		#define ADXL345_DATA_JUST_RIGHT		0x00
		#define ADXL345_DATA_JUST_LEFT		0x04
		#define ADXL345_10BIT				0x00
		#define ADXL345_FULL_RESOLUTION		0x08
		#define ADXL345_INT_LOW				0x20
		#define ADXL345_INT_HIGH			0x00
		#define ADXL345_SPI3WIRE			0x40
		#define ADXL345_SPI4WIRE			0x00
		#define ADXL345_SELFTEST			0x80

		// Bit values in FIFO_CTL
		/* The low bits are a value 0 to 31 used for the watermark or the number
		of pre-trigger samples when in triggered mode */
		#define ADXL345_TRIGGER_INT1		0x00
		#define ADXL345_TRIGGER_INT2		0x20
		#define ADXL345_FIFO_MODE_BYPASS	0x00
		#define ADXL345_FIFO_RESET			0x00
		#define ADXL345_FIFO_MODE_FIFO		0x40
		#define ADXL345_FIFO_MODE_STREAM	0x80
		#define ADXL345_FIFO_MODE_TRIGGER	0xc0

		// Bit values in FIFO_STATUS
		/* The low bits are a value 0 to 32 showing the number of entries
		currently available in the FIFO buffer */
		#define ADXL345_FIFO_TRIGGERED		0x80


		
		/* ADXL345 Sensitivity (from datasheet) => 3.9mg/LSB   1G => 1000mg/3.9mg = 256 steps
		   Gravity as per datasheet: 256
		   Gravity empirical mode calculation: 235 */
		#define GRAVITY_ADC 				235.0  //this equivalent to 1G in the raw data coming from the accelerometer
		#define GRAVITY_MS2					9.80665
		#define NEG_ONE_G					-9.80665
		
		#define ACCEL_NUM_SAMPLES_BIAS 		400
		
		#define ADXL345_PIN_INT1			(1 << 0)
		#define ADXL345_PIN_INT2			(1 << 1)
		#define ADXL345_PORT_INT			PORTH
	
/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
	extern "C" {
#endif
			
		int accel_initialize(void);
			
		uint8_t accel_spi_read(void);
		void accel_spi_write(uint8_t value);
		void accel_read_delay(void);
		uint8_t accel_read_register(uint8_t register_name);
		void accel_write_register(uint8_t register_name, uint8_t register_value);
		bool accel_write_and_verify(uint8_t register_name, uint8_t value);
		int accel_read_register_multibyte(uint8_t register_name, uint8_t *pData, uint8_t numbytes);
		int accel_write_register_multibyte(uint8_t register_name, uint8_t *pData, uint8_t numbytes);
		void accel_update_offset_data(void);
		
		int accel_read_vector(void);
		void accel_reset_sum(void);
		void write_accel_reset_sum_function_to_fpu_eeprom(void);
		void accel_read_vector_and_sum(void);
		void accel_evaluate_ms2(void);
		void accel_compute_bias(void);
		void write_accel_compute_bias_function_to_fpu_eeprom(void);
		
/* Disable C linkage for C++ Compilers: */
#if defined(__cplusplus)
	}
#endif
		
#endif

/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "hmc5883L.h"

/* ================================================================================================
   Externs
   ================================================================================================ */
#ifdef HMC5883L
	extern TWI_Master_t 			twi_master_mag;
#endif

/* ================================================================================================
   Defines
   ================================================================================================ */

	#define HMC5883L_FORMAT 				HMC5883L_DATA_OUT_10HZ | HMC5883L_DATA_OUT_NORMAL
	#define HMC5883L_SELF_TEST_FORMAT 	HMC5883L_DATA_OUT_POS_BIAS | HMC5883L_DATA_OUT_10HZ
	
/* ================================================================================================
   Globals
   ================================================================================================ */
#ifdef HMC5883L
	const unsigned char hmc5883L_init_data[8] = {
			HMC5883L_MODE, HMC5883L_MODE_IDLE,
			HMC5883L_CONFIG_A, HMC5883L_FORMAT,				// 0x00: 10hz (default), normal (these are the defaults)
			HMC5883L_CONFIG_B, HMC5883L_GAIN_1_GA,			// 0x01: +/- 1.0ga gain
			HMC5883L_MODE, HMC5883L_MODE_SINGLE				// 0x02: start single measurement
	};

	const unsigned char hmc5883L_self_test_data[8] = {
			HMC5883L_MODE, HMC5883L_MODE_IDLE,
			HMC5883L_CONFIG_A, HMC5883L_SELF_TEST_FORMAT,		// 0x00: 10hz (default), normal
			HMC5883L_CONFIG_B, HMC5883L_GAIN_1_GA,			// 0x01: +/- 1.0ga gain
			HMC5883L_MODE, HMC5883L_MODE_SINGLE				// 0x02: single measurement
	};
#endif

/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize magnetometer
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_initialize(void)
{
	unsigned char reg;
	uint8_t devIDA = 0;
	uint8_t devIDB = 0;
	uint8_t devIDC = 0;
	int res = ERR_SUCCESS;
	
	res = mag_selftest();
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		return ERR_HMC5883L_INIT;
	}
	
	res = mag_twi_read(HMC5883L_ID_A, &devIDA);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		report_error(ERR_HMC5883L_READ);
		return ERR_HMC5883L_INIT;
	}
	
	res = mag_twi_read(HMC5883L_ID_B, &devIDB);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		report_error(ERR_HMC5883L_READ);
		return ERR_HMC5883L_INIT;
	}
	
	res = mag_twi_read(HMC5883L_ID_C, &devIDC);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		report_error(ERR_HMC5883L_READ);
		return ERR_HMC5883L_INIT;
	}
	
	if( devIDA != 'H' || devIDB != '4' || devIDC != '3' )
	{
		return ERR_HMC5883L_INVALID_ID;
	}
	
	for(int i = 0; i < 8; i += 2)
	{
		reg = hmc5883L_init_data[i];								// get register
		res = mag_twi_write(reg, hmc5883L_init_data[i + 1]);	// write data
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
			report_error(ERR_HMC5883L_WRITE);
			return ERR_HMC5883L_INIT;
		}
	}
	
	// Critical -- wait for the first measurement to be taken and placed in x, y and z registers
	_delay_ms(HMC5883L_10HZ_WAIT);
	
	// The first reading is performed with roll (kinematics_angle_x) and pitch (kinematics_angle_y) both 
	// equal to zero (i.e. quadcopter is flat). The kinematics angles are zero from the call to initialize
	// kinematics earlier in quadcopter initialization.
	mag_measure();
#ifdef MEGANEURA_DEBUG
#ifdef HMC5883L
	usart_tx_string("The first mag reading raw is: ");
	pc_send_mag_raw_data();
	usart_tx_string("The first mag reading measured is: ");
	pc_send_mag_measured_data();
	usart_tx_string(LINE_END);
#endif
#endif

	return res;
}
#endif

/* ----------------------------------------------------------------------------
   Read a measurement from magnetometer
     - the y and z axes are flipped to conform to the right hand coordinate system
	 - measurement is copied to FPU
	 - FPU function MagMeasure is called
	 - we start a new single measurement after we are done
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_measure(void)
{
	int res = ERR_SUCCESS;
	int x, y, z;
 
	/* NOTE: We need 6 bytes; 2 bytes for x, y and z. Why the extra byte? This resets
     * the internal pointer back to the x register. Also x, y, z comes in the order of 
     * MSB followed by LSB. */
    TWI_MasterRead(&twi_master_mag,
		                    HMC5883L_SLAVE_ADDRESS,
                        7);
		
	while (twi_master_mag.status != TWIM_STATUS_READY);
	
	if(twi_master_mag.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_mag.result);
	}
	
	if(twi_master_mag.bytesRead != 7)
	{
		return ERR_HMC5883L_READ;
	}
	
	// Flip the y and z axes so that y+ points to the right and z+ points down to the ground (x points forward)
	x = ((twi_master_mag.readData[0] << 8) | twi_master_mag.readData[1]);
	y = -((twi_master_mag.readData[2] << 8) | twi_master_mag.readData[3]);
	z = -((twi_master_mag.readData[4] << 8) | twi_master_mag.readData[5]);

	fpu_write_int_to_reg_nn(x, FPU_REG_MAG_X);
	fpu_write_int_to_reg_nn(y, FPU_REG_MAG_Y);
	fpu_write_int_to_reg_nn(z, FPU_REG_MAG_Z);
	
	//fpu_wait();
	fpu_write2(FCALL, FPU_FUNC_MAG_MEASURE);
	//fpu_wait();
	
	// Start single measurement
	res = mag_twi_write(HMC5883L_MODE, HMC5883L_MODE_SINGLE);	
		
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		report_error(ERR_HMC5883L_WRITE);
		//return ERR_HMC5883L_WRITE;
	}

	return res;
}
#endif

/* ----------------------------------------------------------------------------
   Perform a self-test
     - self-test is a built in feature of HMC5883L to calculate the scale factor
	 - self-test takes 2 measurements, calculates the difference and places them
	   in the x, y and z registers for us to read
	 - first measurement is with no artificial magnetic field applied
	 - second measurement applies a 0.55ga magnetic field
	 - data in x, y and z registers is (second measurement - first measurement)
	 - the difference is only that of the artificial field (i.e. it removes natural
	   magnetic field created by earth and possibly speaker magnets for example)
	 - 0.55ga is equal to 715 ADC units with +/-1ga range selected
	 - so we scale each value to be 715 since that is what it should be with the
	   artificial field applied
	 - the scale factors are copied to FPU registers for x, y and z
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_selftest(void)
{
	unsigned char reg;
	int res = ERR_SUCCESS;
	
	// Put hmc5883L into self test mode
	for(int i = 0; i < 8; i += 2)
	{
		reg = hmc5883L_self_test_data[i];							// get register
		res = mag_twi_write(reg, hmc5883L_self_test_data[i + 1]);	// write data
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
			report_error(ERR_HMC5883L_WRITE);
			return ERR_HMC5883L_SELF_TEST;
		}
	}
	
	_delay_ms(HMC5883L_10HZ_WAIT);
	
	// read scale vector (target is 0.55 ga or 715 adc units) (range is 0 - 1300 +-1.0ga)
	TWI_MasterRead(&twi_master_mag,
		                    HMC5883L_SLAVE_ADDRESS,
                        6);
		
	while (twi_master_mag.status != TWIM_STATUS_READY);
	
	if(twi_master_mag.result != TWIM_RESULT_OK)
	{
		report_error(process_twi_error(&twi_master_mag.result));
		return ERR_HMC5883L_SELF_TEST;
	}
	
	if(twi_master_mag.bytesRead != 6)
	{
		report_error(ERR_HMC5883L_READ);
		return ERR_HMC5883L_SELF_TEST;
	}
	
	int x, y, z;

	x = ((twi_master_mag.readData[0] << 8) | twi_master_mag.readData[1]);
	y = ((twi_master_mag.readData[2] << 8) | twi_master_mag.readData[3]);
	z = ((twi_master_mag.readData[4] << 8) | twi_master_mag.readData[5]);
	
	double scale_x, scale_y, scale_z;
	scale_x = HMC5883L_SELFTEST_TARGET_1_0_GA / (double)x;
	scale_y = HMC5883L_SELFTEST_TARGET_1_0_GA / (double)y;
	scale_z = HMC5883L_SELFTEST_TARGET_1_0_GA / (double)z;
	
	fpu_eeprom_write_double_to_slot(scale_x, FPU_EEPROM_SLOT_MAG_SCALE_X);
	fpu_eeprom_write_double_to_slot(scale_y, FPU_EEPROM_SLOT_MAG_SCALE_Y);
	fpu_eeprom_write_double_to_slot(scale_z, FPU_EEPROM_SLOT_MAG_SCALE_Z);

	return ERR_SUCCESS;
}
#endif

/* ----------------------------------------------------------------------------
   Read a measurement from magnetometer
     - this function is not called right now since we are using single measurement mode
	 - use this function if continuous measurements are used
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_read_vector(void)
{
	/* NOTE: It is the responsibility of the user to wait appropriate amount of time between readings.
	 * 
	 * 10 Hz: 100ms
	 * 20 Hz: 50ms
	 * 50 Hz: 20ms */
	 
	int res = ERR_SUCCESS;
	int x, y, z;
 
	/* NOTE: We need 6 bytes; 2 bytes for x, y and z. Why the extra byte? This resets
     * the internal pointer back to the x register. Also x, y, z comes in the order of 
     * MSB followed by LSB. */
    TWI_MasterRead(&twi_master_mag,
		                    HMC5883L_SLAVE_ADDRESS,
                        7);
		
	while (twi_master_mag.status != TWIM_STATUS_READY);
	
	if(twi_master_mag.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_mag.result);
	}
	
	if(twi_master_mag.bytesRead != 7)
	{
		return ERR_HMC5883L_READ;
	}
	
	// Flip the y and z axes so that y+ points to the right and z+ points down to the ground (x points forward)
	x = ((twi_master_mag.readData[0] << 8) | twi_master_mag.readData[1]);
	y = -((twi_master_mag.readData[2] << 8) | twi_master_mag.readData[3]);
	z = -((twi_master_mag.readData[4] << 8) | twi_master_mag.readData[5]);

	/* x, y, z */
	fpu_write_int_to_reg_nn(x, FPU_REG_MAG_X);
	fpu_write_int_to_reg_nn(y, FPU_REG_MAG_Y);
	fpu_write_int_to_reg_nn(z, FPU_REG_MAG_Z);

	return res;
}
#endif

/* ----------------------------------------------------------------------------
   Read a register using TWI (I2C) communication
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_twi_read(uint8_t reg, uint8_t *pValue)
{
	uint8_t buf = reg;
	int res = ERR_SUCCESS;
	
	if( NULL == pValue )
	{
		return ERR_NULL_POINTER;
	}

	*pValue = 0;
	
	TWI_MasterWriteRead(&twi_master_mag,
						HMC5883L_SLAVE_ADDRESS,
						&buf,
						1,1);
	
	while (twi_master_mag.status != TWIM_STATUS_READY);
	
	if(twi_master_mag.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_mag.result);
	}
	
	if(twi_master_mag.bytesRead != 1)
	{
		return ERR_HMC5883L_READ;
	}
	
	*pValue = (int)twi_master_mag.readData[0];

	return res;
}
#endif

/* ----------------------------------------------------------------------------
   Write a register using TWI (I2C) communication
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
int mag_twi_write(uint8_t reg, uint8_t value)
{
	uint8_t buf[2];
	int res = ERR_SUCCESS;
	
	buf[0] = reg;
	buf[1] = value;
	
	TWI_MasterWrite(&twi_master_mag,
						HMC5883L_SLAVE_ADDRESS,
						buf,
						2);
	
	while (twi_master_mag.status != TWIM_STATUS_READY);
	
	if(twi_master_mag.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_mag.result);
	}
	
	if(twi_master_mag.bytesWritten != 2)
	{
		return ERR_HMC5883L_WRITE;
	}

	return res;
}
#endif


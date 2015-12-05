/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "itg3200.h"

/* ================================================================================================
   Externs
   ================================================================================================ */
   
	extern TWI_Master_t 			twi_master_gyro;

/* ================================================================================================
   Globals
   ================================================================================================ */

	// Gyroscope internal oscillator runs at 1000hz (8000hz possible) but outputs the data at 200hz
	// because of the sample rate divider. The main application takes measurements at 400hz.
	const uint8_t itg3200_init_data[] = {
			PWR_MGM, 0x80,								// reset device and registers to default
			SAMPLE_RATE_DIVIDER, ITG3200_RATE(200),		// Default is 0x00 default is no prescaler
			DLPF, 0x1D,									// fs_sel = 3, +- 2000º/sec DLPF_CFG = 0, 1000Hz internal sample rate, 10 hz low pass filter
			INT_CFG, 0x00,								// (default) don't generate interrupt when data ready
			PWR_MGM, 0x01,								// clk_sel = internal oscillator, no reset, no sleep, gyro xyz in standby mode
			0x00										// end of configuration
	};

	// Used to evaluate rate (velocity of rotation)
	uint32_t gyro_last_measured_time = 0;

/* ================================================================================================
   Initialization functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize gyroscope
   ---------------------------------------------------------------------------- */
int gyro_initialize(void)
{
	uint8_t* data = (uint8_t*)itg3200_init_data;
	uint8_t reg;
	uint8_t devID = 0;
	int res = ERR_SUCCESS;
	
	res = gyro_twi_read(WHO_AM_I, &devID);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
		return ERR_ITG3200_INIT;
	}
	
	if( devID != ITG3200_SLAVE_ADDRESS )
	{
		report_error(ERR_ITG3200_INVALID_ID);
		return ERR_ITG3200_INVALID_ID;
	}
	
	while (*data)
	{
		reg = *data++;
		res = gyro_twi_write(reg, *data++);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
			return ERR_ITG3200_INIT;
		}
	}
	
	return res;
}

/* ================================================================================================
   Measurement functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read measurement and copy to x, y, z
   ---------------------------------------------------------------------------- */
int gyro_read_vector_to_local_ints(int *pX, int *pY, int *pZ)
{
	int res = ERR_SUCCESS;
	uint8_t high, low;
	
	// Read X
	if(pX != NULL)
	{
		res = gyro_twi_read(GX_H, &high);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		res = gyro_twi_read(GX_L, &low);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		*pX = ((int)(high << 8) | (int)low);
	}
	
	// Read Y
	if(pY != NULL)
	{
		res = gyro_twi_read(GY_H, &high);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		res = gyro_twi_read(GY_L, &low);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		*pY = ((int)(high << 8) | (int)low);
	}
	
	// Read Z
	if(pZ != NULL)
	{
		res = gyro_twi_read(GZ_H, &high);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		res = gyro_twi_read(GZ_L, &low);
		
		if(ERR_SUCCESS != res)
		{
			report_error(res);
		}
		
		*pZ = ((int)(high << 8) | (int)low);
	}
	
	return res;
}

/* ----------------------------------------------------------------------------
   Read measurement and copy to FPU
   ---------------------------------------------------------------------------- */
int gyro_read_vector(void)
{
	int res = ERR_SUCCESS;
	int x = 0, y = 0, z = 0;
	
	res = gyro_read_vector_to_local_ints(&x, &y, &z);
	
	fpu_write_int_to_reg_nn(x, FPU_REG_GYRO_X);
	fpu_write_int_to_reg_nn(y, FPU_REG_GYRO_Y);
	fpu_write_int_to_reg_nn(z, FPU_REG_GYRO_Z);
	
	return res;
}

/* ----------------------------------------------------------------------------
   Read measurement, add to sum registers and increase sample count
     - this function is called by main application
   ---------------------------------------------------------------------------- */
void gyro_read_vector_and_sum(void)
{
	// Read gyro vector
	gyro_read_vector();
	
	// 1. Add to sum registers
	// 2. Increment the sample count
	fpu_function_call(FPU_FUNC_GYRO_SUM);
}

/* ----------------------------------------------------------------------------
     Evaluate raw gyro data into rate of radians/second
	   1. Calculate the average of the samples collected
	   2. Apply the scale factor
	   3. Apply offsets so that the raw data centers on zero
	   4. Reset the sum registers and sample count
	   
	   Gyro ADC calculation is:
	   adc_x = (avgOfSamples - offset_x)
	   adc_y = -1(avgOfSamples - offset_y) = (offset_y - avgSamples)
	   adc_z = -1(avgOfSamples - offset_z) = (offset_z - avgSamples)
	   
	   Orientation:
	   x+ axis points to the front
	   y+ axis points to the left
	   z+ axis points up
	   
	   Clockwise rotation on any axis yields a positive angular velocity.
	   Consequently counterclockwise rotation on any axis yields a negative
	   angular velocity.
   ---------------------------------------------------------------------------- */
void gyro_evaluate_rate(void)
{
	uint32_t currentTime = micros();
	double time_elapsed = (currentTime - gyro_last_measured_time) / 1000000.0;
	
	fpu_write_double_to_reg_nn(time_elapsed, FPU_REG_TEMPORARY_3);

	fpu_function_call(FPU_FUNC_GYRO_EVALUATE_RATE);
	
	gyro_last_measured_time = currentTime;
}

/* ----------------------------------------------------------------------------
   Zero sum registers (x, y, z) and sample count
   ---------------------------------------------------------------------------- */
void gyro_reset_sum(void)
{
	fpu_eeprom_function_call(FPU_EEPROM_FUNC_GYRO_RESET_SUM);
}

/* ----------------------------------------------------------------------------
   FPU function to zero the sum registers (x, y, z) and sample count
     - generated by FPU IDE
	 - readable function implemented in .fpu file
   ---------------------------------------------------------------------------- */
void write_gyro_reset_sum_function_to_fpu_eeprom(void)
{
	fpu_wait();
    fpu_write4(EEWRITE, FPU_EEPROM_FUNC_GYRO_RESET_SUM, 0x0A, 0x09);
    fpu_write4(0x03, 0x52, 0x03, 0x53);
    fpu_write4(0x03, 0x54, 0x03, 0x55);
    fpu_write(0x80);
    fpu_wait();
}

/* ================================================================================================
   Calibration functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Find the median and difference between max and min
   ---------------------------------------------------------------------------- */
int findMedianIntWithDiff(int *data, int arraySize, int * diff) 
{
  int temp;
  bool done = 0;
  byte i;
  
   // Sorts numbers from lowest to highest
  while (done != 1) 
  {        
    done = 1;
    for (i = 0; i < (arraySize - 1); i++) 
	{
      if (data[i] > data[i+1]) 
	  {     
		// numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }

  *diff = abs(data[0] - data[arraySize-1]);
  
  return data[arraySize / 2]; // return the median value
}

/* ----------------------------------------------------------------------------
   Calibrate gyroscope
     - takes 200 samples and throws them away
	 - takes 49 samples per axis, find the median and calculate the difference
	   between max and min
	 - the median is the offset value
	 - the difference is used to make sure the board is stable during this process
   ---------------------------------------------------------------------------- */
bool gyro_calibrate(void)
{
	int findZero[GYRO_NUM_SAMPLES_BIAS] = {0};
	int diff = 0;
	
	// ************************************************************************
	// Throw away the first 200 samples
	// ************************************************************************
	for(int num_samples = 0; num_samples < 200; num_samples++)
	{
		gyro_read_vector();
		_delay_us(2500);
	}
	
	// ************************************************************************
	// X AXIS - Read 49 samples at 400hz and then calculate the offset
	// ************************************************************************
	for(int num_samples = 0; num_samples < GYRO_NUM_SAMPLES_BIAS; num_samples++)
	{
		int x = 0;
		
		gyro_read_vector_to_local_ints(&x, NULL, NULL);
		findZero[num_samples] = x;
		_delay_us(2500);
	}
	
	int tmp = findMedianIntWithDiff(findZero, GYRO_NUM_SAMPLES_BIAS, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) 
	{
		// 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
		//fpu_wait();
		fpu_eeprom_write_double_to_slot((double)tmp, FPU_EEPROM_SLOT_GYRO_OFFSET_X);
	} 
	else 
	{
		return false; //Calibration failed due to movement.
	}
	
	// ************************************************************************
	// Y AXIS - Read 49 samples at 400hz and then calculate the offset
	// ************************************************************************
	for(int num_samples = 0; num_samples < GYRO_NUM_SAMPLES_BIAS; num_samples++)
	{
		int y = 0;
		
		gyro_read_vector_to_local_ints(NULL, &y, NULL);
		findZero[num_samples] = y;
		_delay_us(2500);
	}
	
	tmp = findMedianIntWithDiff(findZero, GYRO_NUM_SAMPLES_BIAS, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) 
	{ 
		// 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
		//fpu_wait();
		fpu_eeprom_write_double_to_slot((double)tmp, FPU_EEPROM_SLOT_GYRO_OFFSET_Y);
	} 
	else 
	{
		return false; //Calibration failed.
	}
	
	// ************************************************************************
	// Z AXIS - Read 49 samples at 400hz and then calculate the offset
	// ************************************************************************
	for(int num_samples = 0; num_samples < GYRO_NUM_SAMPLES_BIAS; num_samples++)
	{
		int z = 0;
		
		gyro_read_vector_to_local_ints(NULL, NULL, &z);
		findZero[num_samples] = z;
		_delay_us(2500);
	}
	
	tmp = findMedianIntWithDiff(findZero, GYRO_NUM_SAMPLES_BIAS, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) 
	{ 
		// 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
		//fpu_wait();
		fpu_eeprom_write_double_to_slot((double)tmp, FPU_EEPROM_SLOT_GYRO_OFFSET_Z);
	}
	else 
	{
		return false; //Calibration failed due to movement
	}
	
#ifdef MEGANEURA_DEBUG
	pc_send_string_resource(STRING_GYRO_BIAS);
	pc_send_gyro_calibration_data();
#endif
	
	return true;
}

/* ================================================================================================
   Gyroscope temperature functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read and calculate the temperature of the gyroscope
     - temperature affects readings so this can be used to improve the quality
	   of data
   ---------------------------------------------------------------------------- */
#if 0
void gyro_read_temperature(void)
{
	int res = ERR_SUCCESS;
	uint8_t high, low;
	int temperature = 0;
	
	// Read Temperature
	res = gyro_twi_read(TMP_H, &high);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
	}
	
	res = gyro_twi_read(TMP_L, &low);
	
	if(ERR_SUCCESS != res)
	{
		report_error(res);
	}
	
	temperature = (int)(high << 8) | (int)low;
	
	fpu_write_int_to_reg_nn(temperature, FPU_REG_GYRO_TEMPERATURE);
	
	//gyro_temperature = 35.0 + ( ( gyro_temperature_raw + 13200 ) / 280.0 );
	fpu_function_call(FPU_FUNC_CALC_GYRO_TEMP);
}
#endif

/* ================================================================================================
   Gyroscope filter data functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Smooth the data
   ---------------------------------------------------------------------------- */
#if 0
double smooth(double current_data, double previous_data, double smooth_factor)
{
	return ( ( previous_data * (1.0 - smooth_factor) ) + (current_data * smooth_factor) );
}
#endif

/* ================================================================================================
   TWI (I2C) communication functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read data with TWI (I2C)
   ---------------------------------------------------------------------------- */
int gyro_twi_read(uint8_t reg, uint8_t *pValue)
{
	uint8_t buf = reg;
	int res = ERR_SUCCESS;
	
	if( NULL == pValue )
	{
		return ERR_NULL_POINTER;
	}
	
	*pValue = 0;
		
	TWI_MasterWriteRead(&twi_master_gyro,
						ITG3200_SLAVE_ADDRESS,
						&buf,
						1,1);
	
	while (twi_master_gyro.status != TWIM_STATUS_READY);
	
	if(twi_master_gyro.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_gyro.result);
	}
	
	if(twi_master_gyro.bytesRead != 1)
	{
		return ERR_ITG3200_READ;
	}

	*pValue = twi_master_gyro.readData[0];

	return res;
}

/* ----------------------------------------------------------------------------
   Write data with TWI (I2C)
   ---------------------------------------------------------------------------- */
int gyro_twi_write(uint8_t reg, uint8_t value)
{
	uint8_t buf[2];
	int res = ERR_SUCCESS;
	
	buf[0] = reg;
	buf[1] = value;
	
	TWI_MasterWrite(&twi_master_gyro,
						ITG3200_SLAVE_ADDRESS,
						buf,
						2);
	
	while (twi_master_gyro.status != TWIM_STATUS_READY);
	
	if(twi_master_gyro.result != TWIM_RESULT_OK)
	{
		return process_twi_error(&twi_master_gyro.result);
	}
	
	if(twi_master_gyro.bytesWritten != 2)
	{
		return ERR_ITG3200_WRITE;
	}

	return res;
}

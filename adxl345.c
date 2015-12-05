/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "adxl345.h"
	
/* ================================================================================================
   Externs
   ================================================================================================ */
   
	extern SPI_Master_t spiMasterD;
	extern uint8_t accel_flags;
	
/* ================================================================================================
   Defines
   ================================================================================================ */

	// 11 bit resolution, 4g range (0 to +/-1024), spi 4 wire, interrupt active low, justify right 
	// (msb right, signed short, 10 bits value and 1 bit for sign.
	// 1g is 256 (typical), test to see what the actual 1g ADC value is.
	#define ADXL345_FORMAT	ADXL345_FULL_RESOLUTION | ADXL345_DATA_JUST_RIGHT | ADXL345_RANGE_4G | ADXL345_SPI4WIRE //| ADXL345_INT_LOW
	#define TAP_DUR			0x10					// 625 us/LSB, 0x10=10ms
	#define THRESH_TAP		0x08					// 62.5mg/LSB, 0x08=0.5g
	#define TAP_AXES		ADXL345_TAP_Z_ENABLE	// 0x2a: TAP_Z enable

/* ================================================================================================
   Globals
   ================================================================================================ */

	// Settings, put on standby first
	const unsigned char adxl345_init_data[] = {
			ADXL345_POWER_CTL, ADXL345_STANDBY,		// 0x2d: standby mode before changing registers below
			ADXL345_BW_RATE, ADXL345_RATE_200,		// 0x2c 200hz sampling
			ADXL345_INT_MAP, 0x00,					// 0x2f: all interrupts to INT1
			ADXL345_DATA_FORMAT, ADXL345_FORMAT,	// 0x31
			ADXL345_POWER_CTL, ADXL345_MEASURE,		// 0x2d: measure
			0x00									// end of configuration
	};

	// Instantiate pointers to SPI, SPI port and SPI master
	PORT_t *ssPortD = &PORTD;
	SPI_t *ssSPID = &SPID;
	SPI_Master_t *ssSPIMasterD = &spiMasterD;

/* ================================================================================================
   Functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Initialize accelerometer
	 - initialize SPI
	 - write default settings
	 - take the first measurement and throw away
   ---------------------------------------------------------------------------- */
int accel_initialize(void)
{
	uint8_t temp;

	unsigned char* data = (unsigned char*)adxl345_init_data;
	unsigned char reg;
	
	/* Init SS pin as output with wired AND and pull-up. */
	ssPortD->DIRSET = SPI_SS_bm;
	ssPortD->PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	ssPortD->OUTSET = SPI_SS_bm;

	/* Initialize SPI master on port C. MOSI and SCK as output */
	/* The maximum SPI clock speed on ADXL345 is 5MHZ */
	SPI_MasterInit(ssSPIMasterD,
	               ssSPID,
	               ssPortD,
	               false,
	               SPI_MODE_3_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV16_gc); // 32 / 16 = 2 MHZ 
	
	accel_read_delay(); // read delay or the data will be erroneous
	
	temp = accel_read_register(ADXL345_DEVID);
	
	if(temp != ADXL345_ID)
	{
		return ERR_ADXL345_INIT;
	}

	// Write settings to registers and start measurements
	while (*data)
	{
		reg = *data++;
			
		if( !accel_write_and_verify(reg, *data++) )
		{
			report_error(ERR_ADXL345_WRITE);
			return ERR_ADXL345_INIT;
		}
	}
	
	accel_read_vector();
	accel_reset_sum();
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Read register
   ---------------------------------------------------------------------------- */
uint8_t accel_read_register(uint8_t register_name)
{
	uint8_t temp = 0;
	
	temp = register_name | 0x80;
	
	SPI_MasterSSLow(ssPortD, SPI_SS_bm);
	
	temp = SPI_MasterTransceiveByte(ssSPIMasterD, temp);
	temp = SPI_MasterTransceiveByte(ssSPIMasterD, 0x00);
	
	SPI_MasterSSHigh(ssPortD, SPI_SS_bm);
	
	return temp;
}

/* ----------------------------------------------------------------------------
   Write register
   ---------------------------------------------------------------------------- */
void accel_write_register(uint8_t register_name, uint8_t register_value)
{
	uint8_t temp = 0;
	
	SPI_MasterSSLow(ssPortD, SPI_SS_bm);
	temp = SPI_MasterTransceiveByte(ssSPIMasterD, register_name);
	temp = SPI_MasterTransceiveByte(ssSPIMasterD, register_value);
	SPI_MasterSSHigh(ssPortD, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   Write register and verify
   ---------------------------------------------------------------------------- */
bool accel_write_and_verify(uint8_t register_name, uint8_t value)
{
	uint8_t temp;
	
	accel_write_register(register_name, value);
	
	if( value != (temp = accel_read_register(register_name)) )
	{
		return false;
	}

	return true;
}

/* ----------------------------------------------------------------------------
   Read multiple bytes starting at register address
   ---------------------------------------------------------------------------- */
int accel_read_register_multibyte(uint8_t register_name, uint8_t *pData, uint8_t numbytes)
{
	uint8_t temp = 0;
	uint8_t i = 0;
	
	temp = register_name | 0x80 | 0x40 ;
	
	SPI_MasterSSLow(ssPortD, SPI_SS_bm);

	_delay_us(1);
	
	if( pData == NULL)
	{
		report_error(ERR_NULL_POINTER);
		return ERR_ADXL345_READ;
	}
	
	if( numbytes < 2 )
	{
		return ERR_ADXL345_READ;
	}
	
	/* write register address and then read first byte */
	SPI_MasterTransceiveByte(ssSPIMasterD, temp);
	*pData++ = SPI_MasterTransceiveByte(ssSPIMasterD, 0x00);
	i++;
	
	while( pData != NULL && i < numbytes )
	{
		*pData++ = SPI_MasterTransceiveByte(ssSPIMasterD, 0x00); // send dummy byte
		i++;
	}
	
	SPI_MasterSSHigh(ssPortD, SPI_SS_bm);
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Read delay
   ---------------------------------------------------------------------------- */
void accel_read_delay(void)
{
	_delay_us(500);
}

/* ----------------------------------------------------------------------------
   Write multiple bytes starting at register address
   ---------------------------------------------------------------------------- */
int accel_write_register_multibyte(uint8_t register_name, uint8_t *pData, uint8_t numbytes)
{
	uint8_t temp = 0;
	uint8_t i = 0;
	
	temp = register_name | 0x40 ;
	
	SPI_MasterSSLow(ssPortD, SPI_SS_bm);
	
	if( pData == NULL)
	{
		report_error(ERR_NULL_POINTER);
		return ERR_ADXL345_WRITE;
	}
	
	if( numbytes < 2 )
	{
		return ERR_ADXL345_WRITE; // it's not a multibyte request
	}
	
	while( pData != NULL && i < numbytes )
	{
		SPI_MasterTransceiveByte(ssSPIMasterD, *pData++);
		i++;
	}
	
	SPI_MasterSSHigh(ssPortD, SPI_SS_bm);
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Read vector (x, y, z)
     - flip the y and z axes to conform to the right hand coordinate system
	 - store vector in FPU
   ---------------------------------------------------------------------------- */
int accel_read_vector(void)
{
	uint8_t data[6];
	int x = 0, y = 0, z = 0;
	
	accel_read_register_multibyte(0x32, data, 6);

	// Flip the y and z axes
	// +x points to the front
	// +y points to the right
	// +z points to the ground
	x = ((int16_t)(data[1] << 8) | data[0]);
	y = -((int16_t)(data[3] << 8) | data[2]);
	z = -((int16_t)(data[5] << 8) | data[4]);

	while( !(PORT_GetPortValue(ssPortD) & SPI_SS_bm) );
	
	fpu_write_int_to_reg_nn(x, FPU_REG_ACCEL_X);
	fpu_write_int_to_reg_nn(y, FPU_REG_ACCEL_Y);
	fpu_write_int_to_reg_nn(z, FPU_REG_ACCEL_Z);
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Reset sum
     - zeroes the sum registers in FPU (i.e. sum x, y, z and number of samples)
   ---------------------------------------------------------------------------- */
void accel_reset_sum(void)
{
	fpu_eeprom_function_call(FPU_EEPROM_FUNC_ACCEL_RESET_SUM);
}

/* ----------------------------------------------------------------------------
   FPU function to zero the sum registers (x, y, z) and sample count
     - generated by FPU IDE
	 - readable function implemented in .fpu file
   ---------------------------------------------------------------------------- */
void write_accel_reset_sum_function_to_fpu_eeprom(void)
{
	fpu_wait();
    fpu_write4(EEWRITE, FPU_EEPROM_FUNC_ACCEL_RESET_SUM, 0x0A, 0x09);
    fpu_write4(0x03, 0x28, 0x03, 0x29);
    fpu_write4(0x03, 0x2A, 0x03, 0x2B);
    fpu_write(0x80);
    fpu_wait();
}

/* ----------------------------------------------------------------------------
   Read vector (x, y, z) and sum
     - add the vector to the sum registers for x, y and z
	 - increment the number of samples by 1
   ---------------------------------------------------------------------------- */
void accel_read_vector_and_sum(void)
{
	accel_read_vector();
	
	fpu_function_call(FPU_FUNC_ACCEL_SUM);
}

/* ----------------------------------------------------------------------------
   Evaluate MS2
     - MS2 = meters per second squared
	 - this functions divides the sum registers for x, y and z by the sample
	   count
	 - it converts the ADC values to MS2 by multiplying by the scale factors
	 - scale factors are 1g factors so that 255 ADC units equals 1g
	 - scale factors where calculated by hand using 50+ samples for each axis and
	   each axis direction (that is 6 tests, up, down, left, right, nose up/down)
	 - it applies the offset
	 - finally it zeroes the sum registers and sample count
   ---------------------------------------------------------------------------- */
void accel_evaluate_ms2(void)
{
	fpu_function_call(FPU_FUNC_ACCEL_EVELUATE_MS2);
}

/*
	Gravity vs. Acceleration
	
	It's very important to understand the acceleration that Accelerometers measure.
	
	Visualize you are floating weightless in a spaceship in deep space. An accelerometer will show no 
	acceleration (0 g). You strap into your seat, and the rockets fire, accelerating the spaceship 
	(and you) at 9.8 m/s^2 (1 g). The accelerometer will show 1 g of acceleration. The acceleration 
	creates the physical impression of weight / gravity. Accelerometers cannot distinguish between 
	the feeling of weight that you feel when you are accelerated (your speed changes), and the steady 
	pull of gravity.
	
	Stated in a different way - Accelerometers do not measure acceleration due to changes in velocity. 
	Rather, they experience Proper Acceleration - the phenomenon of weight experienced by a test mass 
	that resides in the accelerometer device. On earth, if the accelerometer is sitting on a surface, 
	completely immobile, it will register a non-zero acceleration (gravity). If the device is in free 
	fall it will measure no acceleration.

	An accelerometer at rest relative to the Earth's surface will indicate approximately 1 g upwards 
	(Because in order to remain at rest, it is experiencing a force opposite to the earth's gravity- this
    is called the normal force). To obtain the acceleration due to motion with respect to the Earth, this 
	"gravity offset" must be subtracted. The effects of this acceleration are indistinguishable from any 
	other acceleration experienced by the instrument, so an accelerometer cannot detect the difference between 
	sitting in a rocket on the launch pad, and being in the same rocket in deep space while it uses its engines 
	to accelerate at 1 g. For similar reasons, an accelerometer will read zero during any type of free fall in 
	a vacuum. When free falling through air, the air resistance produces drag forces that reduce the acceleration, 
	until constant terminal velocity is reached. At terminal velocity the accelerometer will indicate 1 g 
	acceleration upwards. For the same reason a skydiver, upon reaching terminal velocity, does not feel as 
	though he or she were in "free-fall", but rather experiences a feeling similar to being supported (at 1 g) 
	on a "bed" of uprushing air.
	
	JAT: Above explains why we subtract 1g from the z axis when computing the offset. x and y are offsets to 0
	     and z is offset to negative 1g (-9.80665)

*/

/* ----------------------------------------------------------------------------
   Compute the bias (offset)
     - this is performed automatically on first time boot and the values are
	   written to EEPROM
	 - throws away the first reading
	 - reads 400 samples at 400hz
	 - averages the samples and computes the runtime bias
	 - the scale factors are applied before calculating the bias
	 - the offset (bias) centered on (0, 0, -9.80665)
   ---------------------------------------------------------------------------- */
void accel_compute_bias(void)
{
	fpu_wait();
	
	// STEP1: Get the first reading and throw away. I found it to be 0.0
	accel_read_vector();
	
	accel_reset_sum();
	
	// STEP2: Get 400 samples at 400 HZ and sum x, y, z. FPU maintains sums and the sample count.
	for(int num_samples = 0; num_samples < ACCEL_NUM_SAMPLES_BIAS; num_samples++)
	{
		//fpu_wait(); // Don't overflow the 256 byte instruction buffer
		accel_read_vector_and_sum();
		
		#ifdef MEGANEURA_DEBUG
		pc_send_accel_raw_vector();
		#endif
	
		_delay_us(2500);
	}
	
	fpu_wait();
	fpu_eeprom_function_call(FPU_EEPROM_FUNC_ACCEL_COMPUTE_BIAS);
	
	// STEP4: Zero the sample count and sample sums
	accel_reset_sum();
}

/* ----------------------------------------------------------------------------
   FPU function to calculate the offset centered on (0, 0, -9.80665)
     - generated by FPU IDE
	 - readable function implemented in .fpu file
   ---------------------------------------------------------------------------- */
void write_accel_compute_bias_function_to_fpu_eeprom(void)
{
	fpu_wait();
    fpu_write4(EEWRITE, FPU_EEPROM_FUNC_ACCEL_COMPUTE_BIAS, 0x38, 0x37);
    fpu_write4(0x01, 0x2B, 0x3A, 0x00);
    fpu_write4(0x82, 0x51, 0x2F, 0xDC);
    fpu_write4(0x01, 0xFC, 0xDC, 0x02);
    fpu_write4(0xFB, 0xDC, 0x03, 0xFA);
    fpu_write4(0xDC, 0x04, 0xEF, 0x01);
    fpu_write4(0x28, 0x25, 0x2B, 0x24);
    fpu_write4(0x01, 0x36, 0xFF, 0xDA);
    fpu_write4(0x28, 0xFF, 0x01, 0x29);
    fpu_write4(0x25, 0x2B, 0x24, 0x02);
    fpu_write4(0x36, 0xFF, 0xDA, 0x29);
    fpu_write4(0xFE, 0x01, 0x2A, 0x25);
    fpu_write4(0x2B, 0x24, 0x03, 0x36);
    fpu_write4(0xFF, 0x21, 0x04, 0xDA);
    fpu_write3(0x2A, 0xFD, 0x80);
    fpu_wait();
}
/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "fpu.h"
	
/* ================================================================================================
   Externs
   ================================================================================================ */

	extern SPI_Master_t 			spiMasterE;
	extern VectorDouble3 			accel_scale;
	#ifdef HMC5883L
	extern VectorDouble3 			mag_offset;
	#endif
	extern uint32_t temp_time;
	extern double rotation_speed_factor;
	
/* ================================================================================================
   Globals
   ================================================================================================ */

	// Instantiate pointers to SPI, SPI port and SPI master
	PORT_t *ssPort = &PORTE;
	SPI_t *ssSPI = &SPIE;
	SPI_Master_t *ssSPIMaster = &spiMasterE;

/* ================================================================================================
   Application functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Load constant values to the FPU
   ---------------------------------------------------------------------------- */
int fpu_load_constants(void)
{
	// Gyroscope
	// Start relative heading is 0, will not change until gyro detects more the 
	// a +/-1 degree per second rotational velocity change
	fpu_write_int_to_reg_nn(0, FPU_REG_GYRO_HEADING);
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Init default data to be stored in FPU EEPROM
   ---------------------------------------------------------------------------- */
void fpu_init_eeprom_defaults(void)
{
	// Accelerometer
	// scaled to 1g = 256 ADC units (range is +/-4g or +/- 1024 ADC units)
	accel_scale.x = 0.037498492;
	accel_scale.y = 0.036905318;
	accel_scale.z = 0.037569102;
	
	#ifdef HMC5883L
	// Magnetometer
	// Previously calculated offset based off of raw data (216,-101,418)
	// Recently calculated (x,y) offset based off of measured data (-58.106, -102.9005)
	// Measured data had a scale applied of (0.981, 0.985) that was calculated automatically
	// using the self-test feature of HMC5843. The data was collected by rotating the board
	// 360 degrees in the xy plane. Did not bother with z axis since it doesn't matter in my
	// case
	mag_offset.x = -58.106;
	mag_offset.y = -102.9005;
	mag_offset.z = 0;
	#endif
}

/* ----------------------------------------------------------------------------
   Load data into the FPU's EEPROM
     - first we load the functions
	 - second we load the 32bit values
   ---------------------------------------------------------------------------- */
void fpu_load_eeprom_data(void)
{
	// Write FPU functions to EEPROM
	write_accel_init_fourth_order_function_to_fpu_eeprom();
	write_mag_init_heading_fusion_function_to_fpu_eeprom();
	write_gyro_reset_sum_function_to_fpu_eeprom();
	write_accel_reset_sum_function_to_fpu_eeprom();
	write_accel_compute_bias_function_to_fpu_eeprom();
	
	// NOTE: Accel offset is written to FPU EEPROM during the compute accel bias
	// function on first time boot
	
	// Accel scale is calculated by hand and so we copy it to FPU EEPROM
	// on first time boot
	fpu_wait();
	fpu_eeprom_write_double_to_slot(accel_scale.x, FPU_EEPROM_SLOT_ACCEL_SCALE_X);
	fpu_eeprom_write_double_to_slot(accel_scale.y, FPU_EEPROM_SLOT_ACCEL_SCALE_Y);
	fpu_eeprom_write_double_to_slot(accel_scale.z, FPU_EEPROM_SLOT_ACCEL_SCALE_Z);
	
	// NOTE: Gyro offset is written to FPU EEPROM during the gyro calibrate function
	
	// Gyro scale is the same for all three axes (radians(1.0 / 14.375))
	fpu_eeprom_write_double_to_slot(radians(GYROSCOPE_GAIN), FPU_EEPROM_SLOT_GYRO_SCALE);
	
	#ifdef HMC5883L
	// Mag offset is calculated by hand and so we copy it to FPU EEPROM
	fpu_eeprom_write_double_to_slot(mag_offset.x, FPU_EEPROM_SLOT_MAG_OFFSET_X);
	fpu_eeprom_write_double_to_slot(mag_offset.y, FPU_EEPROM_SLOT_MAG_OFFSET_Y);
	fpu_eeprom_write_double_to_slot(mag_offset.z, FPU_EEPROM_SLOT_MAG_OFFSET_Z);
	#endif
	
	// NOTE: Mag scale is calculated using HMC5843 self test feature during initialization
	
	// Constants
	// Negative 1g
	fpu_eeprom_write_double_to_slot(NEG_ONE_G, FPU_EEPROM_SLOT_CONST_NEG_ONE_G);
	
	// LPF - b0 to b4 and a1 to a4
	fpu_eeprom_write_double_to_slot(_b0, FPU_EEPROM_REG_LPF_CHEBY2_B0);
	fpu_eeprom_write_double_to_slot(_b1, FPU_EEPROM_REG_LPF_CHEBY2_B1);
	fpu_eeprom_write_double_to_slot(_b2, FPU_EEPROM_REG_LPF_CHEBY2_B2);
	fpu_eeprom_write_double_to_slot(_b3, FPU_EEPROM_REG_LPF_CHEBY2_B3);
	fpu_eeprom_write_double_to_slot(_b4, FPU_EEPROM_REG_LPF_CHEBY2_B4);
	
	fpu_eeprom_write_double_to_slot(_a1, FPU_EEPROM_REG_LPF_CHEBY2_A1);
	fpu_eeprom_write_double_to_slot(_a2, FPU_EEPROM_REG_LPF_CHEBY2_A2);
	fpu_eeprom_write_double_to_slot(_a3, FPU_EEPROM_REG_LPF_CHEBY2_A3);
	fpu_eeprom_write_double_to_slot(_a4, FPU_EEPROM_REG_LPF_CHEBY2_A4);
}

/* ================================================================================================
   Initialization and reset functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Initialize FPU
     - setup SPI communication
	 - reset the FPU so it remains in sync with the MCU
   ---------------------------------------------------------------------------- */
int fpu_initialize(void)
{
	// Init SS pin as output with wired AND and pull-up
	ssPort->DIRSET = SPI_SS_bm;
	ssPort->PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	// Set SS output to high. (No slave addressed)
	ssPort->OUTSET = SPI_SS_bm;
	
	if( ERR_SUCCESS != fpu_reset() )
	{
		report_error(ERR_FPU_RESET);
		return ERR_FPU_INIT;
	}
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Reset FPU
     - To reset the FPU send 10 consecutive 0xFF bytes (reset occurs after 9 but 10 is recommended)
	 - The reset does not occur until the SIN signal line goes low. If SIN remains high after the reset
       a 0x00 byte must be sent (or SIN set low) to trigger the reset. NOTE: If the SIN line does not go
	   low after within 100ms of receiving nine 0xFF bytes, a reset will be triggered by default.
	 - Delay 10ms to ensure that the reset is complete and the FPU is ready to receive commands.
	 
	 NOTE: All FPU registers are reset to NaN (0x7FFFFFFF)
	 NOTE: SIN pin on FPU is the MOSI pin on MCU
   ---------------------------------------------------------------------------- */
int fpu_reset(void)
{
	ssPort->OUTCLR = SPI_SS_bm;			// SS low
	
	ssPort->OUTSET = SPI_MOSI_bm;		// MOSI high
	
	for (byte i = 0; i < 80; i++)
	{
		ssPort->OUTSET = SPI_SCK_bm;	// SCK high
		ssPort->OUTCLR = SPI_SCK_bm;	// SCK low
	}
	
	ssPort->OUTCLR = SPI_MOSI_bm;		// MOSI low
	
	_delay_ms(10);
	
	// Initialize SPI master on port C (max speed is of um-fpu v3.1 SPI is 15mhz)
	SPI_MasterInit(ssSPIMaster,
	               ssSPI,
	               ssPort,
	               false,
	               SPI_MODE_0_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV4_gc);  // 32 / 4 = 8 MHZ
				   
	ssPort->OUTSET = SPI_SS_bm;			// SS high
	
	if( ERR_SUCCESS != fpu_sync() )
	{
		report_error(ERR_FPU_SYNC);
		return ERR_FPU_INIT;
	}
	
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   Read the sync char from FPU and verify
   ---------------------------------------------------------------------------- */
int fpu_sync(void)
{
	uint8_t masterReceivedByte = 0;
	
	fpu_write(SYNC);
	masterReceivedByte = fpu_read();
	
	if (masterReceivedByte == SYNC_CHAR)
	{
		return ERR_SUCCESS;
	}
	else
	{
		return ERR_FPU_INIT;
	}
}

/* ================================================================================================
   Read functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   FPU read delay
   ---------------------------------------------------------------------------- */
void fpu_read_delay(void)
{
	_delay_us(FPU_READ_DELAY); // *** THIS IS CRITICAL *** 
}

/* ----------------------------------------------------------------------------
   FPU wait
     - FPU has a 256 byte instruction buffer
	 - The buffer will overflow if instructions in a calculation exceed 256 bytes
	 - The code generated by the FPU IDE accounts for this and inserts waits
	 - Read operations automatically generate a wait sequence, so in many 
	   applications, no additional wait sequences are required
   ---------------------------------------------------------------------------- */
void fpu_wait(void)
{
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	_delay_us(FPU_WAIT_DELAY);
	while( ( ssPort->IN & SPI_MISO_bm ) );
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU read byte
   ---------------------------------------------------------------------------- */
uint8_t fpu_read(void)
{
	uint8_t temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	fpu_read_delay();
	temp = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
	return temp;
}

/* ----------------------------------------------------------------------------
   FPU read word (int)
   ---------------------------------------------------------------------------- */
int fpu_read_word(void)
{
	WORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	fpu_read_delay();
	temp.v[1] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	temp.v[0] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
	return temp._int;
}

/* ----------------------------------------------------------------------------
   FPU read long (32 bits)
   ---------------------------------------------------------------------------- */
long fpu_read_long(void)
{
	DWORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	fpu_read_delay();
	temp.v[3] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	temp.v[2] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	temp.v[1] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	temp.v[0] = SPI_MasterTransceiveByte(&spiMasterE, 0x00);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
	return temp._long;
}

/* ----------------------------------------------------------------------------
   FPU read double
     - Read 5 bytes
	 - Byte 1: is size of data 0x20 = 32 bits
	 - Bytes 2-5: double value
	 - The documentation says read 4 bytes but that is not the case
   ---------------------------------------------------------------------------- */
double fpu_read_double(void)
{
	fpu_wait(); // this is CRITICAL
	fpu_write(FREADA);
	fpu_read_delay();
	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[4];
	uint8_t masterReceivedData[4];
	
	masterSendData[0] = 0;
	masterSendData[1] = 0;
	masterSendData[2] = 0;
	masterSendData[3] = 0;
	//masterSendData[4] = 0;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           5,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
	
	// Ignore the first byte of 0x20 which I believe is the size of the data to follow
	DWORD temp;
	temp.v[3] = masterReceivedData[0];
	temp.v[2] = masterReceivedData[1];
	temp.v[1] = masterReceivedData[2];
	temp.v[0] = masterReceivedData[3];

	return temp._double;
}

/* ----------------------------------------------------------------------------
   FPU read double from register nn
     - Read 5 bytes
	 - Byte 1: is size of data 0x20 = 32 bits
	 - Bytes 2-5: double value
	 - The documentation says read 4 bytes but that is not the case
   ---------------------------------------------------------------------------- */
#ifdef MEGANEURA_DEBUG
uint32_t jat_wait_checks = 0;
#endif

double fpu_read_double_from_register(uint8_t register_num)
{
	fpu_write2(SELECTA, register_num);

	fpu_wait();
	#ifdef MEGANEURA_DEBUG
		time_fpu_read_wait = micros() - temp_time;

		usart_tx_string("wait ");
		usart_tx_ul(jat_wait_checks);
		usart_tx_string(": ");
		usart_tx_ul(time_fpu_read_wait);
		usart_tx_string(" ");
		
		const int reg = register_num;
		usart_tx_string("reg: ");
		usart_tx_int(&reg);
	jat_wait_checks++;
	#endif
			
	fpu_write(FREADA);
	
	fpu_read_delay();
	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[4];
	uint8_t masterReceivedData[4];
	
	masterSendData[0] = 0;
	masterSendData[1] = 0;
	masterSendData[2] = 0;
	masterSendData[3] = 0;
	//masterSendData[4] = 0;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           4,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
	
	// Ignore the first byte of 0x20 which I believe is the size of the data to follow
	DWORD temp;
	temp.v[3] = masterReceivedData[0];
	temp.v[2] = masterReceivedData[1];
	temp.v[1] = masterReceivedData[2];
	temp.v[0] = masterReceivedData[3];
	
	return temp._double;
}

/* ----------------------------------------------------------------------------
   FPU read byte
   ---------------------------------------------------------------------------- */
uint8_t fpu_read_byte(void)
{
	fpu_wait(); // this is CRITICAL
	fpu_write(LREADBYTE);
	fpu_read_delay();
	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[1];
	uint8_t masterReceivedData[1];
	
	masterSendData[0] = 0;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           1,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
	
	return masterReceivedData[0];

}

/* ----------------------------------------------------------------------------
   FPU read int (word)
   ---------------------------------------------------------------------------- */
uint16_t fpu_read_int(void)
{
	fpu_wait(); // this is CRITICAL
	fpu_write(LREADWORD);
	fpu_read_delay();
	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[2];
	uint8_t masterReceivedData[2];
	
	masterSendData[0] = 0;
	masterSendData[1] = 0;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           2,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
	
	WORD temp;
	temp.v[1] = masterReceivedData[0];
	temp.v[0] = masterReceivedData[1];
	
	return temp._uint16_t;
}

/* ================================================================================================
   Write functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   FPU write double
   ---------------------------------------------------------------------------- */
void fpu_write_double(double value)
{
	DWORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	temp._double = value;
	fpu_write4(temp.v[3], temp.v[2], temp.v[1], temp.v[0]);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write long
   ---------------------------------------------------------------------------- */
void fpu_write_long(long value)
{
	DWORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	temp._long = value;
	fpu_write4(temp.v[3], temp.v[2], temp.v[1], temp.v[0]);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write int
   ---------------------------------------------------------------------------- */
void fpu_write_int(int value)
{
	WORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	temp._int = value;
	fpu_write2(temp.v[1], temp.v[0]);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write byte
   ---------------------------------------------------------------------------- */
void fpu_write(uint8_t value)
{
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	SPI_MasterTransceiveByte(ssSPIMaster, value);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write 2 instructions
   ---------------------------------------------------------------------------- */
void fpu_write2(uint8_t value, uint8_t value2)
{	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[2];
	uint8_t masterReceivedData[2];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           2,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write 3 instructions
   ---------------------------------------------------------------------------- */
void fpu_write3(uint8_t value, uint8_t value2, uint8_t value3)
{
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[3];
	uint8_t masterReceivedData[3];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	masterSendData[2] = value3;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           3,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write 4 instructions
   ---------------------------------------------------------------------------- */
void fpu_write4(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4)
{	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[4];
	uint8_t masterReceivedData[4];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	masterSendData[2] = value3;
	masterSendData[3] = value4;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           4,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write 5 instructions
   ---------------------------------------------------------------------------- */
void fpu_write5(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5)
{	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[5];
	uint8_t masterReceivedData[5];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	masterSendData[2] = value3;
	masterSendData[3] = value4;
	masterSendData[4] = value5;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           5,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write 6 instructions
   ---------------------------------------------------------------------------- */
void fpu_write6(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5, uint8_t value6)
{	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[6];
	uint8_t masterReceivedData[6];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	masterSendData[2] = value3;
	masterSendData[3] = value4;
	masterSendData[4] = value5;
	masterSendData[5] = value6;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           6,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write 6 instructions
   ---------------------------------------------------------------------------- */
void fpu_write7(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5, uint8_t value6, uint8_t value7)
{	
	SPI_DataPacket_t dataPacket;
	uint8_t masterSendData[7];
	uint8_t masterReceivedData[7];
	
	masterSendData[0] = value;
	masterSendData[1] = value2;
	masterSendData[2] = value3;
	masterSendData[3] = value4;
	masterSendData[4] = value5;
	masterSendData[5] = value6;
	masterSendData[6] = value7;
	
	SPI_MasterCreateDataPacket(&dataPacket,
	                           masterSendData,
	                           masterReceivedData,
	                           7,
	                           ssPort,
	                           SPI_SS_bm);
							   
	SPI_MasterTransceivePacket(ssSPIMaster, &dataPacket);
}

/* ----------------------------------------------------------------------------
   FPU write int to register nn
   ---------------------------------------------------------------------------- */
void fpu_write_int_to_reg_nn(int value, uint8_t regnn)
{
	WORD temp;
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	temp._int = value;
	fpu_write2(SELECTA, regnn);
	fpu_write4(LOADWORD, temp.v[1], temp.v[0], FSET0);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write double to register nn
   ---------------------------------------------------------------------------- */
void fpu_write_double_to_reg_nn(double value, uint8_t regnn)
{
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	
	fpu_write2(FWRITE, regnn);
	fpu_write_double(value);
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   FPU write byte to register nn (not tested)
   ---------------------------------------------------------------------------- */
void fpu_write_uint8_to_reg_nn(uint8_t value, uint8_t regnn)
{
	SPI_MasterSSLow(ssPort, SPI_SS_bm);
	fpu_write4(SELECTA, regnn, FSETI, value); // Select register a then set immediate value (signed byte even though uint8_t above)
	SPI_MasterSSHigh(ssPort, SPI_SS_bm);
}

/* ----------------------------------------------------------------------------
   Write double to EEPROM slot
   ---------------------------------------------------------------------------- */
void fpu_eeprom_write_double_to_slot(double value, uint8_t slotNumber)
{
	fpu_write_double_to_reg_nn(value, FPU_REG_TEMPORARY_1);
	fpu_wait();
    fpu_write3(EESAVE, FPU_REG_TEMPORARY_1, slotNumber);
	fpu_wait();
}

/* ================================================================================================
   General functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   FPU read status byte (used by many instructions)
   ---------------------------------------------------------------------------- */
uint8_t fpu_read_status(void)
{
	fpu_write(READSTATUS);
	fpu_wait();
	return fpu_read();
}

/* ================================================================================================
   String functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Read FPU version number string
   ---------------------------------------------------------------------------- */
int fpu_read_version(char *pVersion)
{
	fpu_write(VERSION);
	fpu_wait();
	fpu_get_string(pVersion);
	return ERR_SUCCESS;
}

/* ----------------------------------------------------------------------------
   FPU read string
   ---------------------------------------------------------------------------- */
void fpu_get_string(char *pVersion)
{
	uint8_t masterReceivedByte = 0;
	
	fpu_wait();
	fpu_write(READSTR);
	fpu_read_delay();
	
	for(;;)
	{
		masterReceivedByte = fpu_read();
		*pVersion = masterReceivedByte;
		pVersion++;
		if(masterReceivedByte == 0) break;
	}
}

/* ----------------------------------------------------------------------------
   FPU ftoa - double stored in register temporary_1 to string
   ---------------------------------------------------------------------------- */
void fpu_ftoa(uint8_t format, double value, char *pStringValue)
{
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);
	fpu_write(FWRITEA);
	fpu_write_double(value);
	fpu_ftoa_reg_a(0, pStringValue);
}

/* ----------------------------------------------------------------------------
   FPU ftoa - double stored in register A to string
   ---------------------------------------------------------------------------- */
void fpu_ftoa_reg_a(uint8_t format, char *pStringValue)
{
	fpu_write2(FTOA, format);
	fpu_get_string(pStringValue);
}

/* ----------------------------------------------------------------------------
   FPU ltoa - long stored in register A to string
   ---------------------------------------------------------------------------- */
void fpu_ltoa_reg_a(uint8_t format, char *pStringValue)
{
	fpu_write2(LTOA, format);
	fpu_get_string(pStringValue);
}

/* ================================================================================================
   FPU function calls
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Call FPU function stored in flash memory
   ---------------------------------------------------------------------------- */
void fpu_function_call(uint8_t functionNumber)
{
	fpu_write2(FCALL, functionNumber);
}

/* ----------------------------------------------------------------------------
   Call FPU function stored in EEPROM memory
   ---------------------------------------------------------------------------- */
void fpu_eeprom_function_call(uint8_t functionNumber)
{
	fpu_write2(EECALL, functionNumber);
}

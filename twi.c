/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "twi.h"

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	#ifdef HMC5883L
	TWI_Master_t twi_master_mag;	// TWI master module for magnetometer on PD0(SDA) and PD1(SCL)
	#endif
	TWI_Master_t twi_master_gyro;	// TWI master module for gyroscope on PE0(SDA) and PE1(SCL)

/* ================================================================================================
   Initialization functions
   ================================================================================================ */
   
void twi_initialize(void)
{	
	// Initialize TWI masters
	#ifdef HMC5883L
	TWI_MasterInit(&twi_master_mag,
					&TWID, 						// TWI on port D
					TWI_MASTER_INTLVL_LO_gc, 	// Enable low interrupt level
					TWI_BAUDSETTING);
	#endif
	
	TWI_MasterInit(&twi_master_gyro,
					&TWIE, 						// TWI on port E
					TWI_MASTER_INTLVL_LO_gc, 	// Enable low interrupt level
					TWI_BAUDSETTING);
}

/* ================================================================================================
   Interrupt routines
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   TWID Master Interrupt vector
   ---------------------------------------------------------------------------- */
#ifdef HMC5883L
ISR(TWID_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twi_master_mag);
}
#endif

/* ----------------------------------------------------------------------------
   TWIE Master Interrupt vector
   ---------------------------------------------------------------------------- */
ISR(TWIE_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twi_master_gyro);
}

/* ================================================================================================
   Error processing functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Process TWI (I2C) error
   ---------------------------------------------------------------------------- */
int process_twi_error(register8_t *pError)
{
	switch(*pError)
	{
		case 0x00:
			return ERR_TWI_UNKNOWN;
		case 0x02:
			return ERR_TWI_BUF_OVERFLOW;
		case 0x03:
			return ERR_TWI_ARB_LOST;
		case 0x04:
			return ERR_TWI_BUS_ERROR;
		case 0x05:
			return ERR_TWI_NACK;
		case 0x06:
			return ERR_TWI_FAIL;
		default:
			return ERR_INVALID_TWI_RESULT;
	}
}
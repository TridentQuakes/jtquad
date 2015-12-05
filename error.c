/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "error.h"

/* ================================================================================================
   Globals
   ================================================================================================ */

	const Error_t errors[NUM_ERRORS] = {
		{ERR_SUCCESS, "success"},
		{ERR_NULL_POINTER, "null pointer"},
		{ERR_INVALID_OUTPUT_MODE, "invalid output mode"},
		{ERR_NOT_IMPLEMENTED, "not implemented"},
		{ERR_INVALID_EXTERNAL_COMMAND, "unrecognized external command"},
		{ERR_TWI_UNKNOWN, "twi error unknown"},
		{ERR_TWI_BUF_OVERFLOW, "twi buffer overflow"},
		{ERR_TWI_ARB_LOST, "twi arbitration lost"},
		{ERR_TWI_BUS_ERROR, "twi bus error"},
		{ERR_TWI_NACK, "twi NACK received"},
		{ERR_TWI_FAIL, "twi fail"},
		{ERR_ADXL345_INIT, "initializing ADXL345"},
		{ERR_ADXL345_READ, "reading register of ADXL345"},
		{ERR_ADXL345_WRITE, "writing to register of ADXL345"},
		{ERR_ADXL345_INVALID_ID, "invalid device ID for ADXL345"},
		{ERR_HMC5843_INIT, "initializing HMC5843"},
		{ERR_HMC5843_READ, "reading from register of HMC5843"},
		{ERR_HMC5843_WRITE, "writing to register of HMC5843"},
		{ERR_HMC5843_INVALID_ID, "invalid device ID for HMC5843"},
		{ERR_INVALID_ERROR_NUMBER, "invalid error number"},
		{ERR_INVALID_TWI_RESULT, "invalid twi result"},
		{ERR_EXECUTING_EXTERNAL_COMMAND, "error executing external command"},
		{ERR_HMC5843_SELF_TEST, "error executing self test to calculate scale vector to 0.55ga"},
		{ERR_MAX_COMMAND_LENGTH, "error max command length of 30 exceeded"},
		{ERR_FPU_INIT, "initializing fpu"},
		{ERR_FPU_READ, "reading from fpu"},
		{ERR_FPU_WRITE, "writing to fpu"},
		{ERR_OVERFLOW, "string overflow"},
		{ERR_FPU_GET_VERSION, "reading the fpu version string"},
		{ERR_FPU_SYNC, "reading sync from fpu"},
		{ERR_ITG3200_INIT, "initializing ITG3200"},
		{ERR_ITG3200_READ, "reading from register of ITG3200"},
		{ERR_ITG3200_WRITE, "writing to register of ITG3200"},
		{ERR_ITG3200_INVALID_ID, "ITG3200 who am I? failed"},
		{ERR_TMP102_READ, "reading register of TMP102"},
		{ERR_FPU_RESET, "resetting um-FPU v3.1"},
	};

/* ================================================================================================
   Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Report error
   ---------------------------------------------------------------------------- */
int report_error(int errorNum)
{
	return pc_report_error(errorNum);
}

/* ----------------------------------------------------------------------------
   Get error index
   ---------------------------------------------------------------------------- */
int get_error_index(int errorNum, int *pIndex)
{
	int i = -1;
	for(i = 0; i < NUM_ERRORS; i++)
	{
		if( errors[i].errorNum == errorNum )
		{
			break;
		}
	}
	
	if( i > NUM_ERRORS || i < 0)
	{
		return ERR_INVALID_ERROR_NUMBER;
	}
	
	*pIndex = i;
	
	return ERR_SUCCESS;
}

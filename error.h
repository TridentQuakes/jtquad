/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _ERROR_H_
#define _ERROR_H_

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "common.h"
	
/* ================================================================================================
   Defines
   ================================================================================================ */

	// General
	#define MAX_ERROR_MSG_LENGTH				100
	#define NUM_ERRORS 							36

	// General system errors
	#define ERR_SUCCESS 						0x0000

	#define ERR_NULL_POINTER 					0x0001
	#define ERR_INVALID_OUTPUT_MODE 			0x0002
	#define ERR_NOT_IMPLEMENTED					0x0003
	#define ERR_INVALID_EXTERNAL_COMMAND 		0x0004
	#define ERR_INVALID_ERROR_NUMBER			0x0005
	#define ERR_INVALID_TWI_RESULT				0x0006
	#define ERR_EXECUTING_EXTERNAL_COMMAND		0x0007
	#define ERR_MAX_COMMAND_LENGTH				0x0008
	#define ERR_OVERFLOW						0x0009

	// TWI errors
	#define ERR_TWI_UNKNOWN 					0x0100
	#define ERR_TWI_BUF_OVERFLOW 				0x0102
	#define ERR_TWI_ARB_LOST	 				0x0103
	#define ERR_TWI_BUS_ERROR	 				0x0104
	#define ERR_TWI_NACK		 				0x0105
	#define ERR_TWI_FAIL		 				0x0106

	// ADXL345 errors
	#define ERR_ADXL345_INIT					0x1100
	#define ERR_ADXL345_READ					0x1101
	#define ERR_ADXL345_WRITE					0x1102
	#define ERR_ADXL345_INVALID_ID				0x1103

	// HMC5883L errors

	#define ERR_HMC5883L_INIT					0x1200
	#define ERR_HMC5883L_READ					0x1201
	#define ERR_HMC5883L_WRITE					0x1202
	#define ERR_HMC5883L_INVALID_ID				0x1203
	#define ERR_HMC5883L_SELF_TEST				0x1204

	// ITG3200 errors

	#define ERR_ITG3200_READ					0x1401
	#define ERR_ITG3200_WRITE					0x1402
	#define ERR_ITG3200_INIT					0x1403
	#define ERR_ITG3200_INVALID_ID				0x1404

	// FPU errors
	#define ERR_FPU_INIT						0x1300
	#define ERR_FPU_READ						0x1301
	#define ERR_FPU_WRITE						0x1302
	#define ERR_FPU_GET_VERSION					0x1303
	#define ERR_FPU_SYNC						0x1304
	#define ERR_FPU_EEPROM_CONSTANT_INVALID		0x1305
	#define ERR_FPU_RESET						0x1306

	// TMP102 errors
	#define ERR_TMP102_READ						0x1501
		
/* ================================================================================================
   Typedefs
   ================================================================================================ */

	typedef struct Error_Struct
	{
		int errorNum;
		char errorMsg[MAX_ERROR_MSG_LENGTH];
	} Error_t;

/* ================================================================================================
   Function Prototypes
   ================================================================================================ */

	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif
	
	int report_error(int errorNum);
	int get_error_index(int errorNum, int *pIndex);
	
	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif
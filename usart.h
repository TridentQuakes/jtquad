/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _USART_H_
#define _USART_H_

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "common.h"
	
/* ================================================================================================
   Defines
   ================================================================================================ */
   
		#define USART 				USARTC0
		#define USART_TX_TIMEOUT 	1000000
		#define USART_BAUD 			153600		// (e.g. 76800 yields UBBR/BSEL is 25 at normal speed)
	
/* ================================================================================================
   Macros
   ================================================================================================ */
   
		// IMPORTANT NOTE: See Table 21-5 in atxmega128a1 datasheet for more on bsel and bscale. 
		// These macros only work when bscale = 0. The table shows various speeds and values. Max
		// speed is 4.0M (0% error).
		
		// Macro for calculating the baud value from a given baud rate when the U2X (double speed) 
		// bit is not set.
		#define SERIAL_UBBRVAL(baud)    ((((F_CPU / 16) + (baud / 2)) / (baud)) - 1)

		// Macro for calculating the baud value from a given baud rate when the U2X (double speed) 
		// bit is set.
		#define SERIAL_2X_UBBRVAL(baud) ((((F_CPU / 8) + (baud / 2)) / (baud)) - 1)

/* ================================================================================================
   Globals
   ================================================================================================ */
   
	USART_data_t USART_data;	

/* ================================================================================================
   Function prototypes
   ================================================================================================ */
   
	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif

	// Initialization functions
	void usart_initialize(void);
	
	// Tx functions
	void usart_tx_int(const int *pValue);
	void usart_tx_ul(uint32_t value);
	void usart_tx_uint(uint16_t value);
	void usart_tx_double(double value, uint8_t width, uint8_t precision);
	
	// Tx FPU functions
	void usart_tx_double_from_fpu(uint8_t fpu_reg, uint8_t format);
	void usart_tx_double_from_fpu_eeprom(uint8_t slotNumber, uint8_t format);
	
	// Rx functions
	void usart_rx_value(uint8_t *data, uint8_t size);
	double usart_rx_double(void);
	
/* ================================================================================================
   Inline functions
   ================================================================================================ */

	/* ----------------------------------------------------------------------------
       Tx byte
       ---------------------------------------------------------------------------- */
	static inline void usart_tx_byte(const uint8_t DataByte)
	{
		int i = 0;

		while (i < USART_TX_TIMEOUT) 
		{
			bool byteToBuffer;

			byteToBuffer = USART_TXBuffer_PutByte(&USART_data, DataByte);

			if(byteToBuffer)
			{
				break;
			}
			
			i++;
		}
	}
	
	/* ----------------------------------------------------------------------------
       Tx string
       ---------------------------------------------------------------------------- */
	static inline void usart_tx_string(const char *StringPtr)
	{
		uint8_t CurrByte;

		while ((CurrByte = *StringPtr) != 0x00)
		{
			usart_tx_byte(CurrByte);
			StringPtr++;
		}
	}
	
	// Disable C linkage for C++ Compilers
	#if defined(__cplusplus)
	}
	#endif

#endif
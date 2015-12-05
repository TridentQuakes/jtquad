/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "usart.h"

/* ================================================================================================
   Initialization functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize USART
   ---------------------------------------------------------------------------- */
void usart_initialize(void)
{
	PORTC.DIRSET = PIN3_bm;			// PIN3 (TXD0) as output
	PORTC.DIRCLR = PIN2_bm;			// PC2 (RXD0) as input

	// Use USARTC0 and initialize buffers
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	// Format: 8 Data bits, No Parity, 1 Stop bit
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	// Enable RXC interrupt
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	// Enable double transmission speed
	//USART_2X_Enable(USART_data.usart);
	
	// Set Baudrate (see table 25-1 in datasheet for more)
	//USART_Baudrate_Set(USART_data.usart, SERIAL_2X_UBBRVAL(USART_BAUD), 0);	// UBBR macro only works with bscale = 0
	//USART_Baudrate_Set(USART_data.usart, 131, -3);							// 115200 baud at 2X
	USART_Baudrate_Set(USART_data.usart, 123, -4);								// 230400 baud at 1X
	
	// Enable both RX and TX
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);
}

/* ================================================================================================
   Interrupt routines
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Receive complete interrupt service routine
   
   Calls the common receive complete handler with pointer to the correct USART
   as argument.
   ---------------------------------------------------------------------------- */
ISR(USARTC0_RXC_vect)
{
	USART_RXComplete(&USART_data);
}

/* ----------------------------------------------------------------------------
   Data register empty  interrupt service routine
   
   Calls the common data register empty complete handler with pointer to the
   correct USART as argument. When register has room for more data the dreif 
   is set
   ---------------------------------------------------------------------------- */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}

/* ================================================================================================
   Tx functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Tx int
   ---------------------------------------------------------------------------- */
void usart_tx_int(const int *pValue)
{
	char sTemp[6] = {0};
	
	itoa(*pValue, sTemp, 10);
	usart_tx_string(sTemp);
}

/* ----------------------------------------------------------------------------
   Tx unsigned long
   ---------------------------------------------------------------------------- */
void usart_tx_ul(uint32_t value)
{
	char sTemp[11] = {0};
	ultoa(value, sTemp, 10);
	usart_tx_string(sTemp);
}

/* ----------------------------------------------------------------------------
   Tx unsigned int
   ---------------------------------------------------------------------------- */
void usart_tx_uint(uint16_t value)
{
	char sTemp[6] = {0};
	utoa(value, sTemp, 10);
	usart_tx_string(sTemp);
}

/* ----------------------------------------------------------------------------
   Tx double
	- this function uses dtostrf in math library libm.a, format "[-]d.ddd"
	- there is also function dtostre, format "[-]d.ddde±dd"
	- see avr gcc documentation for more (stdlib.h)
	- the minimum field width of the output string (including the '.' and the 
	  possible sign for negative values) is given in width, and prec determines
      the number of digits after the decimal sign. 
   ---------------------------------------------------------------------------- */
void usart_tx_double(double value, uint8_t width, uint8_t precision)
{
	char sTemp[13] = {0};
	//sprintf(sTemp2, "double: %lf\r\n", test);
	dtostrf(value, width, precision, sTemp);
	deblank(sTemp);
	usart_tx_string(sTemp);
}

/* ================================================================================================
   Tx FPU functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Tx double stored in FPU register
   ---------------------------------------------------------------------------- */
void usart_tx_double_from_fpu(uint8_t fpu_reg, uint8_t format)
{
	char sTemp[15] = {0};
	//fpu_wait();
	fpu_write2(SELECTA, fpu_reg);
	//fpu_wait();
	fpu_ftoa_reg_a(format, sTemp);
	deblank(sTemp);
	usart_tx_string(sTemp);
}

/* ----------------------------------------------------------------------------
   Tx double stored in FPU EEPROM
   ---------------------------------------------------------------------------- */
void usart_tx_double_from_fpu_eeprom(uint8_t slotNumber, uint8_t format)
{
	char sTemp[15] = {0};
	//fpu_wait();
	fpu_write3(EELOAD, FPU_REG_TEMPORARY_1, slotNumber);
	//fpu_wait();
	fpu_write2(SELECTA, FPU_REG_TEMPORARY_1);
	//fpu_wait();
	fpu_ftoa_reg_a(format, sTemp);
	deblank(sTemp);
	usart_tx_string(sTemp);
}

/* ================================================================================================
   Rx functions
   ================================================================================================ */
   
/* ----------------------------------------------------------------------------
   Read value/data from serial
   ---------------------------------------------------------------------------- */
void usart_rx_value(uint8_t *data, uint8_t size)
{
	uint8_t index = 0;
	uint8_t timeout = 0;
	data[0] = '\0';
	
	do
	{
		if( USART_RXBufferData_Available(&USART_data) == 0 )
		{
			_delay_us(1);
			timeout++;
		}
		else
		{
			data[index] = USART_RXBuffer_GetByte(&USART_data);
			timeout = 0;
			index++;
		}
		
	} while ( (index == 0 || data[index-1] != ';') && (timeout < 10) && (index < (size - 1) ) );
	
	data[index] = '\0';
}

/* ----------------------------------------------------------------------------
   Read double from serial
   ---------------------------------------------------------------------------- */
double usart_rx_double(void)
{
	uint8_t data[15] = {0};
	
	usart_rx_value(data, sizeof(data));
	return atof((char*)data);
}


 
   
   
   
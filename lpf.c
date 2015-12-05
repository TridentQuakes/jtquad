/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "lpf.h"

/* ================================================================================================
   Initialization functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize the 4th order low pass filter
   ---------------------------------------------------------------------------- */
void init_fourth_oder(void)
{
	//fpu_function_call(FPU_FUNC_ACCEL_INIT_FOURTH_ORDER);
	fpu_eeprom_function_call(FPU_EEPROM_FUNC_ACCEL_INIT_FOURTH_ORDER);
}

/* ================================================================================================
   Calculation functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Compute the 4th order low pass filter
   ---------------------------------------------------------------------------- */
void compute_fourth_order(void)
{
	// NOTE: Current vector input are the 3 values in the accelerometers 'ms2' fpu registers

	// cheby2(4,60,12.5/50)
	fpu_function_call(FPU_FUNC_ACCEL_COMPUTE_FOURTH_ORDER);
	
	// NOTE: Output vector as result from the fourth order filter are the 3 values 
	//       in the accelerometers 'filtered' fpu registers
}

/* ================================================================================================
   FPU functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Write the 4th order low pass filter function to FPU EEPROM
     - this function is implemented in .fpu file
	 - the FPU IDE generates the instructions below
   ---------------------------------------------------------------------------- */
void write_accel_init_fourth_order_function_to_fpu_eeprom(void)
{
	fpu_wait();
    fpu_write4(EEWRITE, FPU_EEPROM_FUNC_ACCEL_INIT_FOURTH_ORDER, 0x65, 0x64);
    fpu_write4(0xDC, 0x01, 0xEF, 0x01);
    fpu_write4(0x3A, 0x32, 0x00, 0x01);
    fpu_write4(0x3B, 0x32, 0x00, 0x01);
    fpu_write4(0x3C, 0x32, 0x00, 0x01);
    fpu_write4(0x3D, 0x32, 0x00, 0x01);
    fpu_write4(0x3E, 0x32, 0x00, 0x01);
    fpu_write4(0x3F, 0x32, 0x00, 0x01);
    fpu_write4(0x40, 0x32, 0x00, 0x01);
    fpu_write4(0x41, 0x32, 0x00, 0x01);
    fpu_write4(0x42, 0x32, 0x00, 0x01);
    fpu_write4(0x43, 0x32, 0x00, 0x01);
    fpu_write4(0x44, 0x32, 0x00, 0x01);
    fpu_write4(0x45, 0x32, 0x00, 0x01);
    fpu_write4(0x46, 0x32, 0x00, 0x01);
    fpu_write4(0x47, 0x32, 0x00, 0x01);
    fpu_write4(0x48, 0x32, 0x00, 0x01);
    fpu_write4(0x49, 0x32, 0x00, 0x01);
    fpu_write4(0x4A, 0x20, 0x01, 0x01);
    fpu_write4(0x4B, 0x20, 0x01, 0x01);
    fpu_write4(0x4C, 0x20, 0x01, 0x01);
    fpu_write4(0x4D, 0x20, 0x01, 0x01);
    fpu_write4(0x4E, 0x20, 0x01, 0x01);
    fpu_write4(0x4F, 0x20, 0x01, 0x01);
    fpu_write4(0x50, 0x20, 0x01, 0x01);
    fpu_write4(0x51, 0x20, 0x01, 0x80);
    fpu_wait();
}
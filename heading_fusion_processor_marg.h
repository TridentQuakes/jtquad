/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _HEADING_FUSION_PROCESSOR_MARG_H_
#define _HEADING_FUSION_PROCESSOR_MARG_H_

/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "common.h"

/* ================================================================================================
   Function prototypes
   ================================================================================================ */
   
	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif

	void initialize_heading_fusion(void);
	void calculate_heading(void);
	void write_mag_init_heading_fusion_function_to_fpu_eeprom(void);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif
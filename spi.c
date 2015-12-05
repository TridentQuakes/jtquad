/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */

	#include "spi.h"

/* ================================================================================================
   Globals
   ================================================================================================ */

	SPI_Master_t spiMasterD;		// SPI master on port D
	SPI_Master_t spiMasterE;		// SPI master on port E

/* ================================================================================================
   Interrupt routines
   ================================================================================================ */

 /* ----------------------------------------------------------------------------
   SPI master interrupt service routine
   
     The interrupt service routines calls one common function,
     SPI_MasterInterruptHandler(SPI_Master_t *spi),
     passing information about what module to handle.
	 
	 Similar ISRs must be added if other SPI modules are to be used.
   ---------------------------------------------------------------------------- */
ISR(SPID_INT_vect)
{
	SPI_MasterInterruptHandler(&spiMasterD);
}

/* ----------------------------------------------------------------------------
   SPI master interrupt service routine
   
     The interrupt service routines calls one common function,
     SPI_MasterInterruptHandler(SPI_Master_t *spi),
     passing information about what module to handle.
	 
	 Similar ISRs must be added if other SPI modules are to be used.
   ---------------------------------------------------------------------------- */
ISR(SPIE_INT_vect)
{
	SPI_MasterInterruptHandler(&spiMasterE);
}


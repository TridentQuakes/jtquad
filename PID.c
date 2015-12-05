/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   Includes
   ================================================================================================ */
   
    #include "PID.h"

/* ================================================================================================
   Externs
   ================================================================================================ */

	extern uint32_t 	current_time;
	extern bool 		in_flight;
	
/* ================================================================================================
   Public Functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   PID update
   ---------------------------------------------------------------------------- */
double PID_update(double targetPosition, double currentPosition, struct PIDdata *PIDparameters)
{
	// AKA PID experiments
	const double deltaPIDTime = (current_time - PIDparameters->previousPIDTime) / 1000000.0;

	PIDparameters->previousPIDTime = current_time;
	double error = targetPosition - currentPosition;

	if( in_flight )
	{
		PIDparameters->integratedError += error * deltaPIDTime;
	}
	else 
	{
		PIDparameters->integratedError = 0.0;
	}
	
	PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
	double dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastError) / (deltaPIDTime * 100);
	PIDparameters->lastError = currentPosition;

	return (PIDparameters->P * error) + (PIDparameters->I * PIDparameters->integratedError) + dTerm;
}

/* ----------------------------------------------------------------------------
   Zero integral error
   ---------------------------------------------------------------------------- */

void zero_integral_error(void) 
{
	for (uint8_t axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++)
	{
		PID[axis].integratedError = 0;
		PID[axis].previousPIDTime = current_time;
	}
}

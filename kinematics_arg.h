/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================
   NOTE: Kinematics ARG + plus fusion of magnetometer. In the future try to implement MARG. 
         quad had a rough time with it. Needs more research.
		 
	There are many ways to calculate kinematics: ARG, MARG and DCM
	
	DCM: 			Direct cosine matrix
	ARG sensors: 	Accelerometer, Rate Gyroscope
	MARG sensors: 	Magnetometer, Accelerometer, Rate Gyroscope
	
	Euler angles:	Describe the orientation of rigid body
   ================================================================================================ */
 
#ifndef __KINEMATICS_ARG_H__
#define __KINEMATICS_ARG_H__

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"
	
/* ================================================================================================
   Function prototypes
   ================================================================================================ */

	// Enable C linkage for C++ Compilers
	#if defined(__cplusplus)
	extern "C" {
	#endif

	// Initialization functions
	void initialize_base_kinematics_param(void);	
	void initialize_kinematics(void);
	
	// Calculation functions
	void calculate_kinematics(void);
	void arg_update(void);
	void euler_angles(void);
	
	// Calibration functions (not used)
	void calibrate_kinematics(void);
	
	#if defined(__cplusplus)
	}
	#endif
	
#endif

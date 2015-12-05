/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

/* ================================================================================================

	Quaternion implementation of the 'DCM filter' [Mayhony et al].
	
	'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
	orientation.  See my report for an overview of the use of quaternions in this application.
	
	'exInt', 'eyInt', 'ezInt' scaled integral error
	
	'Kp' proportional gain governs rate of convergence to accelerometer/magnetometer (filter gain)
	
	'Ki'integral gain governs rate of convergence of gyroscope biases (filter gain)
	
	'halfT' half the sample period
	
	User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'
	
	Gyroscope units are radians/second, accelerometer units are irrelevant as the vector is normalised.
	
   ================================================================================================ */


/* ================================================================================================
   Includes
   ================================================================================================ */
   
	#include "kinematics_arg.h"

/* ================================================================================================
   Defines
   ================================================================================================ */
   
	#define KP 		0.2
	#define KI		0.0005

/* ================================================================================================
   Externs
   ================================================================================================ */
   
	extern double 	G_Dt;
	
/* ================================================================================================
   Initialization functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Initialize base kinematic parameters
   ---------------------------------------------------------------------------- */
void initialize_base_kinematics_param(void)
{
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_ANGLE_X);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_ANGLE_Y);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_ANGLE_Z);
}

/* ----------------------------------------------------------------------------
   Initialize kinematic parameters
   ---------------------------------------------------------------------------- */
void initialize_kinematics(void)
{
	initialize_base_kinematics_param();
	
	fpu_write_int_to_reg_nn(1, FPU_REG_KINEMATICS_Q0);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_Q1);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_Q2);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_Q3);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_EXINT);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_EYINT);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_EZINT);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_PREVIOUS_EX);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_PREVIOUS_EY);
	fpu_write_int_to_reg_nn(0, FPU_REG_KINEMATICS_PREVIOUS_EZ);
}

/* ================================================================================================
   Calculation functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Calculate kinematics
   ---------------------------------------------------------------------------- */
void calculate_kinematics(void)
{
	arg_update();
	euler_angles();
}

/* ----------------------------------------------------------------------------
   ARG update
   ---------------------------------------------------------------------------- */
void arg_update(void)
{
	fpu_write_double_to_reg_nn(G_Dt, FPU_REG_TEMPORARY_1);		// halfT is temporary_1 in function ARG_UPDATE
	fpu_write_double_to_reg_nn(KI, FPU_REG_TEMPORARY_13);		// Ki is temporary_13 in function ARG_UPDATE
	fpu_write_double_to_reg_nn(KP, FPU_REG_TEMPORARY_14);		// Kp is temporary_14 in function ARG_UPDATE
	fpu_function_call(FPU_FUNC_KINEMATICS_ARG_UPDATE);
}

/* ----------------------------------------------------------------------------
   Calculate Euler angles
   ---------------------------------------------------------------------------- */
void euler_angles(void)
{
	fpu_function_call(FPU_FUNC_KINEMATICS_EULER_ANGLES);
	
	// flip y and z axes to keep with our coordinate system
	//fpu_write3(SELECTA, FPU_REG_KINEMATICS_ANGLE_Y, FNEG);
	//fpu_write3(SELECTA, FPU_REG_KINEMATICS_ANGLE_Z, FNEG);
}

/* ================================================================================================
   Calibration functions
   ================================================================================================ */

/* ----------------------------------------------------------------------------
   Calibrate kinematics (not implemented)
   ---------------------------------------------------------------------------- */
void calibrate_kinematics() { }
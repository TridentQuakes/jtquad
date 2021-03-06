/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */

#ifndef _FPU_H_
#define _FPU_H_

/* ================================================================================================
   Includes
   ================================================================================================ */
	
	#include "common.h"
	
/* ================================================================================================
   Application Defines
   ================================================================================================ */

	/* ----------------------------------------------------------------------------
	   FPU registers (0 to 127)
	   ---------------------------------------------------------------------------- */
	   
	// FPU has 128 registers (register 0 is reserved since it is used by many operations)
	// FPU has 8 temporary registers that can be used only with LEFT and RIGHT operations

	// RESERVED: register used temporarily by many fpu calculations (it can be used if needed)
	#define FPU_REG_0											0

	// Temporary appliation registers (used by many functions)
	#define FPU_REG_TEMPORARY_1									1				
	#define FPU_REG_TEMPORARY_2									2
	#define FPU_REG_TEMPORARY_3									3
	#define FPU_REG_TEMPORARY_4									4
	#define FPU_REG_TEMPORARY_5									5
	#define FPU_REG_TEMPORARY_6									6
	#define FPU_REG_TEMPORARY_7									7
	#define FPU_REG_TEMPORARY_8									8
	#define FPU_REG_TEMPORARY_9									9
	#define FPU_REG_TEMPORARY_10								10
	#define FPU_REG_TEMPORARY_11								11
	#define FPU_REG_TEMPORARY_12								12
	#define FPU_REG_TEMPORARY_13								13
	#define FPU_REG_TEMPORARY_14								14
	#define FPU_REG_TEMPORARY_15								15
	#define FPU_REG_TEMPORARY_16								16
	#define FPU_REG_TEMPORARY_17								17
	#define FPU_REG_TEMPORARY_18								18
	#define FPU_REG_TEMPORARY_19								19
	#define FPU_REG_TEMPORARY_20								20
	#define FPU_REG_TEMPORARY_21								21
	#define FPU_REG_TEMPORARY_22								22
	#define FPU_REG_TEMPORARY_23								23
	#define FPU_REG_TEMPORARY_24								24
	#define FPU_REG_TEMPORARY_25								25
	#define FPU_REG_TEMPORARY_26								26
	#define FPU_REG_TEMPORARY_27								27
	#define FPU_REG_TEMPORARY_28								28
	#define FPU_REG_TEMPORARY_29								29
	#define FPU_REG_TEMPORARY_30								30

	// Accelerometer
	#define FPU_REG_ACCEL_X										31
	#define FPU_REG_ACCEL_Y										32
	#define FPU_REG_ACCEL_Z										33
	#define FPU_REG_ACCEL_MS2_X									37
	#define FPU_REG_ACCEL_MS2_Y									38
	#define FPU_REG_ACCEL_MS2_Z									39
	#define FPU_REG_ACCEL_X_SUM_OF_SAMPLES						40
	#define FPU_REG_ACCEL_Y_SUM_OF_SAMPLES						41
	#define FPU_REG_ACCEL_Z_SUM_OF_SAMPLES						42
	#define FPU_REG_ACCEL_NUMBER_OF_SAMPLES						43
	#define FPU_REG_ACCEL_FILTERED_X 							44
	#define FPU_REG_ACCEL_FILTERED_Y 							45
	#define FPU_REG_ACCEL_FILTERED_Z 							46
	#define FPU_REG_ACCEL_ACTUAL_ONE_G							48
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM1_X 		58
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM2_X 		59
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM3_X 		60
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM4_X 		61
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM1_X 		62
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM2_X 		63
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM3_X 		64
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM4_X 		65
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM1_Y 		66
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM2_Y 		67
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM3_Y 		68
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM4_Y 		69
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM1_Y 		70
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM2_Y 		71
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM3_Y 		72
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM4_Y 		73
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM1_Z 		74
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM2_Z 		75
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM3_Z 		76
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_INPUT_TM4_Z 		77
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM1_Z 		78
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM2_Z 		79
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM3_Z 		80
	#define FPU_REG_LPF_CHEBY2_FOURTH_ORDER_OUTPUT_TM4_Z 		81

	// Gyroscope
	#define FPU_REG_GYRO_X_SUM_OF_SAMPLES						82
	#define FPU_REG_GYRO_Y_SUM_OF_SAMPLES						83
	#define FPU_REG_GYRO_Z_SUM_OF_SAMPLES						84
	#define FPU_REG_GYRO_NUMBER_OF_SAMPLES						85
	#define FPU_REG_GYRO_FILTERED_X								86
	#define FPU_REG_GYRO_FILTERED_Y								87
	#define FPU_REG_GYRO_FILTERED_Z								88
	#define FPU_REG_GYRO_HEADING								90
	#define FPU_REG_GYRO_X										91	
	#define FPU_REG_GYRO_Y										92	
	#define FPU_REG_GYRO_Z										93		

	// Kinematics
	#define FPU_REG_KINEMATICS_ANGLE_X							94
	#define FPU_REG_KINEMATICS_ANGLE_Y							95
	#define FPU_REG_KINEMATICS_ANGLE_Z							96
	#define FPU_REG_KINEMATICS_Q0								97
	#define FPU_REG_KINEMATICS_Q1								98
	#define FPU_REG_KINEMATICS_Q2								99
	#define FPU_REG_KINEMATICS_Q3								100
	#define FPU_REG_KINEMATICS_EXINT							101
	#define FPU_REG_KINEMATICS_EYINT							102
	#define FPU_REG_KINEMATICS_EZINT							103
	#define FPU_REG_KINEMATICS_PREVIOUS_EX						104
	#define FPU_REG_KINEMATICS_PREVIOUS_EY						105
	#define FPU_REG_KINEMATICS_PREVIOUS_EZ						106

	// Magnetometer
	#define FPU_REG_MAG_X										107
	#define FPU_REG_MAG_Y										108
	#define FPU_REG_MAG_Z										109
	#define FPU_REG_MAG_TILT_COMPENSATED_X						110		// used during offset calculations only, calc by hand, used to calculate mag_heading_x and mag_heading_y
	#define FPU_REG_MAG_TILT_COMPENSATED_Y						111		// used during offset calculations only, calc by hand, used to calculate mag_heading_x and mag_heading_y
	#define FPU_REG_MAG_MEASURED_X								114
	#define FPU_REG_MAG_MEASURED_Y								115
	#define FPU_REG_MAG_MEASURED_Z								116
	#define FPU_REG_MAG_HEADING_X								117		// absolute heading = atan2(hdgY, hdgX)
	#define FPU_REG_MAG_HEADING_Y								118		// if (heading < 0)
																		//    heading += radians(360)
	#define FPU_REG_MAG_KINEMATICS_LQ0							119
	#define FPU_REG_MAG_KINEMATICS_LQ1							120
	#define FPU_REG_MAG_KINEMATICS_LQ2							121
	#define FPU_REG_MAG_KINEMATICS_LQ3							122
	#define FPU_REG_MAG_KINEMATICS_LEXINT						123
	#define FPU_REG_MAG_KINEMATICS_LEYINT						124
	#define FPU_REG_MAG_KINEMATICS_LEZINT						125
	#define FPU_REG_MAG_TRUE_NORTH_HEADING						126
	// last register 127 is free
	
	// EEPROM data - slots range between 0 and 255 (each slot is 4 bytes)
	#define FPU_EEPROM_SLOT_ACCEL_OFFSET_X						255
	#define FPU_EEPROM_SLOT_ACCEL_OFFSET_Y						254
	#define FPU_EEPROM_SLOT_ACCEL_OFFSET_Z						253
	#define FPU_EEPROM_SLOT_ACCEL_SCALE_X						252
	#define FPU_EEPROM_SLOT_ACCEL_SCALE_Y						251
	#define FPU_EEPROM_SLOT_ACCEL_SCALE_Z						250
	#define FPU_EEPROM_SLOT_GYRO_OFFSET_X						249
	#define FPU_EEPROM_SLOT_GYRO_OFFSET_Y						248
	#define FPU_EEPROM_SLOT_GYRO_OFFSET_Z						247
	#define FPU_EEPROM_SLOT_GYRO_SCALE							246
	#define FPU_EEPROM_SLOT_MAG_SCALE_X							245
	#define FPU_EEPROM_SLOT_MAG_SCALE_Y							244
	#define FPU_EEPROM_SLOT_MAG_SCALE_Z							243
	#define FPU_EEPROM_SLOT_MAG_OFFSET_X						242
	#define FPU_EEPROM_SLOT_MAG_OFFSET_Y						241
	#define FPU_EEPROM_SLOT_MAG_OFFSET_Z						240
	#define FPU_EEPROM_SLOT_CONST_NEG_ONE_G						239
	#define FPU_EEPROM_REG_LPF_CHEBY2_B0 						238
	#define FPU_EEPROM_REG_LPF_CHEBY2_B1 						237
	#define FPU_EEPROM_REG_LPF_CHEBY2_B2 						236
	#define FPU_EEPROM_REG_LPF_CHEBY2_B3 						235
	#define FPU_EEPROM_REG_LPF_CHEBY2_B4 						234
	#define FPU_EEPROM_REG_LPF_CHEBY2_A1 						233
	#define FPU_EEPROM_REG_LPF_CHEBY2_A2 						232
	#define FPU_EEPROM_REG_LPF_CHEBY2_A3 						231
	#define FPU_EEPROM_REG_LPF_CHEBY2_A4 						230
	// slots 92 to 229 are free
	
	/* ----------------------------------------------------------------------------
	   FPU functions (0 to 63)
	   ---------------------------------------------------------------------------- */
	
	#define FPU_FUNC_ACCEL_SUM									0
	#define FPU_FUNC_ACCEL_EVELUATE_MS2							1
	#define FPU_FUNC_ACCEL_COMPUTE_FOURTH_ORDER					2
	#define FPU_FUNC_GYRO_EVALUATE_RATE							3
	#define FPU_FUNC_GYRO_SUM									4
	#define FPU_FUNC_KINEMATICS_ARG_UPDATE						5
	#define FPU_FUNC_KINEMATICS_EULER_ANGLES					6
	#define FPU_FUNC_MAG_MEASURE								7
	#define FPU_FUNC_MAG_CALCULATE_HEADING						8
	#define FPU_FUNC_MAG_CALCULATE_POSITIVE_HEADING				9
	#define FPU_FUNC_MAG_CALCULATE_ABSOLUTE_HEADING				10
	
	// EEPROM Functions - slots range between 0 and 255
	#define FPU_EEPROM_FUNC_ACCEL_INIT_FOURTH_ORDER				0		// EEPROM slot 0 to 25		26 slots needed
	#define FPU_EEPROM_FUNC_MAG_INITIALIZE_HEADING_FUSION		26		// EEPROM slots 26 to 71	46 slots needed
	#define FPU_EEPROM_FUNC_GYRO_RESET_SUM						72		// EEPROM slot 72 to 74		3 slots needed
	#define FPU_EEPROM_FUNC_ACCEL_RESET_SUM						75		// EEPROM slot 75 to 77		3 slots needed
	#define FPU_EEPROM_FUNC_ACCEL_COMPUTE_BIAS					78		// EEPROM slot 78 to 91		14 slots needed
	// next slot number available is 92
	// slots 92 to 229 are free
	
/* ================================================================================================
   um-FPU v3.1 Defines
   ================================================================================================ */

	// General
	#define	SYNC_CHAR	0x5C		// sync character

	// uM-FPU V3.1 opcode definitions
	#define	NOP			0x00	// No Operation
	#define	SELECTA		0x01	// Select register A
	#define	SELECTX		0x02	// Select register X
	#define	CLR			0x03	// reg[nn] = 0
	#define	CLRA		0x04	// reg[A] = 0
	#define	CLRX		0x05	// reg[X] = 0, X = X + 1
	#define	CLR0		0x06	// reg[0] = 0
	#define	COPY		0x07	// reg[nn] = reg[mm]
	#define	COPYA		0x08	// reg[nn] = reg[A]
	#define	COPYX		0x09	// reg[nn] = reg[X], X = X + 1
	#define	LOAD		0x0A	// reg[0] = reg[nn]
	#define	LOADA		0x0B	// reg[0] = reg[A]
	#define	LOADX		0x0C	// reg[0] = reg[X], X = X + 1
	#define	ALOADX		0x0D	// reg[A] = reg[X], X = X + 1
	#define	XSAVE		0x0E	// reg[X] = reg[nn], X = X + 1
	#define	XSAVEA		0x0F	// reg[X] = reg[A], X = X + 1
	#define	COPY0		0x10	// reg[nn] = reg[0]
	#define	COPYI		0x11	// reg[nn] = long(unsigned bb)
	#define	SWAP		0x12	// Swap reg[nn] and reg[mm]
	#define	SWAPA		0x13	// Swap reg[A] and reg[nn]
	#define	LEFT		0x14	// Left parenthesis
	#define	RIGHT		0x15	// Right parenthesis
	#define	FWRITE		0x16	// Write 32-bit float to reg[nn]
	#define	FWRITEA		0x17	// Write 32-bit float to reg[A]
	#define	FWRITEX		0x18	// Write 32-bit float to reg[X], X = X + 1
	#define	FWRITE0		0x19	// Write 32-bit float to reg[0]
	#define	FREAD		0x1A	// Read 32-bit float from reg[nn]
	#define	FREADA		0x1B	// Read 32-bit float from reg[A]
	#define	FREADX		0x1C	// Read 32-bit float from reg[X], X = X + 1
	#define	FREAD0		0x1D	// Read 32-bit float from reg[0]
	#define	ATOF		0x1E	// Convert ASCII to float, store in reg[0]
	#define	FTOA		0x1F	// Convert float to ASCII
	#define	FSET		0x20	// reg[A] = reg[nn]
	#define	FADD		0x21	// reg[A] = reg[A] + reg[nn]
	#define	FSUB		0x22	// reg[A] = reg[A] - reg[nn]
	#define	FSUBR		0x23	// reg[A] = reg[nn] - reg[A]
	#define	FMUL		0x24	// reg[A] = reg[A] * reg[nn]
	#define	FDIV		0x25	// reg[A] = reg[A] / reg[nn]
	#define	FDIVR		0x26	// reg[A] = reg[nn] / reg[A]
	#define	FPOW		0x27	// reg[A] = reg[A] ** reg[nn]
	#define	FCMP		0x28	// Float compare reg[A] - reg[nn]
	#define	FSET0		0x29	// reg[A] = reg[0]
	#define	FADD0		0x2A	// reg[A] = reg[A] + reg[0]
	#define	FSUB0		0x2B	// reg[A] = reg[A] - reg[0]
	#define	FSUBR0		0x2C	// reg[A] = reg[0] - reg[A]
	#define	FMUL0		0x2D	// reg[A] = reg[A] * reg[0]
	#define	FDIV0		0x2E	// reg[A] = reg[A] / reg[0]
	#define	FDIVR0		0x2F	// reg[A] = reg[0] / reg[A]
	#define	FPOW0		0x30	// reg[A] = reg[A] ** reg[0]
	#define	FCMP0		0x31	// Float compare reg[A] - reg[0]
	#define	FSETI		0x32	// reg[A] = float(bb)
	#define	FADDI		0x33	// reg[A] = reg[A] + float(bb)
	#define	FSUBI		0x34	// reg[A] = reg[A] - float(bb)
	#define	FSUBRI		0x35	// reg[A] = float(bb) - reg[A]
	#define	FMULI		0x36	// reg[A] = reg[A] * float(bb)
	#define	FDIVI		0x37	// reg[A] = reg[A] / float(bb)
	#define	FDIVRI		0x38	// reg[A] = float(bb) / reg[A]
	#define	FPOWI		0x39	// reg[A] = reg[A] ** bb
	#define	FCMPI		0x3A	// Float compare reg[A] - float(bb)
	#define	FSTATUS		0x3B	// Float status of reg[nn]
	#define	FSTATUSA	0x3C	// Float status of reg[A]
	#define	FCMP2		0x3D	// Float compare reg[nn] - reg[mm]
	#define	FNEG		0x3E	// reg[A] = -reg[A]
	#define	FABS		0x3F	// reg[A] = | reg[A] |
	#define	FINV		0x40	// reg[A] = 1 / reg[A]
	#define	SQRT		0x41	// reg[A] = sqrt(reg[A])
	#define	ROOT		0x42	// reg[A] = root(reg[A], reg[nn])
	#define	LOG			0x43	// reg[A] = log(reg[A])
	#define	LOG10		0x44	// reg[A] = log10(reg[A])
	#define	EXP			0x45	// reg[A] = exp(reg[A])
	#define	EXP10		0x46	// reg[A] = exp10(reg[A])
	#define	SIN			0x47	// reg[A] = sin(reg[A])
	#define	COS			0x48	// reg[A] = cos(reg[A])
	#define	TAN			0x49	// reg[A] = tan(reg[A])
	#define	ASIN		0x4A	// reg[A] = asin(reg[A])
	#define	ACOS		0x4B	// reg[A] = acos(reg[A])
	#define	ATAN		0x4C	// reg[A] = atan(reg[A])
	#define	ATAN2		0x4D	// reg[A] = atan2(reg[A], reg[nn])
	#define	DEGREES		0x4E	// reg[A] = degrees(reg[A])
	#define	RADIANS		0x4F	// reg[A] = radians(reg[A])
	#define	FMOD		0x50	// reg[A] = reg[A] MOD reg[nn]
	#define	FLOOR		0x51	// reg[A] = floor(reg[A])
	#define	CEIL		0x52	// reg[A] = ceil(reg[A])
	#define	ROUND		0x53	// reg[A] = round(reg[A])
	#define	FMIN		0x54	// reg[A] = min(reg[A], reg[nn])
	#define	FMAX		0x55	// reg[A] = max(reg[A], reg[nn])
	#define	FCNV		0x56	// reg[A] = conversion(nn, reg[A])
	#define	FMAC		0x57	// reg[A] = reg[A] + (reg[nn] * reg[mm])
	#define	FMSC		0x58	// reg[A] = reg[A] - (reg[nn] * reg[mm])
	#define	LOADBYTE	0x59	// reg[0] = float(signed bb)
	#define	LOADUBYTE	0x5A	// reg[0] = float(unsigned byte)
	#define	LOADWORD	0x5B	// reg[0] = float(signed word)
	#define	LOADUWORD	0x5C	// reg[0] = float(unsigned word)
	#define	LOADE		0x5D	// reg[0] = 2.7182818
	#define	LOADPI		0x5E	// reg[0] = 3.1415927
	#define	LOADCON		0x5F	// reg[0] = float constant(nn)
	#define	F_FLOAT		0x60	// reg[A] = float(reg[A])
	#define	F_FIX		0x61	// reg[A] = fix(reg[A])
	#define	FIXR		0x62	// reg[A] = fix(round(reg[A]))
	#define	FRAC		0x63	// reg[A] = fraction(reg[A])
	#define	FSPLIT		0x64	// reg[A] = int(reg[A]), reg[0] = frac(reg[A])
	#define	SELECTMA	0x65	// Select matrix A
	#define	SELECTMB	0x66	// Select matrix B
	#define	SELECTMC	0x67	// Select matrix C
	#define	LOADMA		0x68	// reg[0] = matrix A[bb,bb]
	#define	LOADMB		0x69	// reg[0] = matrix B[bb, bb]
	#define	LOADMC		0x6A	// reg[0] = matrix C[bb, bb]
	#define	SAVEMA		0x6B	// matrix A[bb,bb] = reg[A]
	#define	SAVEMB		0x6C	// matrix B[bb,bb] = reg[A]
	#define	SAVEMC		0x6D	// matrix C[bb,bb] = reg[A]
	#define	MOP			0x6E	// Matrix operation
	#define	FFT			0x6F	// FFT operation
	#define	WRBLK		0x70	// write register block
	#define	RDBLK		0x71	// read register block
	#define	LOADIND		0x7A	// reg[0] = reg[reg[nn]]
	#define	SAVEIND		0x7B	// reg[reg[nn]] = reg[A]
	#define	INDA		0x7C	// Select A using reg[nn]
	#define	INDX		0x7D	// Select X using reg[nn]
	#define	FCALL		0x7E	// Call function in Flash memory
	#define	EECALL		0x7F	// Call function in EEPROM memory
	#define	RET			0x80	// Return from function
	#define	BRA			0x81	// Unconditional branch
	#define	BRACC		0x82	// Conditional branch
	#define	JMP			0x83	// Unconditional jump
	#define	JMPCC		0x84	// Conditional jump
	#define	TABLE		0x85	// Table lookup
	#define	FTABLE		0x86	// Floating point reverse table lookup
	#define	LTABLE		0x87	// Long integer reverse table lookup
	#define	POLY		0x88	// reg[A] = nth order polynomial
	#define	F_GOTO		0x89	// Computed goto
	#define	RETCC		0x8A	// Conditional return from function
	#define	LWRITE		0x90	// Write 32-bit long integer to reg[nn]
	#define	LWRITEA		0x91	// Write 32-bit long integer to reg[A]
	#define	LWRITEX		0x92	// Write 32-bit long integer to reg[X], X = X + 1
	#define	LWRITE0		0x93	// Write 32-bit long integer to reg[0]
	#define	LREAD		0x94	// Read 32-bit long integer from reg[nn]
	#define	LREADA		0x95	// Read 32-bit long integer from reg[A]
	#define	LREADX		0x96	// Read 32-bit long integer from reg[X], X = X + 1
	#define	LREAD0		0x97	// Read 32-bit long integer from reg[0]
	#define	LREADBYTE	0x98	// Read lower 8 bits of reg[A]
	#define	LREADWORD	0x99	// Read lower 16 bits reg[A]
	#define	ATOL		0x9A	// Convert ASCII to long integer
	#define	LTOA		0x9B	// Convert long integer to ASCII
	#define	LSET		0x9C	// reg[A] = reg[nn]
	#define	LADD		0x9D	// reg[A] = reg[A] + reg[nn]
	#define	LSUB		0x9E	// reg[A] = reg[A] - reg[nn]
	#define	LMUL		0x9F	// reg[A] = reg[A] * reg[nn]
	#define	LDIV		0xA0	// reg[A] = reg[A] / reg[nn]
	#define	LCMP		0xA1	// Signed long compare reg[A] - reg[nn]
	#define	LUDIV		0xA2	// reg[A] = reg[A] / reg[nn]
	#define	LUCMP		0xA3	// Unsigned long compare of reg[A] - reg[nn]
	#define	LTST		0xA4	// Long integer status of reg[A] AND reg[nn]
	#define	LSET0		0xA5	// reg[A] = reg[0]
	#define	LADD0		0xA6	// reg[A] = reg[A] + reg[0]
	#define	LSUB0		0xA7	// reg[A] = reg[A] - reg[0]
	#define	LMUL0		0xA8	// reg[A] = reg[A] * reg[0]
	#define	LDIV0		0xA9	// reg[A] = reg[A] / reg[0]
	#define	LCMP0		0xAA	// Signed long compare reg[A] - reg[0]
	#define	LUDIV0		0xAB	// reg[A] = reg[A] / reg[0]
	#define	LUCMP0		0xAC	// Unsigned long compare reg[A] - reg[0]
	#define	LTST0		0xAD	// Long integer status of reg[A] AND reg[0]
	#define	LSETI		0xAE	// reg[A] = long(bb)
	#define	LADDI		0xAF	// reg[A] = reg[A] + long(bb)
	#define	LSUBI		0xB0	// reg[A] = reg[A] - long(bb)
	#define	LMULI		0xB1	// reg[A] = reg[A] * long(bb)
	#define	LDIVI		0xB2	// reg[A] = reg[A] / long(bb)
	#define	LCMPI		0xB3	// Signed long compare reg[A] - long(bb)
	#define	LUDIVI		0xB4	// reg[A] = reg[A] / unsigned long(bb)
	#define	LUCMPI		0xB5	// Unsigned long compare reg[A] - ulong(bb)
	#define	LTSTI		0xB6	// Long integer status of reg[A] AND ulong(bb)
	#define	LSTATUS		0xB7	// Long integer status of reg[nn]
	#define	LSTATUSA	0xB8	// Long integer status of reg[A]
	#define	LCMP2		0xB9	// Signed long compare reg[nn] - reg[mm]
	#define	LUCMP2		0xBA	// Unsigned long compare reg[nn] - reg[mm]
	#define	LNEG		0xBB	// reg[A] = -reg[A]
	#define	LABS		0xBC	// reg[A] = | reg[A] |
	#define	LINC		0xBD	// reg[nn] = reg[nn] + 1
	#define	LDEC		0xBE	// reg[nn] = reg[nn] - 1
	#define	LNOT		0xBF	// reg[A] = NOT reg[A]
	#define	LAND		0xC0	// reg[A] = reg[A] AND reg[nn]
	#define	LOR			0xC1	// reg[A] = reg[A] OR reg[nn]
	#define	LXOR		0xC2	// reg[A] = reg[A] XOR reg[nn]
	#define	LSHIFT		0xC3	// reg[A] = reg[A] shift reg[nn]
	#define	LMIN		0xC4	// reg[A] = min(reg[A], reg[nn])
	#define	LMAX		0xC5	// reg[A] = max(reg[A], reg[nn])
	#define	LONGBYTE	0xC6	// reg[0] = long(signed byte bb)
	#define	LONGUBYTE	0xC7	// reg[0] = long(unsigned byte bb)
	#define	LONGWORD	0xC8	// reg[0] = long(signed word wwww)
	#define	LONGUWORD	0xC9	// reg[0] = long(unsigned word wwww)
	#define	SETSTATUS	0xCD	// Set status byte
	#define	SEROUT		0xCE	// Serial output
	#define	SERIN		0xCF	// Serial Input
	#define	SETOUT		0xD0	// Set OUT1 and OUT2 output pins
	#define	ADCMODE		0xD1	// Set A/D trigger mode
	#define	ADCTRIG		0xD2	// A/D manual trigger
	#define	ADCSCALE	0xD3	// ADCscale[ch] = B
	#define	ADCLONG		0xD4	// reg[0] = ADCvalue[ch]
	#define	ADCLOAD		0xD5	// reg[0] = float(ADCvalue[ch]) * ADCscale[ch]
	#define	ADCWAIT		0xD6	// wait for next A/D sample
	#define	TIMESET		0xD7	// time = reg[0]
	#define	TIMELONG	0xD8	// reg[0] = time (long)
	#define	TICKLONG	0xD9	// reg[0] = ticks (long)
	#define	EESAVE		0xDA	// EEPROM[nn] = reg[mm]
	#define	EESAVEA		0xDB	// EEPROM[nn] = reg[A]
	#define	EELOAD		0xDC	// reg[nn] = EEPROM[mm]
	#define	EELOADA		0xDD	// reg[A] = EEPROM[nn]
	#define	EEWRITE		0xDE	// Store bytes in EEPROM
	#define	EXTSET		0xE0	// external input count = reg[0]
	#define	EXTLONG		0xE1	// reg[0] = external input counter (long)
	#define	EXTWAIT		0xE2	// wait for next external input
	#define	STRSET		0xE3	// Copy string to string buffer
	#define	STRSEL		0xE4	// Set selection point
	#define	STRINS		0xE5	// Insert string at selection point
	#define	STRCMP		0xE6	// Compare string with string buffer
	#define	STRFIND		0xE7	// Find string and set selection point
	#define	STRFCHR		0xE8	// Set field separators
	#define	STRFIELD	0xE9	// Find field and set selection point
	#define	STRTOF		0xEA	// Convert string selection to float
	#define	STRTOL		0xEB	// Convert string selection to long
	#define	READSEL		0xEC	// Read string selection
	#define	STRBYTE		0xED	// Insert 8-bit byte at selection point
	#define	STRINC		0xEE	// increment selection point
	#define	STRDEC		0xEF	// decrement selection point
	#define	SYNC		0xF0	// Get synchronization byte
	#define	READSTATUS	0xF1	// Read status byte
	#define	READSTR		0xF2	// Read string from string buffer
	#define	VERSION		0xF3	// Copy version string to string buffer
	#define	IEEEMODE	0xF4	// Set IEEE mode (default)
	#define	PICMODE		0xF5	// Set PIC mode
	#define	CHECKSUM	0xF6	// Calculate checksum for uM-FPU code
	#define	F_BREAK		0xF7	// Debug breakpoint
	#define	TRACEOFF	0xF8	// Turn debug trace off
	#define	TRACEON		0xF9	// Turn debug trace on
	#define	TRACESTR	0xFA	// Send string to debug trace buffer
	#define	TRACEREG	0xFB	// Send register value to trace buffer
	#define	READVAR		0xFC	// Read internal variable, store in reg[0]
	#define	RESET		0xFF	// Reset after 9 consecutive 0xFF bytes

	// uM-FPU V3.1 MOP definitions
	#define	SCALAR_SET		0	// MA[r,c] = reg[0], for all elements
	#define	SCALAR_ADD		1	// MA[r,c] = MA[r,c] + reg[0], for all elements
	#define	SCALAR_SUB		2	// MA[r,c] = reg[0] - MA[r,c], for all elements
	#define	SCALAR_SUBR		3	// MA[r,c] = MA[r,c] + reg[0], for all elements
	#define	SCALAR_MUL		4	// MA[r,c] = MA[r,c] * reg[0], for all elements
	#define	SCALAR_DIV		5	// MA[r,c] = MA[r,c] / reg[0], for all elements
	#define	SCALAR_DIVR		6	// MA[r,c] = reg[0] / MA[r,c], for all elements
	#define	SCALAR_POW		7	// MA[r,c] = MA[r,c] ** reg[0], for all elements
	#define	EWISE_SET		8	// MA[r,c] = MB[r,c], for all elements
	#define	EWISE_ADD		9	// MA[r,c] = MA[r,c] + MB[r,c], for all elements
	#define	EWISE_SUB		10	// MA[r,c] = MB[r,c] - MA[r,c], for all elements
	#define	EWISE_SUBR		11	// MA[r,c] = MA[r,c] + MB[r,c], for all elements
	#define	EWISE_MUL		12	// MA[r,c] = MA[r,c] * MB[r,c], for all elements
	#define	EWISE_DIV		13	// MA[r,c] = MA[r,c] / MB[r,c], for all elements
	#define	EWISE_DIVR		14	// MA[r,c] = MB[r,c] / MA[r,c], for all elements
	#define	EWISE_POW		15	// MA[r,c] = MA[r,c] ** MB[r,c], for all elements
	#define	MX_MULTIPLY		16	// MA = MB * MC matrix multiply
	#define	MX_IDENTITY		17	// MA = identity matrix
	#define	MX_DIAGONAL		18	// MA = diagonal matrix
	#define	MX_TRANSPOSE	19	// MA = transpose of MB
	#define	MX_COUNT		20	// reg[0] = number of elements in MA
	#define	MX_SUM			21	// reg[0] = sum of all elements in MA
	#define	MX_AVE			22	// reg[0] = average value of all elements in MA
	#define	MX_MIN			23	// reg[0] = minimum value of all elements in MA
	#define	MX_MAX			24	// reg[0] = maximum value of all elements in MA
	#define	MX_COPYAB		25	// copy MA to MC
	#define	MX_COPYAC		26	// copy MA to MC
	#define	MX_COPYBA		27	// copy MB to MA
	#define	MX_COPYBC		28	// copy MB to MC
	#define	MX_COPYCA		29	// copy MC to MA
	#define	MX_COPYCB		30	// copy MC to MB
	#define	MX_DETERM		31	// reg0 = determinant of MA (2x2 and 3x3)
	#define	MX_INVERSE		32	// MA = inverse of MB (2x2 and 3x3)
	#define	MX_ILOADRA		33	// indexed load registers to MA
	#define	MX_ILOADRB		34	// indexed load registers to MB
	#define	MX_ILOADRC		35	// indexed load registers to MC
	#define	MX_ILOADBA		36	// indexed load MB to MA
	#define	MX_ILOADCA		37	// indexed load MC to MA
	#define	MX_ISAVEAR		38	// indexed save MA to registers
	#define	MX_ISAVEAB		39	// indexed save MA to MB
	#define	MX_ISAVEAC		40	// indexed save MA to MC

	// uM-FPU V3.1 FFT definitions
	#define	FIRST_STAGE		0	// first stage of FFT
	#define	NEXT_STAGE		1	// next stage of FFT
	#define	NEXT_LEVEL		2	// next level of FFT
	#define	NEXT_BLOCK		3	// next block of FFT
	#define	BIT_REVERSE		4	// bit reverse sort (pre-processing)
	#define	PRE_ADJUST		8	// inverse FFT (pre-processing)
	#define	POST_ADJUST		16	// inverse FFT (post-processing)

	// FPU status bits
	#define STATUS_SUCCESS		0x80
	#define STATUS_IS_ZERO		0x81	// positive zero
	#define STATUS_IS_NEG		0x82	// negative
	#define STATUS_IS_NEG_ZERO	0x83	// negative zero 
	#define STATUS_IS_NAN		0x84	// NaN (Not-a-Number) status bit
	#define STATUS_IS_PINF		0x88	// positive infinity
	#define STATUS_IS_NINF		0x8A	// negative infinity
	
	// FPU timing delays
	#define FPU_READ_DELAY		15		// 15 us
	#define FPU_WAIT_DELAY		1		// 1 us
	
/* ================================================================================================
   Function Prototypes
   ================================================================================================ */
	
	/* Enable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	extern "C" {
	#endif

	// Application initialization
	int fpu_load_constants(void);
	void fpu_load_eeprom_data(void);
	
	// FPU initialization and reset
	int fpu_initialize(void);
	int fpu_sync(void);
	int fpu_reset(void);

	// Read (SPI)
	void 		fpu_read_delay(void);
	void 		fpu_wait(void);
	uint8_t 	fpu_read(void);
	int 		fpu_read_word(void);
	long 		fpu_read_long(void);
	double 		fpu_read_double(void);
	double 		fpu_read_double_from_register(uint8_t register_num);
	uint8_t 	fpu_read_byte(void);
	uint16_t 	fpu_read_int(void);
	
	// Write (SPI)
	void fpu_write(uint8_t value);
	void fpu_write2(uint8_t value, uint8_t value2);
	void fpu_write3(uint8_t value, uint8_t value2, uint8_t value3);
	void fpu_write4(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4);
	void fpu_write5(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5);
	void fpu_write6(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5, uint8_t value6);
	void fpu_write7(uint8_t value, uint8_t value2, uint8_t value3, uint8_t value4, uint8_t value5, uint8_t value6, uint8_t value7);
	
	void fpu_write_int(int value);
	void fpu_write_long(long value);
	void fpu_write_double(double value);
	void fpu_write_int_to_reg_nn(int value, uint8_t regnn);
	void fpu_write_double_to_reg_nn(double value, uint8_t regnn);
	void fpu_write_uint8_to_reg_nn(uint8_t value, uint8_t regnn);
	void fpu_eeprom_write_double_to_slot(double value, uint8_t slotNumber);
	
	// General functions
	uint8_t 	fpu_read_status(void);
	
	// String functions
	int 		fpu_read_version(char *pVersion);
	void 		fpu_get_string(char *pVersion);
	void 		fpu_ftoa(uint8_t format, double value, char *pStringValue);
	void 		fpu_ftoa_reg_a(uint8_t format, char *pStringValue);
	void 		fpu_ltoa_reg_a(uint8_t format, char *pStringValue);

	// FPU functions
	void fpu_function_call(uint8_t functionNumber);
	void fpu_eeprom_function_call(uint8_t functionNumber);


	//double fpu_read_dbl(void);
	
	void fpu_init_eeprom_defaults(void);

	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
	}
	#endif

#endif

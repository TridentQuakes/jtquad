/* ================================================================================================
   JTQuad
   
   Author: Jerry Tostao   
   ================================================================================================ */
   
#ifndef __NUMERIC_TYPEDEFS_H__
#define __NUMERIC_TYPEDEFS_H__

/* ================================================================================================
   Typedefs
   ================================================================================================ */
   
typedef uint8_t   	byte;           // 8-bit
typedef uint16_t    word;           // 16-bit
typedef uint32_t   	dword;          // 32-bit

/* ----------------------------------------------------------------------------
   Byte with individual bits addressable
   ---------------------------------------------------------------------------- */
typedef union _BYTE
{	
	byte _byte;
	
	// The ':' construct specifies the length in bits for each field
    struct
    {   unsigned b0:1;
        unsigned b1:1;
        unsigned b2:1;
        unsigned b3:1;
        unsigned b4:1;
        unsigned b5:1;
        unsigned b6:1;
        unsigned b7:1;
    };
	
} BYTE;

/* ----------------------------------------------------------------------------
   WORD (2 bytes) with individual bytes and bits addressable
   ---------------------------------------------------------------------------- */
typedef union _WORD
{	
	int	_int;
	
	uint16_t _uint16_t;
	
	struct
    {   
		uint8_t _uint8_t_0;
        uint8_t _uint8_t_1;
    };
	
	word _word;
	
    struct
    {   
		byte byte0;
        byte byte1;
    };
    
    struct
    {   
		BYTE Byte0;
        BYTE Byte1;
    };
    
    struct
    {   
		BYTE LowB;
        BYTE HighB;
    };
    
    struct
    {   
		byte v[2];
    };
	
} WORD;

/* ----------------------------------------------------------------------------
   DWORD (4 bytes) with individual bytes and bits addressable
   ---------------------------------------------------------------------------- */
typedef union _DWORD
{	
	dword _dword;
	
	double _double;
	
	long _long;
	
	uint32_t _uint32_t;
	
	struct
    {   
		uint8_t _uint8_t_0;
        uint8_t _uint8_t_1;
		uint8_t _uint8_t_2;
		uint8_t _uint8_t_3;
    };
	
    struct
    {   
		byte byte0;
        byte byte1;
        byte byte2;
        byte byte3;
    };
    
    struct
    {   
		word word0;
        word word1;
    };
    
    struct
    {   
		BYTE Byte0;
        BYTE Byte1;
        BYTE Byte2;
        BYTE Byte3;
    };
    
    struct
    {   
		WORD Word0;
        WORD Word1;
    };
    
    struct
    {   
		byte v[4];
    };

} DWORD;

/* ================================================================================================
   Macros
   ================================================================================================ */
   
	// Macros for addressing individual bytes from arrays of char
	#define LSB(a)      	((a).v[0])
	#define MSB(a)      	((a).v[1])
	#define LOWER_LSB(a)    ((a).v[0])
	#define LOWER_MSB(a)    ((a).v[1])
	#define UPPER_LSB(a)    ((a).v[2])
	#define UPPER_MSB(a)    ((a).v[3])

#endif

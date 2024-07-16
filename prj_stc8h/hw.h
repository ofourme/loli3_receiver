/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   HW.H-V1.0.0 (2023.Oct.25th)
*
********************************************************************************
*/

#ifndef __HW_H__
#define __HW_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#define STC_15W408AS		0
#define STC_15W408AS_SOP20	0
#define ARDUINO_ATMEAG328P	0
#define STC_8H				1
#define STC_8H_TSSOP20		1

#define FOSC	12000000UL

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/*////////////////////////////////////////////////////////////////////////////*/
#if STC_15W408AS
/*////////////////////////////////////////////////////////////////////////////*/

#include "stc15w.h"
#include<intrins.h>

#define u8 unsigned char
#define u16 unsigned int
#define s16 signed int
#define u32 unsigned long
#define word_t u8
#define uint8_t u8
#define bool bit
	
/*////////////////////////////////////////////////////////////////////////////*/
#elif STC_8H
/*////////////////////////////////////////////////////////////////////////////*/

#include "STC8H.h"
#include<intrins.h>

#define u8 unsigned char
#define u16 unsigned int
#define s16 signed int
#define u32 unsigned long
#define s32 signed long
#define word_t u8
#define uint8_t u8
#define bool bit

/*////////////////////////////////////////////////////////////////////////////*/
#elif ARDUINO_ATMEAG328P
/*////////////////////////////////////////////////////////////////////////////*/

#include <arduino.h>

#define u8 unsigned char
#define u16 unsigned int
#define s16 signed int
#define u32 unsigned long
#define word_t u8
#define uint8_t u8
#define idata
#define xdata
#define code

/*////////////////////////////////////////////////////////////////////////////*/

#else
	#error "board not defined!"
#endif

/*////////////////////////////////////////////////////////////////////////////*/

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif

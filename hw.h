/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2008-2021, 微风山谷/ofourme@163.com
*           License: LGPL
*
*   HW.H-V1.0.0 (2021.Dec.5th)
*
********************************************************************************
*/

#ifndef __HW_H__
#define __HW_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include<reg52.h>
#include<intrins.h>

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long
	
#define boot bit

//15系列新增寄存器
sfr IAP_DATA=0xc2;
sfr IAP_ADDRH=0xc3;
sfr IAP_ADDRL=0xc4;
sfr IAP_CMD=0xc5;
sfr IAP_TRIG=0xc6;
sfr IAP_CONTR=0xc7;

sfr AUXR=0x8e;
sfr T2H=0xd6;
sfr T2L=0xd7;
sfr IE2=0xaf;
sfr P1ASF=0x9d;
sfr ADC_CONTR=0xbc;
sfr ADC_RES=0xbd;
sfr ADC_RESL=0xbe;

sfr P5=0xc8;
sfr P0M0=0x94;
sfr P1M0=0x92;
sfr P2M0=0x96;
sfr P3M0=0xb2;
sfr P4M0=0xb4;
sfr P1M1=0x91;

sfr P_SW1 = 0xA2;             //外设功能切换寄存器1
sfr CMOD=0xd9;
sfr CCON=0xd8;
sfr CCAPM0=0xda;
sfr CCAPM1=0xdb;
sfr CCAPM2=0xdc;

sfr CCAP0L=0xea;
sbit CCF0 = CCON^0;

sfr CCAP0H=0xfa;
sfr CCAP1H=0xfb;
sfr CCAP2H=0xfc;
sfr CL=0xe9;
sfr CH=0xf9;
sfr PCA_PWM0=0xf2;
sfr PCA_PWM1=0xf3;
sfr PCA_PWM2=0xf4;

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif
/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2008-2021, 微风山谷/ofourme@163.com
*           License: LGPL
*
*   CONFIG.H-V1.0.0 (2021.Dec.5th)
*
********************************************************************************
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/
//引脚定义

//定义指示灯
sbit LED=P1^0;
#define LED_ON()	do{LED = 1;}while(0)
#define LED_OFF()	do{LED = 0;}while(0)

//定义8个通道输出
sbit CH1=P3^7;
sbit CH2=P3^6;
sbit CH3=P3^5;
sbit CH4=P3^4;
sbit CH5=P3^3;
sbit CH6=P3^2;
sbit CH7=P3^1;
sbit CH8=P3^0;

//定义无线模块的管脚
sbit CE  =P5^5;
sbit SCK =P5^4;
sbit MISO=P1^7;
sbit IRQ =P1^6;
sbit MOSI=P1^5;
sbit CSN =P1^4;


/*////////////////////////////////////////////////////////////////////////////*/

#define noRF			IRQ
#define PIN_SBUS	CH7

// enable SBUS_DEBUG 	would disable SBUS pin output
//#define SBUS_DEBUG

// change the SBUS baud rate for debug. default:100K
//#define SBUS_BPS_9600
//#define SBUS_BPS_115200


/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif

/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   CONFIG.H-V1.1.0 (2023.Oct.25th)
*
********************************************************************************
*/

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"
#include "loli3_pact.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

// enable SBUS_DEBUG 	would disable SBUS pin output
//#define SBUS_DEBUG
//----------------------------------------------------------------------------//
// Set the SBUS baud rate. UART always be SERIAL_8E2(8位数据，偶校验，2位停止位)
// Optional: SBUS_BPS_9600, SBUS_BPS_115200, SBUS_BPS_100K; Default: SBUS_BPS_100K.
//#define SBUS_BPS_115200
#define SBUS_BPS_100K
//----------------------------------------------------------------------------//
// 轮询任务定时器步进精度，默认3ms
#define LOLI3_RECV_TIMERS_TICK_SETPS 3
//----------------------------------------------------------------------------//
// LOLI3_RECV_PWM_MINNUM: LOLI3_RECV_PWM_MINNUM_DEFAULT would compatibility with loli3 original.
//												0 would set the minimum if possible.	摇杆虚位大可能导致无法输出0.
#define LOLI3_RECV_PWM_MINNUM 	LOLI3_RECV_PWM_MINNUM_DEFAULT
// #define LOLI3_RECV_PWM_MINNUM 0
//----------------------------------------------------------------------------//
// 动力电池电阻分压放大倍数，正常是3。
#define VOLTAGE_BAT_MULTIPLE 3
//----------------------------------------------------------------------------//
// 芯片电压调整系数，微调此系数以匹配各个芯片。
/*
	BGV: 内部参考信号源电压ADC值；BGV5：5V时ADC值；BGVx：VCC参考电压时转换的ADC值
	STC15W:	内部参考信号源电压固定但不确定，固定电压Va=BGV5/1024*5=BGVx/1024*VCC
					--> VCC=BGV5/1024*5*1024/BGVx=BGV5*5/BGVx
	STC8H:	内部参考信号源电压固定(1.19v)， 1.19v=BGV5/1024*5=BGVx/1024*VCC
					--> VCC=BGV5/1024*5*1024/BGVx=BGV5*5/BGVx
					其中BGV5=1.19*1024/5+0.5~=244
*/
#if STC_15W408AS
	#define BGV5  260
#elif STC_8H
	#define BGV5	((unsigned long)(1.19*1024/5+0.5))
#else
	#err "BGV5 not defined."
#endif
	// 此处5与5V电压匹配，传输电压数据需扩大100倍以保留小数点后2位，而输入adc值滤波后扩大8倍，为统一单位故*5*100*8
#define VOLTAGE_IC_ADJUST	((5ul*100*8)*BGV5)
//----------------------------------------------------------------------------//

#ifdef SBUS_DEBUG
	#define DBG(str) hw_uart_puts(str)
#else
	#define DBG(str)
#endif

/*////////////////////////////////////////////////////////////////////////////*/
#if STC_15W408AS
/*////////////////////////////////////////////////////////////////////////////*/

#if STC_15W408AS_SOP20
//定义指示灯
sbit LED=P1^0;
//定义8个通道输出
sbit CH1=P3^7;
sbit CH2=P3^6;
sbit CH3=P3^5;
sbit CH4=P3^4;
sbit CH5=P3^3;
sbit CH6=P3^2;
sbit CH7=P3^1;
sbit CH8=P3^0;

sbit CH9=P5^4;
sbit CH10=P5^5;

//定义无线模块的管脚
sbit CE  =P1^2;
sbit SCK =P1^4;
sbit MISO=P1^6;
sbit IRQ =P1^7;
sbit MOSI=P1^5;
sbit CSN =P1^3;

#else
//定义指示灯
sbit LED=P1^2;
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

#endif

/*////////////////////////////////////*/
// 根据硬件实际情况条件编译
// PIN_CH5/PIN_CH_OUT用于输出，PIN_CH6/PIN_CH_IN用于输入。接收机启动时如果检测到CH5、CH6短接，则重新配对。
#define PIN_CH_OUT_P33
#define PIN_CH_IN_P32
// 动力电池电压检测引脚P1.1
#define PIN_ADC_P1X	1
// EEPROM页面大小
#define SIZE_STC_EEPROM_PAGE	512
#define SIZE_EEPROM_PAGE			SIZE_STC_EEPROM_PAGE
// 状态灯驱动
#define LED_ON()	do{LED = 1;}while(0)
#define LED_OFF()	do{LED = 0;}while(0)

/*////////////////////////////////////*/
#define FUNC_MILLIS_EN	0
#define FUNC_MICROS_EN	1
#define FUNC_DELAY_EN		1
#define FUNC_DELAYMICROSECONDS_EN	0

/*////////////////////////////////////////////////////////////////////////////*/
#elif STC_8H
/*////////////////////////////////////////////////////////////////////////////*/

#if STC_8H_TSSOP20

sbit LED=P1^2;

sbit CH1=P5^4;
sbit CH2=P1^7;
sbit CH3=P1^0;
sbit CH4=P3^7;
sbit CH5=P3^6;
sbit CH6=P3^4;
sbit CH7=P3^3;
sbit CH8=P3^0;

sbit CHx=P3^1;

sbit CE  =P3^2;
sbit SCK =P1^5;
sbit MISO=P1^4;
sbit IRQ =P1^6;
sbit MOSI=P1^3;
sbit CSN =P3^5;

#else
#endif

/*////////////////////////////////////*/
// 再次定义VOLTAGE_BAT_MULTIPLE，覆盖掉前面的数据
#if defined  VOLTAGE_BAT_MULTIPLE
#define VOLTAGE_BAT_MULTIPLE 4
#endif
#define PIN_CH_OUT_P37
#define PIN_CH_IN_P36
#define PIN_ADC_P1X	1
#define SIZE_STC_EEPROM_PAGE	512
#define SIZE_EEPROM_PAGE			SIZE_STC_EEPROM_PAGE
#define LED_ON()	do{LED = 1;}while(0)
#define LED_OFF()	do{LED = 0;}while(0)

/*////////////////////////////////////*/
#define FUNC_MILLIS_EN	0
#define FUNC_MICROS_EN	1
#define FUNC_DELAY_EN		1
#define FUNC_DELAYMICROSECONDS_EN	0

/*////////////////////////////////////////////////////////////////////////////*/
#elif ARDUINO_ATMEAG328P
/*////////////////////////////////////////////////////////////////////////////*/

#define DEF_SPI_SOFT
//定义指示灯
#define LED 13
//定义8个通道输出
//定义无线模块的管脚
#define IRQ   2
#define MISO  3
#define MOSI  4
#define SCK   5
#define CSN   6
#define CE    7

/*////////////////////////////////////////////////////////////////////////////*/
#endif
/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif

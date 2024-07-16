/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
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

#if 0

//定义指示灯
#define LED 7
//定义8个通道输出
#define CH1 1
#define CH2 2
#define CH3 3
#define CH4 4
#define CH5 5
#define CH6 6
#define CH7 7
#define CH8 15
//定义无线模块的管脚
#define MOSI  11
#define MISO  12
#define SCK   13
#define CSN   10
#define CE    8
#define IRQ   9

#else

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

#endif
/*////////////////////////////////////////////////////////////////////////////*/

#define noRF			digitalRead(IRQ)
#define PIN_SBUS	CH7

// enable SBUS_DEBUG 	would disable SBUS pin output
#define SBUS_DEBUG

// change the SBUS baud rate for debug. default:100K
#define SBUS_BPS_9600
//#define SBUS_BPS_115200

#define TIMERS_TICK_SETPS 3

//LED
#define LED_ON()        digitalWrite(LED, HIGH)
#define LED_OFF()       digitalWrite(LED, LOW)

/*////////////////////////////////////////////////////////////////////////////*/
/*////////////////////////////////////////////////////////////////////////////*/
// port for arduino
//NRF24L01
#define SPI_SLAVE_EN()  digitalWrite(CSN, LOW)
#define SPI_SLAVE_DIS() digitalWrite(CSN, HIGH)
#ifdef DEF_SPI_SOFT
#define SPI_SCK_CLR()   digitalWrite(SCK, LOW)
#define SPI_MOSI_CLR()  digitalWrite(MOSI, LOW)
#define SPI_SCK_SET()   digitalWrite(SCK, HIGH)
#define SPI_MOSI_SET()  digitalWrite(MOSI, HIGH)
#define SPI_MISO_GET()  digitalRead(MISO)
#endif
#define NRF_CE_SET()    digitalWrite(CE, HIGH)
#define NRF_CE_CLR()    digitalWrite(CE, LOW)


#define EEPROM_begin()
#define EEPROM_end()
#define DATA_mutex_get()
#define DATA_mutex_release()


#define UART_BPS_100K	0
#define UART_BPS_9600	1
#define UART_BPS_115200	2

#define SIZE_STC_EEPROM_PAGE	512
#define SIZE_EEPROM_PAGE			SIZE_STC_EEPROM_PAGE

#define INTERRUPT_dis()
#define INTERRUPT_en()

//T0启动
#define output_en()
//PCA控制寄存器：启动PCA定时器，清除中断标志
#define tick_en()



/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#ifdef SBUS_DEBUG
	#define DBG(str) UART_puts(str)
#else
	#define DBG(str)
#endif

/*////////////////////////////////////////////////////////////////////////////*/

#endif

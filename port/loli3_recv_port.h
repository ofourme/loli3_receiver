/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_RECV_PORT.H-V1.1.0 (2023.Oct.30th)
*
********************************************************************************
*/

#ifndef __LOLI3_RECV_PORT_H__
#define __LOLI3_RECV_PORT_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"
#include "config.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define UART_BPS_100K	0
#define UART_BPS_9600	1
#define UART_BPS_115200	2

/*////////////////////////////////////////////////////////////////////////////*/

// 系统初始化函数
void hw_sys_init();
//----------------------------------------------------------------------------//
// 基础功能函数
void hw_delay_1ms(void);
void hw_delay_ms(u8 ms);
//----------------------------------------------------------------------------//
// 端口初始化及输入输出函数
void hw_io_chs_init();
// CH口推挽输出
void hw_io_chs_pushpull();
	// 输出口
void hw_io_cho_init();
void hw_io_cho_clr();
void hw_io_cho_set();
	// 输入口
void hw_io_chi_init();
u8   hw_io_chi_get();
//----------------------------------------------------------------------------//
// 串口初始化及输出函数
#define UART_BPS_100K	0
#define UART_BPS_9600	1
#define UART_BPS_115200	2
void hw_uart_init(u8 bps);			// bps可选值[UART_BPS_100K, UART_BPS_9600, UART_BPS_115200]
void UART_pushByte(char c);			// push a byte into UART then return, not waitting for finished
void hw_uart_puts(const char* str);
//----------------------------------------------------------------------------//
// 电压ADC功能初始化及读取函数
void hw_adc_init();
void hw_adc_open(u8 i);	// adc转换预启动，减少adc_read等待时间。//0：内部参考电压；1：外部电压
u16  hw_adc_read();			// 接收机程序对ADC返回数据按10位处理，移植不同硬件需注意保持一致。
//----------------------------------------------------------------------------//
// 系统心跳功能初始化
void hw_tick_init();
		//启动系统周期性任务（PCA控制寄存器：启动PCA定时器，清除中断标志）
		//void hw_tick_en()	// #define hw_tick_en()			do{CCON =0x40;}while(0)

//兼容arduino的时间函数，依赖于hw_tick_init()
#if FUNC_MILLIS_EN
u32  millis(void);
#endif
#if FUNC_MICROS_EN
u16  micros(void);
#endif
#if FUNC_DELAY_EN
void delay(u16 ms);
#endif
#if FUNC_DELAYMICROSECONDS_EN
void delayMicroseconds(u16 us);
#endif

//----------------------------------------------------------------------------//
// 通道舵量输出/pwm输出功能初始化及使能
void hw_output_init();
		//启动CH通道输出（启动T0，T0中断函数控制输出）
		//void hw_output_en();
//通道ch范围[0,7]对应[CH1,CH8]; STC15W实现CH1&CH2，STC8H实现CH1&CH2&CH6&CH7
//通道pwm范围[0,255]
void hw_pwm_en (u8 ch);
void hw_pwm_dis(u8 ch);
void hw_pwm_set(u8 ch, u8 pwm);
//----------------------------------------------------------------------------//
// 状态灯操作函数
void LED_on(void);
void LED_off(void);
//----------------------------------------------------------------------------//
// 系统EEPROM功能函数
u8   EEPROM_read (u16 addr);
void EEPROM_write(u16 addr,u8 byte);
void EEPROM_cleanPage(u8 addPage);
#if 0
void EEPROM_begin();
void EEPROM_end();
void DATA_mutex_get();
void DATA_mutex_release();
#elif 0
#define EEPROM_begin()				do{IAP_CONTR=0x83;}while(0)
#define EEPROM_end()					do{IAP_CONTR=0x00;}while(0)
#define DATA_mutex_get() 			do{EA = 0;}while(0)
#define DATA_mutex_release()	do{EA = 1;}while(0)
#endif
//----------------------------------------------------------------------------//

// NRF24L01 SPI端口IO模式初始化
void SPI_init_0(void);
void SPI_init_1(void);

/*////////////////////////////////////////////////////////////////////////////*/
#if STC_15W408AS
/*////////////////////////////////////////////////////////////////////////////*/

// 24L01 IRQ 输出1表示没有中断发生
#define noRF()					IRQ
#define SPI_MISO_GET()	MISO
// stc51 uart 口输出0将导致串口无法输出，重置为1保证串口正常工作
#define PIN_SBUS_SET()	CH7=1
#define PIN_PPM_SET()		CH1=1
#define PIN_PPM_CLR()		CH1=0

#define hw_tick_en()					do{CCON =0x40;}while(0)
#define hw_output_en()				do{TR0=1;}while(0)
#define EEPROM_begin()				do{IAP_CONTR=0x83;}while(0)
#define EEPROM_end()					do{IAP_CONTR=0x00;}while(0)
//#define DATA_mutex_get() 			do{EA = 0;}while(0)
//#define DATA_mutex_release()	do{EA = 1;}while(0)
//假设数据仅在T0中断中被使用，在多字节数据修改前仅禁止T0中断而不是禁止所有中断，避免使用到错误的数据。
#define DATA_mutex_get() 			do{ET0 = 0;}while(0)
#define DATA_mutex_release()	do{ET0 = 1;}while(0)

//----------------------------------------------------------------------------//
// port for stc15w
#define NSS CSN
#define SPI_SLAVE_EN()	do{NSS =0;}while(0)
#define SPI_SLAVE_DIS()	do{NSS =1;}while(0)
#define SPI_SCK_SET()		do{SCK =1;}while(0)
#define SPI_SCK_CLR()		do{SCK =0;}while(0)
#define SPI_MOSI_SET()	do{MOSI=1;}while(0)
#define SPI_MOSI_CLR()	do{MOSI=0;}while(0)

#define NRF_CE_SET()		do{CE = 1;}while(0)
#define NRF_CE_CLR()		do{CE = 0;}while(0)

//----------------------------------------------------------------------------//
#define INTERRUPT_dis() do{EA = 0;}while(0)
#define INTERRUPT_en()	do{EA = 1;}while(0)

/*////////////////////////////////////////////////////////////////////////////*/
#elif STC_8H
/*////////////////////////////////////////////////////////////////////////////*/

#define noRF()								IRQ
#define SPI_MISO_GET()				MISO
#define PIN_SBUS_SET()				CHx=1
#define PIN_PPM_SET()					CHx=1
#define PIN_PPM_CLR()					CHx=0
// 启动定时器T1
#define hw_tick_en()					do{TR1=1;}while(0)
#define hw_output_en()				do{TR0=1;}while(0)
// stc8h 与 stc15w eeprom不兼容需要注意
#define EEPROM_begin()				do{IAP_TPS=((FOSC+500000)/1000000);IAP_CONTR=0x80;}while(0)
#define EEPROM_end()					do{IAP_CONTR=0x00;}while(0)
#define DATA_mutex_get() 			do{ET0 = 0;}while(0)
#define DATA_mutex_release()	do{ET0 = 1;}while(0)
#define NSS 									CSN
#define SPI_SLAVE_EN()				do{NSS =0;}while(0)
#define SPI_SLAVE_DIS()				do{NSS =1;}while(0)
#define SPI_SCK_SET()					do{SCK =1;}while(0)
#define SPI_SCK_CLR()					do{SCK =0;}while(0)
#define SPI_MOSI_SET()				do{MOSI=1;}while(0)
#define SPI_MOSI_CLR()				do{MOSI=0;}while(0)
#define NRF_CE_SET()					do{CE = 1;}while(0)
#define NRF_CE_CLR()					do{CE = 0;}while(0)
#define INTERRUPT_dis() 			do{EA = 0;}while(0)
#define INTERRUPT_en()				do{EA = 1;}while(0)

/*////////////////////////////////////////////////////////////////////////////*/
#elif ARDUINO_ATMEAG328P
/*////////////////////////////////////////////////////////////////////////////*/

#define noRF()					digitalRead(IRQ)
//LED
#define LED_ON()        digitalWrite(LED, HIGH)
#define LED_OFF()       digitalWrite(LED, LOW)

// port for arduino
//NRF24L01
#define SPI_SLAVE_EN()  digitalWrite(CSN, LOW)
#define SPI_SLAVE_DIS() digitalWrite(CSN, HIGH)
#define NRF_CE_SET()    digitalWrite(CE, HIGH)
#define NRF_CE_CLR()    digitalWrite(CE, LOW)

#ifdef DEF_SPI_SOFT
#define SPI_SCK_SET()   digitalWrite(SCK, HIGH)
#define SPI_SCK_CLR()   digitalWrite(SCK, LOW)
#define SPI_MOSI_SET()  digitalWrite(MOSI, HIGH)
#define SPI_MOSI_CLR()  digitalWrite(MOSI, LOW)
#define SPI_MISO_GET()  digitalRead(MISO)
#endif


#define EEPROM_begin()
#define EEPROM_end()
#define DATA_mutex_get()
#define DATA_mutex_release()

#define SIZE_STC_EEPROM_PAGE	512
#define SIZE_EEPROM_PAGE			SIZE_STC_EEPROM_PAGE

#define INTERRUPT_dis()
#define INTERRUPT_en()
/*////////////////////////////////////////////////////////////////////////////*/
#endif
/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif

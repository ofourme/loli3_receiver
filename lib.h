/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LIB.H-V1.0.1 (2021.Dec.11th)
*
********************************************************************************
*/

#ifndef __LIB_H__
#define __LIB_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define NSS CSN
#define SPI_SLAVE_EN()	do{NSS=0;}while(0)
#define SPI_SLAVE_DIS()	do{NSS=1;}while(0)
#define SPI_SCK_SET()		do{SCK=1;}while(0)
#define SPI_SCK_CLR()		do{SCK=0;}while(0)
#define SPI_MOSI_SET()	do{MOSI=1;}while(0)
#define SPI_MOSI_CLR()	do{MOSI=0;}while(0)

#define NRF_CE_SET()	do{CE=1;}while(0)
#define NRF_CE_CLR()	do{CE=0;}while(0)

#define EEPROM_begin()				do{IAP_CONTR=0x83;}while(0)
#define EEPROM_end()					do{IAP_CONTR=0x00;}while(0)
#define DATA_mutex_get() 			do{EA = 0;}while(0)
#define DATA_mutex_release()	do{EA = 1;}while(0)


#define UART_BPS_100K	0
#define UART_BPS_9600	1
#define UART_BPS_115200	2

#define SIZE_STC_EEPROM_PAGE	512
#define SIZE_EEPROM_PAGE			SIZE_STC_EEPROM_PAGE

#define INTERRUPT_dis() 		do{EA = 0;}while(0)
#define INTERRUPT_en()			do{EA = 1;}while(0)

//T0启动
#define output_en()		do{TR0=1;}while(0)
//PCA控制寄存器：启动PCA定时器，清除中断标志
#define tick_en()			do{CCON =0x40;}while(0)



/*////////////////////////////////////////////////////////////////////////////*/

typedef struct
{
	u8	isRuning;
	u8	isTimeout;
	u16 count;
	u16 reload;
	void (*callback)(void);

}	STimer;

/*////////////////////////////////////////////////////////////////////////////*/

u16  micros(void);
u32  millis(void);
void delay_1_ms();
void delay(u16 ms);
void delayMicroseconds(u16 us);

/*////////////////////////////////////////////////////////////////////////////*/

void port_init();
void port_ch5_mode_OUTPUT();	
void port_ch5_clr();
void port_ch5_set();
void port_ch6_mode_INPUT_PULLUP();
u8   port_ch6_get();
void port_chs_mode_OUTPUT();

void pwm_en(u8 ch);
void pwm_dis(u8 ch);
void pwm_set(u8 ch, u8 pwm);
void hw_init();
u16  adc_read();
void adc_open(u8 i);

/*////////////////////////////////////////////////////////////////////////////*/

u8		EEPROM_read (u16 addr);
void	EEPROM_write(u16 addr,u8 byte);
void	EEPROM_cleanPage(u8 addPage);
u8		EEPROM_test (void);

/*////////////////////////////////////////////////////////////////////////////*/
//NRF24L01
void NRF_FIFO_write(u8 DATA_OUT[],u8 lengh);
void NRF_FIFO_read(u8 DATA_IN[],u8 lengh);
void NRF_addr_tx(u8 DATA_IN[]);
void NRF_addr_rx(u8 DATA_IN[]);
void NRF_mode_rx();
void NRF_mode_tx();
void NRF_power(u8 P);
//void NRF_size(u8 l);
void NRF_channel(u8 c);
void NRF_init(void);//u8 ch, u8 address[]);
u8   NRF_test();
void NRF_irq_clean(void);

/*////////////////////////////////////////////////////////////////////////////*/

void LED_on(void);
void LED_off(void);
void LED_flash(u8 t);

/*////////////////////////////////////////////////////////////////////////////*/

void UART_init(u8 bps);
void UART_pushByte(char c);	// push a byte into UART then return, not waitting for finished
void UART_puts(char* str);
//void UART_putData(u16 i);
char* Int16ToStr(u16 i);

/*////////////////////////////////////////////////////////////////////////////*/

void timer_init(STimer* pTimers, u8 numTimers);
void timer_cbRegist(u8 index, void (*callback)(void));
void timer_startOnce(u8 index, u16 ms);
void timer_startTimes(u8 index, u16 ms, u8 times);
void timer_startCycle(u8 index, u16 ms);
void timer_tick();	// called by ISR
void timer_process();

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif
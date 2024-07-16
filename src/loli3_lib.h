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

#ifndef __LOLI3_LIB_H__
#define __LOLI3_LIB_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_recv_port.h"
#include "loli3_pact.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

typedef struct
{
	u8	isRuning;
	u8	isTimeout;
	u16 count;
	u16 reload;
	void (*callback)(void);

}	STimer;

/*////////////////////////////////////////////////////////////////////////////*/
u8		EEPROM_test (void);
void  LED_flash(u8 t);
char* Int16ToStr(u16 i);
/*----------------------------------------------------------------------------*/
//NRF24L01基础函数

void NRF_init(void);

u8   NRF_REG_read(u8 address);
void NRF_REG_write(u8 address,u8 command);
void NRF_FIFO_write(u8 DATA_OUT[],u8 lengh);
void NRF_FIFO_read(u8 DATA_IN[],u8 lengh);
void NRF_addr_tx(u8 DATA_IN[]);
void NRF_addr_rx(u8 DATA_IN[]);
void NRF_mode_rx();
void NRF_mode_tx();
void NRF_power(u8 P);
void NRF_channel(u8 c);
void NRF_irq_clean(void);
void NRF_reset();

#define NRF_BAD_CSNCLK	0x01
#define NRF_BAD_MOSI		0x02
#define NRF_BAD_CE			0x04
#define NRF_BAD_IRQ			0x08
#define NRF_BAD_MISO		0x10
#define NRF_BAD_RESET		0x80
u8   		NRF_test();

/*----------------------------------------------------------------------------*/
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

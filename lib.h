#ifndef __LIB_H__
#define __LIB_H__

#include "hw.h"


void Delay1ms();
void delay_ms(u8 i);

/**************************************************************************/

u8		EEPROM_read (u8 address_H,u16 address_L);
void	EEPROM_write(u8 address_H,u16 address_L,u8 byte);
void	EEPROM_clean(u8 address_H);
u8		EEPROM_test (void);
#define EEPROM_open()					do{IAP_CONTR=0x83;}while(0)
#define EEPROM_close()				do{IAP_CONTR=0x00;}while(0)
#define DATA_mutex_get() 			do{EA = 0;}while(0)
#define DATA_mutex_release()	do{EA = 1;}while(0)


//NRF24L01

void FIFO_write(u8 DATA_OUT[],u8 lengh);
void FIFO_read(u8 DATA_IN[],u8 lengh);
void NRF_addr_tx(u8 DATA_IN[]);
void NRF_addr_rx(u8 DATA_IN[]);
void NRF_mode_rx();
void NRF_mode_tx();
void NRF_power(u8 P);
//void NRF_size(u8 l);
void NRF_channel(u8 c);
void NRF_init(u8 ch, u8 address[]);
u8   NRF_test();
void NRF_irq_clean(void);


void LED_on(void);
void LED_off(void);
void LED_flash(u8 t);

#endif
/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2008-2021, 微风山谷/ofourme@163.com
*           License: LGPL
*
*   LIB.C-V0.5.0 (2021.Dec.5th)
*
********************************************************************************
*/

#define __LIB_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "lib.h"
#include "config.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

//延时函数
void Delay1ms()		//@12.000MHz
{
	unsigned char i, j;
	i = 12;
	j = 168;
	do
	{
		while (--j);
	} while (--i);
}

void delay_ms(u8 i)
{
	while(i--)
	Delay1ms();
}

/*////////////////////////////////////////////////////////////////////////////*/
//ROM存储
u8 EEPROM_read(u8 address_H,u16 address_L)	//读数据
{
	IAP_CMD=0x01;
	if(address_L>>8)
	{
		IAP_ADDRH=address_H+1;
		IAP_ADDRL=(u8)address_L;	//修正错误？：IAP_ADDRL=address_L-255;
	}
	else 
	{
		IAP_ADDRH=address_H;
		IAP_ADDRL=address_L;	
	}
	
	IAP_TRIG=0x5a;			
	IAP_TRIG=0xa5;
	_nop_();
	return IAP_DATA;
}

void EEPROM_write(u8 address_H,u16 address_L,u8 byte)	//写入数据
{
	IAP_CMD=0x02;
	IAP_DATA=byte;
	if(address_L>>8)
	{
		IAP_ADDRH=address_H+1;
		IAP_ADDRL=(u8)address_L;	//修正错误？：IAP_ADDRL=address_L-255;
	}
	else 
	{
		IAP_ADDRH=address_H;
		IAP_ADDRL=address_L;	
	}

	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
	_nop_();
}

void EEPROM_clean(u8 address_H)	//擦除数据
{
	IAP_CMD=0x03;
	IAP_ADDRH=address_H;
	IAP_ADDRL=0;
	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
	_nop_();
}

u8 EEPROM_test(void)
{
	u8 address_H = 2;
	
	IAP_CONTR=0x83;
	
	EEPROM_clean(address_H);
	EEPROM_write(address_H,0,0x88);
	if(EEPROM_read(address_H,0)==0x88)
	{
		EEPROM_clean(address_H);
		EEPROM_write(address_H,0,0x55);
		if(EEPROM_read(address_H,0)==0x55)
		{
			return 0;	//ok
		}
	}
	
	return 1;	//err
}


/*////////////////////////////////////////////////////////////////////////////*/
//NRF24L01
u8 SPI(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		MOSI=(byte&0x80);
		SCK=1;
		byte<<=1;
		byte|=MISO;
		SCK=0;
	}
	return byte;
}

void REG_write(u8 address,u8 command)
{
	CSN=0;
	SPI(0x20+address);
	SPI(command);
	CSN=1;
}

void FIFO_write(u8 DATA_OUT[],u8 lengh)
{
	u8 i;
	CSN=0;
	SPI(0xa0);
	for(i=0;i<lengh;i++)
	SPI(DATA_OUT[i]);
	CSN=1;
}
void FIFO_read(u8 DATA_IN[],u8 lengh)		//读取接收数据缓冲区
{
	u8 i;
	CSN=0;
	SPI(0x61);	//读取命令
	for(i=0;i<lengh;i++)
	DATA_IN[i]=SPI(0);	   
	CSN=1;
}


void NRF_addr_tx(u8 DATA_IN[])
{
	CSN=0;		 
	SPI(0x20+0x10);
	SPI(DATA_IN[0]);
	SPI(DATA_IN[1]);
	SPI(DATA_IN[2]);
	SPI(DATA_IN[3]);
	SPI(DATA_IN[4]);
	CSN=1;  
}  
void NRF_addr_rx(u8 DATA_IN[])
{
	CSN=0;		 
	SPI(0x20+0x0a);
	SPI(DATA_IN[0]);
	SPI(DATA_IN[1]);
	SPI(DATA_IN[2]);
	SPI(DATA_IN[3]);
	SPI(DATA_IN[4]);
	CSN=1;  
}
void NRF_mode_rx()				 
{
	CE=0;
	REG_write(0x00,0x3b); //CRC,8 bit,Power on,RX
	CE=1;
} 				   
	
void NRF_mode_tx()				 
{
	CE=0;
	REG_write(0x00,0x0a);
  CE=1;
}

void  NRF_power(u8 P)				//发射功率设置 250k
{														
	CE=0;
	if(P==3)REG_write(0x06,0x27);		  //0db 修正之前注释错误
	else if(P==2)REG_write(0x06,0x25);	  //-6db
	else if(P==1)REG_write(0x06,0x23);	  //-12db
	else if(P==0)REG_write(0x06,0x21);    //-18db
	CE=1;
}

void NRF_size(u8 l)
{
	CE=0;
	REG_write(0x11,l);  
	CE=1;
}

void NRF_channel(u8 c)
{
	CE=0;
	REG_write(0x05,c);  
	CE=1;
}


void NRF_init(u8 ch,u8 address[])
{
	u8 TX_power=3;
	
	CE=0;
	SCK=0;
	REG_write(0x01,0x00); //禁止 自动应答
	REG_write(0x02,0x01); //允许 P0信道
	REG_write(0x04,0x00); //禁止 自动重发
	NRF_mode_rx();
	NRF_channel(ch);
	NRF_power(TX_power);
	NRF_size(11);
	NRF_addr_rx(address);
	NRF_addr_tx(address);
}

void NRF_irq_clean(void)
{
		CE=0;
		REG_write(0x07,0x40);	//清除无线模块中断信号
		CE=1;
}

u8 NRF_test()	//无线模块终极测试
{	
	u8 reset_err=0;
	u8 NRF_error = 0;
	u8 tx[1] = 'T';	// test?

	CE=0;
	SCK=0;
	CSN=0;
	
	
	if(SPI(0x20)!=0x0e){reset_err=1;}
	SPI(0x0a);

	CSN=1;

	CSN=0;
	SPI(0x00);

	if(SPI(0x00)!=0x0a){NRF_error|=0x02;}//MOSI bad
	CSN=1;

	REG_write(0x01,0x00);
	REG_write(0x04,0x00);
	REG_write(0x11,1);
	
	FIFO_write(tx,1);
	CE=1;

	delay_ms(2);
	
	CSN=0;

	if(SPI(0x00)!=0x2e){NRF_error|=0x04;}//CE bad
	CSN=1;
		
	if(IRQ)NRF_error|=0x18;	//IRQ bad
	else 
	{
		if(NRF_error&0x04==0)NRF_error|=0x10;		//MISO bad
	}
	CE=1;
	
	if(reset_err&&NRF_error>1)NRF_error|=0x01;//CSN,CLK bad
	
	REG_write(0x07,0x20);	//清除TX中断信号
	
	return NRF_error;
}

/*////////////////////////////////////////////////////////////////////////////*/

void LED_on(void)
{
	LED_ON();
}

void LED_off(void)
{
	LED_OFF();
}

void LED_flash(u8 t)
{
	while(t)
	{
		LED_ON();		
		delay_ms(50);
		LED_OFF();		
		delay_ms(50);
		t--;
	}
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
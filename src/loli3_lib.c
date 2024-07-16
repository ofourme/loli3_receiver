/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_LIB.C-V1.0.0 (2023.Sep.25th)
*
********************************************************************************
*/

#define __LOLI3_LIB_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_lib.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

//串口通信
#if defined( SBUS_BPS_115200 )
	#define UART_BPS	UART_BPS_115200
#elif	defined ( SBUS_BPS_9600 )
	#define	UART_BPS	UART_BPS_9600
#else
	#define UART_BPS	UART_BPS_100K
#endif

/*////////////////////////////////////////////////////////////////////////////*/
// soft SPI
#define SPI_MODE_0		(0<<0)
#define SPI_MODE_1		(1<<0)
#define SPI_MODE_2		(2<<0)
#define SPI_MODE_3		(3<<0)
#define SPI_MSBFIRST	(0<<2)
#define SPI_LSBFIRST	(1<<2)
#define SPI_SPEED_SOF	(0<<3)

u8   SPI_begin(u8 setting);
u8   SPI_transfer(u8 byte);
void SPI_slaveEnable(void);
void SPI_slaveDisable(void);

u8 SPI_begin(u8 setting)
{
	if(setting!=(SPI_MODE_0|SPI_MSBFIRST)) return 0;

	// set SCK/MOSI/NSS output mode, MISO input mode
  SPI_init_0();//端口模式初始化
	// then set SCK/MOSI 0, set NNS 1.
	SPI_init_1();//软件or硬件初始化

	return 1;
}

u8 SPI_transfer(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(byte&0x80) SPI_MOSI_SET();
		else 					SPI_MOSI_CLR();
		SPI_SCK_SET();
		byte<<=1;
		byte|=SPI_MISO_GET();
		SPI_SCK_CLR();
	}
	return byte;
}

void SPI_slaveEnable(void)
{
	SPI_SLAVE_EN();
}

void SPI_slaveDisable(void)
{
	SPI_SLAVE_DIS();
}

/*
void SPI_end(void)
{

}
*/

/*////////////////////////////////////////////////////////////////////////////*/
//NRF24L01
u8 NRF_REG_read(u8 address)
{
  u8 t;
  SPI_slaveEnable();
  SPI_transfer(0x00+address);
  t = SPI_transfer(0x00);
  SPI_slaveDisable();

  return t;
}

void NRF_REG_write(u8 address,u8 command)
{
	SPI_slaveEnable();
	SPI_transfer(0x20+address);
	SPI_transfer(command);
	SPI_slaveDisable();
}

void NRF_init_0(void)
{
  SPI_begin(0);
	NRF_CE_CLR();
	NRF_REG_write(0x00,0x0a); //CRC使能，上电
	NRF_REG_write(0x01,0x00); //禁止 自动应答
	NRF_REG_write(0x02,0x01); //允许 P0信道
	NRF_REG_write(0x04,0x00); //禁止 自动重发
	NRF_CE_SET();
}

void NRF_FIFO_write(u8 DATA_OUT[],u8 lengh)
{
	u8 i;

	SPI_slaveEnable();
	SPI_transfer(0xa0);
	for(i=0;i<lengh;i++) SPI_transfer(DATA_OUT[i]);
	SPI_slaveDisable();
}

void NRF_FIFO_read(u8 DATA_IN[],u8 lengh)
{
	u8 i;

	SPI_slaveEnable();
	SPI_transfer(0x61);	//读取命令
	for(i=0;i<lengh;i++) DATA_IN[i]=SPI_transfer(0);
	SPI_slaveDisable();
}

void NRF_addr_tx(u8 DATA_IN[])
{
	SPI_slaveEnable();
	SPI_transfer(0x20+0x10);
	SPI_transfer(DATA_IN[0]);
	SPI_transfer(DATA_IN[1]);
	SPI_transfer(DATA_IN[2]);
	SPI_transfer(DATA_IN[3]);
	SPI_transfer(DATA_IN[4]);
	SPI_slaveDisable();
}
void NRF_addr_rx(u8 DATA_IN[])
{
	SPI_slaveEnable();
	SPI_transfer(0x20+0x0a);
	SPI_transfer(DATA_IN[0]);
	SPI_transfer(DATA_IN[1]);
	SPI_transfer(DATA_IN[2]);
	SPI_transfer(DATA_IN[3]);
	SPI_transfer(DATA_IN[4]);
	SPI_slaveDisable();
}
void NRF_mode_rx()
{
	NRF_CE_CLR();
	NRF_REG_write(0x00,0x3b); //CRC,8 bit,Power on,RX
	NRF_CE_SET();
}

void NRF_mode_tx()
{
	NRF_CE_CLR();
	NRF_REG_write(0x00,0x0a);
  NRF_CE_SET();
}

void  NRF_power(u8 P)				//发射功率设置
{
	NRF_CE_CLR();
	if(P>=3)		 NRF_REG_write(0x06,0x27);		//0db 修正之前注释错误
	else if(P==2)NRF_REG_write(0x06,0x25);	  //-6db
	else if(P==1)NRF_REG_write(0x06,0x23);	  //-12db
	else if(P==0)NRF_REG_write(0x06,0x21);    //-18db
	NRF_CE_SET();
}

void NRF_channel(u8 c)
{
	NRF_CE_CLR();
	NRF_REG_write(0x05,c);
	NRF_CE_SET();
}

void NRF_irq_clean(void)
{
		NRF_CE_CLR();
		NRF_REG_write(0x07,0x70);	//清除无线模块（重发失败+发送完成+数据接受）中断信号
		NRF_CE_SET();
}

void NRF_data_length(u8 l)
{
	NRF_CE_CLR();
	NRF_REG_write(0x11,l);
	NRF_CE_SET();
}

void NRF_init(void)	//u8 ch,u8 address[])
{
	NRF_init_0();
	NRF_mode_rx();
//	NRF_channel(ch);
	NRF_power(0);
	NRF_data_length(LOLI3_NRF_DATA_LENGTH);
//	NRF_addr_rx(address);
//	NRF_addr_tx(address);
}

void NRF_reset()
{
  NRF_init_0();
  NRF_irq_clean();

  SPI_slaveEnable();
  SPI_transfer(0xe1); // FLUSH_TX
  SPI_slaveDisable();
  SPI_slaveEnable();
  SPI_transfer(0xe2); // FLUSH_RX
  SPI_slaveDisable();
}

u8 NRF_test()	//无线模块终极测试
{
	u8 NRF_error = 0;
	u8 tx[1] = {'T'};
  u8 temp;

	NRF_CE_CLR();
#if 1
	if(noRF()==0)         NRF_error |= NRF_BAD_IRQ;
#endif

	// 参考与24L01兼容的《SI24R1中文用户手册》，每一次 SPI 操作，MISO 输出的第一字节为状态寄存器的值，之后通过命令来确定是否输出值(不输出为高阻态)。
	SPI_slaveEnable();
	if((temp=SPI_transfer(0x20))!=0x0e){// 状态寄存器RX FIFO不为空，说明模块没有重启
    NRF_error |= NRF_BAD_RESET;
	}
	SPI_transfer(0x0a);	// 将0x0a写入配置寄存器
	SPI_slaveDisable();

	SPI_slaveEnable();
	SPI_transfer(0x00);
	if(SPI_transfer(0x00)!=0x0a){NRF_error|=NRF_BAD_MOSI;}//MOSI bad
	SPI_slaveDisable();

	NRF_REG_write(0x01,0x00);
	NRF_REG_write(0x04,0x00);
	NRF_REG_write(0x11,1);
	NRF_FIFO_write(tx,1);
	NRF_CE_SET();	// 启动发送
	delay(2);			// 等待发送完毕
	SPI_slaveEnable();
	if(SPI_transfer(0x00)!=0x2e){NRF_error|=NRF_BAD_CE+NRF_BAD_MISO;}//状态寄存器没有发射完成中断，说明CE bad
	SPI_slaveDisable();

	if(noRF()) NRF_error|=NRF_BAD_IRQ;

	if(NRF_error && NRF_error!=NRF_BAD_RESET) NRF_error|=NRF_BAD_CSNCLK;//CSN,CLK bad

	NRF_irq_clean();
  //NRF_CE_SET();

	return NRF_error;
}

/*////////////////////////////////////////////////////////////////////////////*/

u8 EEPROM_test(void)
{
	u8 addrPage = 1;

	EEPROM_begin();

	EEPROM_cleanPage(addrPage);
	EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0xAA);
	if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0xAA)	{EEPROM_end();	return 1;	}//err
	EEPROM_cleanPage(addrPage);
	EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0x55);
	if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0x55)	{EEPROM_end();	return 1;	}//err

	EEPROM_end();	// bugfix(20230924): add in
	
	return 0;	//ok
}

/*----------------------------------------------------------------------------*/

void LED_flash(u8 t)
{
	while(t)
	{
		LED_ON();
		delay(10);
		LED_OFF();
		delay(90);
		t--;
	}
}

/*----------------------------------------------------------------------------*/

char* Int16ToStr(u16 i)
{
	u8 t;
	static xdata char str[7];

	str[0] = '0';
	str[1] = 'x';
	t=i>>12;
	if(t<10) str[2]=(t+'0');
	else		 str[2]=(t+'A'-10);
	t=(i>>8)&0x0F;
	if(t<10) str[3]=(t+'0');
	else		 str[3]=(t+'A'-10);
	t=(i>>4)&0x0F;
	if(t<10) str[4]=(t+'0');
	else		 str[4]=(t+'A'-10);
	t=(i>>0)&0x0F;
	if(t<10) str[5]=(t+'0');
	else		 str[5]=(t+'A'-10);
	str[6] = '\0';

	return str;
}

/*////////////////////////////////////////////////////////////////////////////*/

static STimer* _pTimers;
static u8	_numTimers=0;

void timer_init(STimer* pTimers, u8 numTimers)
{
	u8 i;

	_pTimers = pTimers;
	_numTimers = numTimers;

	for(i=0; i<_numTimers; i++)	// stop all the timers to prevent exception
	{
		_pTimers[i].isRuning  = 0;		// prevent timers change by <timers_tick>
		_pTimers[i].isTimeout = 0;		// prevent timers call by <timers_process>
		_pTimers[i].count 		 = 0;
		_pTimers[i].reload 	 = 0;
		_pTimers[i].callback  = 0;
	}
}

void timer_cbRegist(u8 index, void (*callback)(void))
{
/*
	_pTimers[index].isRuning = 0;
	_pTimers[index].isTimeout = 0;
	_pTimers[index].count = 0;
	_pTimers[index].reload = 0;
*/
	_pTimers[index].callback = callback;
}

void timer_startOnce(u8 index, u16 ms)
{
	_pTimers[index].isRuning = 0;	// stop the timer to prevent exception
	_pTimers[index].isTimeout = 0;
	_pTimers[index].count = ms;
	_pTimers[index].reload = 0;
	_pTimers[index].isRuning = 1;	// start the timer
}

void timer_startTimes(u8 index, u16 ms, u8 times)
{
	_pTimers[index].isRuning = 0;	// stop the timer to prevent exception
	_pTimers[index].isTimeout = 0;
	_pTimers[index].count = _pTimers[index].reload = ms;
	_pTimers[index].isRuning = times;	// start the timer
}

void timer_startCycle(u8 index, u16 ms)
{
	timer_startTimes(index, ms, 0xff);
}

void timer_tick()	// called by ISR
{
	static u8 step=0;
	u8 i;

	step++;
	if(step < LOLI3_RECV_TIMERS_TICK_SETPS) return;
	step = 0;

	for(i=0;i<_numTimers;i++)
	{
		if(_pTimers[i].isRuning)
		{
			if(_pTimers[i].count>LOLI3_RECV_TIMERS_TICK_SETPS) _pTimers[i].count -= LOLI3_RECV_TIMERS_TICK_SETPS;
			else
			{
				_pTimers[i].count = _pTimers[i].reload;
				if(_pTimers[i].isRuning != 0xFF) _pTimers[i].isRuning--;
				if(_pTimers[i].reload==0) _pTimers[i].isRuning = 0;
				_pTimers[i].isTimeout = 1;
			}
		}
	}
}

void timer_process()
{
	u8 i;

	for(i=0;i<_numTimers;i++)
	{
		if(_pTimers[i].isTimeout)
		{
			_pTimers[i].isTimeout = 0;
			if(_pTimers[i].callback) _pTimers[i].callback();
		}
	}

}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
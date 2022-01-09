/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   APP.C-V0.6.0 (2021.Dec.26th)
*
********************************************************************************
*/

#define __APP_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "lib.h"
#include "app.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/*////////////////////////////////////////////////////////////////////////////*/

const u8 code _random_hopping_index[100]={
					4,1,3,2,2,1,0,0,2,2,2,3,4,1,2,1,4,3,3,4,//随机跳频序列
					2,0,2,2,3,1,2,3,2,2,2,4,2,4,0,3,4,2,3,1,
					0,3,1,3,3,0,2,0,4,3,3,3,3,3,4,1,1,4,3,0,
					1,0,3,2,3,2,3,3,4,4,1,3,0,0,3,1,3,3,3,0,
					3,3,4,1,2,4,1,3,0,1,3,4,4,3,2,3,1,2,3,3};

const u8 code recv_address_startup[5]={LOLI3_ADDRESS_STARTUP};

Loli3RecvSet xdata	recvSet = {0,LOLI3_HOPPING_STARTUP,LOLI3_ADDRESS_STARTUP,0,511,511,80,511,511,511,511,511};
Loli3RecvData		recv 	= {0,1023,0,1023,0,1023,0,0};
Loli3RecvNrf  xdata	nrf 	= {0};

enum {STATE_LED_OFF=0, STATE_LED_FLASH, STATE_LED_ON, STATE_LED_IDLE} E_STATE_LED;
enum {STATE_NRF_INIT=0, STATE_NRF_RX, STATE_NRF_TX, STATE_NRF_TX_WAIT, STATE_NRF_DATA_PROC} E_STATE_NRF;
enum {TIMER_LED_FLASH=0, TIMER_SIGNAL_COUNT, TIMER_SIGNAL_LOST, TIMER_DATA_SAVE,  TIMER_LED_OFF, TIMER_CHANNEL_LOST, TIMER_ADC, NUM_TIMERS} E_TIMER;

STimer idata timer[NUM_TIMERS];

u16 _adc_ic,_adc_batt;


/*////////////////////////////////////////////////////////////////////////////*/

#define MAGIC_RECV_SET	0xab

void DATA_read()
{
	u8 i;
	u16 sum,sum_read;
	u8* set;

	recvSet.isValid = 0;
	EEPROM_begin();
	if( EEPROM_read(0)!= MAGIC_RECV_SET )
	{
		EEPROM_end();
		return;
	}

	sum = MAGIC_RECV_SET;
	for(i=1,set=(u8*)&recvSet+1;i<sizeof(Loli3RecvSet);i++)
	{
		*set=EEPROM_read(i);
		sum+=*set++;
	}

	sum_read=EEPROM_read(sizeof(Loli3RecvSet));//读校验和
	sum_read<<=8;
	sum_read+=EEPROM_read(sizeof(Loli3RecvSet)+1);

	EEPROM_end();

	if(sum!=sum_read) return;	//比较校验和
	recvSet.isValid = MAGIC_RECV_SET;
	return;
}

void DATA_save()
{
	u8 i;
	u16 sum;
	u8* set;

	EEPROM_begin();
	EEPROM_cleanPage(0);
	recvSet.isValid = MAGIC_RECV_SET;
	for(i=0,sum=0,set=(u8*)&recvSet;i<sizeof(Loli3RecvSet);i++)
	{
		EEPROM_write(i,*set);
		sum+=*set++;
	}
	EEPROM_write(sizeof(Loli3RecvSet),sum>>8);
	EEPROM_write(sizeof(Loli3RecvSet)+1,sum);

	EEPROM_end();
}

#undef MAGIC_RECV_SET

/*////////////////////////////////////////////////////////////////////////////*/

void SBUS_data_push()
{
	u8 i;
	u8 buff[12];

	buff[1] = 						   (u8)(recv.CH_data[0])<<1;
	buff[2] = (u8)(recv.CH_data[0]>>7)+(u8)(recv.CH_data[1])<<4;
	buff[3] = (u8)(recv.CH_data[1]>>4)+(u8)(recv.CH_data[2])<<7;
	buff[4] = (u8)(recv.CH_data[2]>>1);
	buff[5] = (u8)(recv.CH_data[2]>>9)+(u8)(recv.CH_data[3])<<2;
	buff[6] = (u8)(recv.CH_data[3]>>6)+(u8)(recv.CH_data[4])<<5;
	buff[7] = (u8)(recv.CH_data[4]>>3)+(u8)(recv.CH_data[5])<<8;
	buff[8] = (u8)(recv.CH_data[5]);
	buff[9] = (u8)(recv.CH_data[5]>>8)+(u8)(recv.CH_data[6])<<3;
	buff[10]= (u8)(recv.CH_data[6]>>5)+(u8)(recv.CH_data[7])<<6;
	buff[11]= (u8)(recv.CH_data[7]>>2);

	recv.SBUS_tx[0]=0x0f;	//Star
	DATA_mutex_get();			//防止修改SBUS数据的时候，中断程序在端口输出错误数据
	for(i=1	; i<=11; i++)	//CH1~CH8
	{
		recv.SBUS_tx[i]=buff[i];
	}
	DATA_mutex_release();
	for(		; i<=22; i++)	//CH9~CH16
	{
		recv.SBUS_tx[i]=0;
	}
	recv.SBUS_tx[23]=0;		//flag
	recv.SBUS_tx[24]=0;		//End
}

/*////////////////////////////////////////////////////////////////////////////*/

void callback_timer_signal_count(void)
{
		recv.rx_num=nrf.rx_cnt;
		nrf.rx_cnt=0;
}

void callback_timer_signal_lost(void)
{
	u8 i;

	DBG("LOLI3 recv signal lost!!!");
	recv.stateLED = STATE_LED_ON;

	DATA_mutex_get();
	for(i=0; i<8; i++)
	{
		recv.CH_data[i]=recvSet.out_control_data[i];
	}
	DATA_mutex_release();

	SBUS_data_push();
}

void callback_timer_data_save(void)
{
	DATA_save();
	recv.stateLED = STATE_LED_FLASH;
}

void callback_timer_led_off(void)
{
	recv.stateLED = STATE_LED_OFF;
}

void callback_timer_channel_lost(void)
{
	static u8 index = 0;

	if( noRF )
	{
		nrf.hopping_cnt++;
		if(nrf.hopping_cnt>LOLI3_NUM_HOPPING)
		{
			if(index>=   (sizeof(_random_hopping_index)/sizeof(_random_hopping_index[0])) ) index=0;
			nrf.channel_index =  _random_hopping_index[index++];
		}
		else
		{
			nrf.channel_index++;
			if(nrf.channel_index>=LOLI3_NUM_HOPPING)nrf.channel_index=0;
		}
		NRF_channel(recvSet.hopping[nrf.channel_index]);
	}
}

void callback_timer_adc(void)
{
	static u8 i=0;
	u16 adc;

	adc = adc_read();
	i++;
	if(i&0x01)
	{
		_adc_ic = adc + ((_adc_ic*7)>>3);	//扩大8倍
		adc_open(1);
	}
	else
	{
		_adc_batt = adc + ((_adc_batt*7)>>3);	//扩大8倍
		adc_open(0);
	}
}

void timers_init()
{
	timer_init(timer, NUM_TIMERS);

	timer_cbRegist(TIMER_SIGNAL_COUNT,callback_timer_signal_count);
	timer_cbRegist(TIMER_SIGNAL_LOST, callback_timer_signal_lost);
	timer_cbRegist(TIMER_DATA_SAVE,		callback_timer_data_save);
	timer_cbRegist(TIMER_LED_OFF,			callback_timer_led_off);
	timer_cbRegist(TIMER_CHANNEL_LOST,callback_timer_channel_lost);
	timer_cbRegist(TIMER_ADC,					callback_timer_adc);
}


/*////////////////////////////////////////////////////////////////////////////*/

void NRF_read(void)
{
	NRF_FIFO_read(nrf.rx, LOLI3_NRF_DATA_LENGTH);
}

void NRF_write(void)
{
	NRF_FIFO_write(nrf.tx,LOLI3_NRF_DATA_LENGTH);
}

/*////////////////////////////////////////////////////////////////////////////*/

void recv_connect()
{
		NRF_channel(LOLI3_CHANNEL_CONNECT);
		NRF_addr_rx(recvSet.address);
		NRF_addr_tx(recvSet.address);
		NRF_power(3);
}

void recv_reconnect()
{
	u8 i,t;

	NRF_channel(LOLI3_CHANNEL_STARTUP);
	NRF_addr_tx(recv_address_startup);
	NRF_addr_rx(recv_address_startup);

	do
	{
		while(noRF);
		NRF_read();
		NRF_irq_clean();

	}	while(nrf.rx[0]!=0xa0);


	for(i=0,t=1; i<LOLI3_NUM_HOPPING; i++,t++)
	{
		recvSet.hopping[i]=nrf.rx[t];
	}
	for(i=0; i<5; i++,t++)
	{
		recvSet.address[i]=nrf.rx[t];
	}
	nrf.tx[0]='O',nrf.tx[1]='K';

	while(1)
	{
		NRF_mode_tx();
		NRF_channel(LOLI3_CHANNEL_STARTUP);
		NRF_addr_tx(recv_address_startup);
		NRF_addr_rx(recv_address_startup);
		NRF_write();
		delay_1_ms();

		NRF_mode_rx();
		NRF_channel(recvSet.hopping[0]);
		NRF_addr_tx(recvSet.address);
		NRF_addr_rx(recvSet.address);
		for(t=100; t--; )
		{
			delay_1_ms();
			if(noRF==0)	// 接收到信号
			{
				NRF_read();
				NRF_irq_clean();
				DATA_save();
				NRF_power(3);
				return;
			}
		}
	}
}

/*////////////////////////////////////////////////////////////////////////////*/

void recv_init()
{
	u8 restar = 0;
	u8 temp;

	port_ch5_mode_OUTPUT();
	port_ch6_mode_INPUT_PULLUP();
	port_ch5_clr();
	delay_1_ms();
	if(port_ch6_get()==0)	//如果CH5与CH6被短接，重新对码
	{
		port_ch5_set();		// 插上舵机会拉低CH6电平。改变CH5电平，确认CH5与CH6短接。
		delay_1_ms();
		if(port_ch6_get())		LED_flash(20), restar=1;   //启动重新对码
	}
	port_chs_mode_OUTPUT();

	while(EEPROM_test())
	{
		LED_flash(3);
		DBG("EEPROM error!");
		delay(1000);
	}
	DATA_read();

	temp = NRF_test();
	while(temp)
	{
		LED_flash(10);
		DBG("NRF24L01 error!");
		DBG(Int16ToStr(temp));
		delay(1000);
		temp = NRF_test();
	}

	NRF_init();
	if(restar || recvSet.isValid==0)
	{
		DBG("LOLI3 recv reconnecting...");
		recv_reconnect();
	}
	else
	{
		DBG("LOLI3 recv connecting...");
		recv_connect();
	}

	recv.stateLED = STATE_LED_OFF;
	recv.stateNRF = STATE_NRF_INIT;
}


void recv_begin(void)
{
	if(recvSet.CH1_PWM) pwm_en(1);
	if(recvSet.CH2_PWM) pwm_en(2);

	output_en();
	tick_en();
	INTERRUPT_en();
}


/*////////////////////////////////////////////////////////////////////////////*/

void setup(void)
{
	port_init();
	LED_on();
	delay(500);//开机延时以避过电源波动

	hw_init();

	timers_init();
	timer_startCycle(TIMER_ADC, 50);
	timer_startCycle(TIMER_SIGNAL_COUNT,1000);
	DBG("LOLI3 recv start up.");
	recv_init();
	LED_off();		//点亮指示灯再关闭，表示单片机正常工作
	DBG("LOLI3 recv connected.");
	NRF_channel(recvSet.hopping[0]);
	while(noRF);
	recv_begin();
}

void loop (void)
{
	u16 buff[8];
	u8 i;
	static u16 t1,t2;

		timer_process();

		switch(recv.stateLED)
		{
			case STATE_LED_OFF:
				LED_off();
				recv.stateLED = STATE_LED_IDLE;
				break;
			case STATE_LED_FLASH:
				LED_on();
				timer_startOnce(TIMER_LED_OFF,10);
				recv.stateLED = STATE_LED_IDLE;
				break;
			case STATE_LED_ON:
				LED_on();
				break;
			case STATE_LED_IDLE:
				break;
			default:
				break;
		}

		switch(recv.stateNRF)
		{
			case STATE_NRF_RX:
				if(noRF) break;
			
				if(recv.stateLED ==STATE_LED_ON) recv.stateLED = STATE_LED_OFF;

				nrf.rx_cnt++;
				nrf.hopping_cnt=0;//收到有效信号后刷新跳频器

				timer_startOnce(TIMER_SIGNAL_LOST, 2000);
				timer_startCycle(TIMER_CHANNEL_LOST, 24);	//3*7~3*8

				NRF_read();		//读取接收数据
				NRF_irq_clean();

				NRF_mode_tx();
				recv.stateNRF = STATE_NRF_TX;
				//break;
			case STATE_NRF_TX:
				nrf.tx[0]=recv.rx_num;
				#define BGV5	260
				recv.voltage_ic=(5ul*100*8)*BGV5/_adc_ic;	 // 详见《STC15》P898说明，此处输出voltage扩大100倍，输入adc值扩大8倍，故*100*8
				nrf.tx[1]=recv.voltage_ic>>8;
				nrf.tx[2]=recv.voltage_ic;
				recv.voltage_batt=((long)_adc_batt*recv.voltage_ic*3)>>13;
				nrf.tx[3]=recv.voltage_batt>>8;
				nrf.tx[4]=recv.voltage_batt;
				NRF_write();

				recv.stateNRF = STATE_NRF_TX_WAIT;
				t1 = micros();
				//break;
			case STATE_NRF_TX_WAIT:
				t2 = micros()-t1;
				if(!noRF || t2>=1000)	// wait about 0.72ms for TX_DS irq
				{
					if(t2>=1000) DBG(Int16ToStr(t2));	// timeout
					NRF_mode_rx();	// NRF_irq_clean(); change mode would clean irq.
					recv.stateNRF = STATE_NRF_DATA_PROC;
				}
				break;
			case STATE_NRF_DATA_PROC:
				nrf.channel_index++;
				if(nrf.channel_index>4)nrf.channel_index=0;
				NRF_channel(recvSet.hopping[nrf.channel_index]);

				if(nrf.rx[0]==0xa2)
				{
					if(nrf.rx[1]&0x80)	{	recvSet.PPM=1;	}
					else  							{	recvSet.PPM=0;	}
					if(nrf.rx[1]&0x40)	{	recvSet.SBUS=1;			}
					else  							{	recvSet.SBUS=0;			}
					if(nrf.rx[1]&0x08)	{	recvSet.CH1_PWM=1; pwm_en(1);	}
					else 								{	recvSet.CH1_PWM=0; pwm_dis(1);}
					if(nrf.rx[1]&0x04)	{	recvSet.CH2_PWM=1; pwm_en(2);	}
					else 								{	recvSet.CH2_PWM=0; pwm_dis(2);}
					if(nrf.rx[1]&0x02)	{	recvSet.CH7_PWM=1;	}//pwm_ch7_en(); }
					else 								{	recvSet.CH7_PWM=0;	}//pwm_ch7_dis();}
					if(nrf.rx[2]&0x80)	{ recvSet.CH1_SW=1;		}
					else 								{ recvSet.CH1_SW=0;		}
					if(nrf.rx[2]&0x40)	{ recvSet.CH2_SW=1;		}
					else 								{ recvSet.CH2_SW=0;		}
					if(nrf.rx[2]&0x20)	{	recvSet.CH3_SW=1;		}
					else								{	recvSet.CH3_SW=0;		}
					if(nrf.rx[2]&0x10)	{	recvSet.CH4_SW=1;		}
					else								{	recvSet.CH4_SW=0;		}
					if(nrf.rx[2]&0x08)	{	recvSet.CH5_SW=1;		}
					else								{	recvSet.CH5_SW=0;		}
					if(nrf.rx[2]&0x04)	{	recvSet.CH6_SW=1;		}
					else								{	recvSet.CH6_SW=0;		}
					if(nrf.rx[2]&0x02)	{	recvSet.CH7_SW=1;		}
					else								{	recvSet.CH7_SW=0;		}
					if(nrf.rx[2]&0x01)	{	recvSet.CH8_SW=1;		}
					else								{ recvSet.CH8_SW=0;		}

					recv.bModeChange = 1;
					timer_startOnce(TIMER_DATA_SAVE, 1000);	// save data 1000ms later
				}
				else if(nrf.rx[0]==0xa0 || nrf.rx[0]==0xa1)
				{
					buff[0]=nrf.rx[1];
					buff[0]<<=2;
					buff[0]+=nrf.rx[2]>>6;
					buff[1]=nrf.rx[2]&0x3f;
					buff[1]<<=4;
					buff[1]+=nrf.rx[3]>>4;
					buff[2]=nrf.rx[3]&0x0f;
					buff[2]<<=6;
					buff[2]+=nrf.rx[4]>>2;
					buff[3]=nrf.rx[4]&0x03;
					buff[3]<<=8;
					buff[3]+=nrf.rx[5];

					buff[4]=nrf.rx[6];
					buff[4]<<=2;
					buff[4]+=nrf.rx[7]>>6;
					buff[5]=nrf.rx[7]&0x3f;
					buff[5]<<=4;
					buff[5]+=nrf.rx[8]>>4;
					buff[6]=nrf.rx[8]&0x0f;
					buff[6]<<=6;
					buff[6]+=nrf.rx[9]>>2;
					buff[7]=nrf.rx[9]&0x03;
					buff[7]<<=8;
					buff[7]+=nrf.rx[10];

					if(nrf.rx[0]==0xa0)
					{
						for(i=0; i<8; i++)
						{
							recvSet.out_control_data[i]=buff[i];
						}

						timer_startOnce(TIMER_DATA_SAVE, 1000);	// save data 1000ms later
					}
					else	// nrf.rx[0]==0xa1
					{
						for(i=0; i<8; i++)
						{
							DATA_mutex_get();		// 防止中断程序输出错误数据
							recv.CH_data[i]=buff[i];
							DATA_mutex_release();
						}
						SBUS_data_push();
					}
			}
				
			recv.stateNRF = STATE_NRF_RX;
			break;
		default:
			//NRF_mode_rx();
			recv.stateNRF = STATE_NRF_RX;
			break;
		}
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

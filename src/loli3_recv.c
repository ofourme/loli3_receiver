/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_RECV.C-V1.0.0 (2023.Sep.25th)
*
********************************************************************************
*/

#define __LOLI3_RECV_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_recv.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/*////////////////////////////////////////////////////////////////////////////*/

#define MAGIC_RECV_SET  0xab

void DATA_read()
{
  u8 i;
  u16 sum,sum_read;
  u8* set;
	
	u8 xdata T[sizeof(Loli3RecvSet)];
	
  EEPROM_begin();
  for(sum=i=0;i<sizeof(Loli3RecvSet);i++)
  {
    T[i]=EEPROM_read(i);
    sum+=T[i];
  }
  sum_read=EEPROM_read(sizeof(Loli3RecvSet));//读校验和
  sum_read<<=8;
  sum_read+=EEPROM_read(sizeof(Loli3RecvSet)+1);
  EEPROM_end();

	recvSet.isValid = 0;
  if(T[0]!= MAGIC_RECV_SET || sum!=sum_read) return; //比较起始码和校验和
	// 读取的数据完全没问题才写入recvSet，防止错误的数据覆盖recvSet默认数据
  for(i=1,set=(u8*)&recvSet;i<sizeof(Loli3RecvSet);i++) set[i]=T[i];
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

  buff[1] =                (u8)(recv.CH_data[0])<<1;
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

  recv.SBUS_tx[0]=0x0f; //Star
  DATA_mutex_get();     //防止修改SBUS数据的时候，中断程序在端口输出错误数据
  for(i=1 ; i<=11; i++) //CH1~CH8
  {
    recv.SBUS_tx[i]=buff[i];
  }
  DATA_mutex_release();
  for(    ; i<=22; i++) //CH9~CH16
  {
    recv.SBUS_tx[i]=0;
  }
  recv.SBUS_tx[23]=0;   //flag
  recv.SBUS_tx[24]=0;   //End
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

  if( noRF() )
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

  adc = hw_adc_read();
  i++;
  if(i&0x01)
  {
    _adc_ic = adc + ((_adc_ic*7)>>3); //扩大8倍
    hw_adc_open(1);
  }
  else
  {
    _adc_batt = adc + ((_adc_batt*7)>>3); //扩大8倍
    hw_adc_open(0);
  }
}

void timers_init()
{
  timer_init(timer, NUM_TIMERS);

	timer_cbRegist(TIMER_SIGNAL_COUNT,callback_timer_signal_count);
  timer_cbRegist(TIMER_SIGNAL_LOST, callback_timer_signal_lost);
  timer_cbRegist(TIMER_DATA_SAVE,   callback_timer_data_save);
  timer_cbRegist(TIMER_LED_OFF,     callback_timer_led_off);
  timer_cbRegist(TIMER_CHANNEL_LOST,callback_timer_channel_lost);
  timer_cbRegist(TIMER_ADC,         callback_timer_adc);
}


/*////////////////////////////////////////////////////////////////////////////*/
#if 0
void recv_nrf_read(void)
{
  NRF_FIFO_read(nrf.rx, LOLI3_NRF_DATA_LENGTH);
}

void recv_nrf_write(void)
{
  NRF_FIFO_write(nrf.tx,LOLI3_NRF_DATA_LENGTH);
}
#endif
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
    while(noRF());
    NRF_FIFO_read(nrf.rx, LOLI3_NRF_DATA_LENGTH);
    NRF_irq_clean();

  } while(nrf.rx[0]!=0xa0);


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
    NRF_FIFO_write(nrf.tx, LOLI3_NRF_DATA_LENGTH);
    hw_delay_1ms();

    NRF_mode_rx();
    NRF_channel(recvSet.hopping[0]);
    NRF_addr_tx(recvSet.address);
    NRF_addr_rx(recvSet.address);
    for(t=100; t--; )
    {
      hw_delay_1ms();
      if(noRF()==0) // 接收到信号
      {
        NRF_FIFO_read(nrf.rx, LOLI3_NRF_DATA_LENGTH);
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

  hw_io_cho_init();
  hw_io_chi_init();
  hw_io_cho_clr();
  hw_delay_1ms();
  if(hw_io_chi_get()==0) //如果CH5与CH6被短接，重新对码
  {
    hw_io_cho_set();   // 插上舵机会拉低CH6电平。改变CH5电平，确认CH5与CH6短接。
    hw_delay_1ms();
    if(hw_io_chi_get())    LED_flash(20), restar=1;   //启动重新对码
  }
  hw_io_chs_pushpull();

  while(EEPROM_test())
  {
    LED_flash(2);
    DBG("EEPROM error!");
    delay(1000);
  }
  DATA_read();

  NRF_reset();	// 防止CPU复位但模块未断电复位导致的错误
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

	if(recvSet.SBUS)			recv.stateOUTPUT = STATE_OUTPUT_SBUS;
	else if(recvSet.PPM)  recv.stateOUTPUT = STATE_OUTPUT_PPM;
	else
	{
		recv.stateOUTPUT = STATE_OUTPUT_SERVO;
	  if(recvSet.CH1_PWM) hw_pwm_en(0);
		if(recvSet.CH2_PWM) hw_pwm_en(1);
		if(recvSet.CH6_PWM) hw_pwm_en(5);	// STC8H扩展loli3 各版本CH6通道没有的pwm功能
		if(recvSet.CH7_PWM) hw_pwm_en(6);	// STC8H恢复loli3 STC15W版本CH7通道没有的pwm功能
	}
	recv.isOutputChanged = 1;

  recv.stateLED = STATE_LED_OFF;
  recv.stateNRF = STATE_NRF_INIT;
}


void recv_begin(void)
{
  hw_output_en();
  hw_tick_en();
  INTERRUPT_en();
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_RECV.H-V1.0.0 (2023.Sep.25th)
*
********************************************************************************
*/

#ifndef __LOLI3_RECV_H__
#define __LOLI3_RECV_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"
#include "config.h"
#include "loli3_pact.h"
#include "loli3_lib.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

typedef struct
{
	u8 isValid;
	u8 hopping[LOLI3_NUM_HOPPING];		// 频道/channel数组，循环使用
	u8 address[5];		// 接收机地址
	u8 SBUS;					// 是否SBUS输出
	u8 PPM;						// 是否PPM输出
	u8 CH1_PWM,CH2_PWM,CH7_PWM;	// 是否PMW输出（CH7输出无效，此处仅为兼容Loli3发送机，同时保证out_control_data字节对齐）
	u8 CH1_SW,CH2_SW,CH3_SW,CH4_SW,CH5_SW,CH6_SW,CH7_SW,CH8_SW;	// 是否开关量输出
	u16 out_control_data[8];
#if STC_8H
	u8 CH6_PWM;	// STC8H新增CH6/CH7输出PWM，其中CH6输出需要修改发射机程序增加相关。
#endif
	
} Loli3RecvSet;		// 接收机设置，保存在EEPROM里

typedef struct
{
	u16 CH_data[8];		// 通道数值[0:1023]
	u16 voltage_ic, voltage_batt;	// 芯片电压,电池电压：数值扩大100倍，如125表示1.25V
	u16 adc_ic, adc_batt;					// 电压adc转换滤波数据暂存
	u8 	rx_num;				// 每秒接收到发送机数据次数
	u8 	SBUS_tx[25];	// 通道数值转换为SBUS数组
	u8  stateLED;			// LED状态
	u8  stateNRF;
	u8  stateOUTPUT;
	u8  isOutputChanged;

} Loli3RecvData;	// 接收机动态数据

typedef struct
{
	u8 rx[LOLI3_NRF_DATA_LENGTH];
	u8 tx[LOLI3_NRF_DATA_LENGTH];
	u8 rx_cnt;				// 发送机信号计数器
	u8 hopping_cnt;		// 信号丢失跳频计数器
	u8 channel_index;	// 频道索引

} Loli3RecvNrf;			// 无线模块数据

typedef enum {STATE_LED_OFF=0, STATE_LED_FLASH, STATE_LED_ON, STATE_LED_IDLE} E_STATE_LED;
typedef enum {STATE_NRF_INIT=0, STATE_NRF_RX, STATE_NRF_TX, STATE_NRF_TX_WAIT, STATE_NRF_DATA_PROC} E_STATE_NRF;
typedef enum {STATE_OUTPUT_SERVO=0, STATE_OUTPUT_PPM, STATE_OUTPUT_SBUS} E_STATE_OUTPUT;
typedef enum {TIMER_LED_FLASH=0, TIMER_SIGNAL_COUNT, TIMER_SIGNAL_LOST, TIMER_DATA_SAVE,  TIMER_LED_OFF, TIMER_CHANNEL_LOST, TIMER_ADC, NUM_TIMERS} E_TIMER;

/*////////////////////////////////////////////////////////////////////////////*/
#ifdef __LOLI3_RECV_C__
/*----------------------------------------------------------------------------*/

// 伪随机跳频索引
const u8 code _random_hopping_index[100]={
          4,1,3,2,2,1,0,0,2,2,2,3,4,1,2,1,4,3,3,4,//随机跳频序列
          2,0,2,2,3,1,2,3,2,2,2,4,2,4,0,3,4,2,3,1,
          0,3,1,3,3,0,2,0,4,3,3,3,3,3,4,1,1,4,3,0,
          1,0,3,2,3,2,3,3,4,4,1,3,0,0,3,1,3,3,3,0,
          3,3,4,1,2,4,1,3,0,1,3,4,4,3,2,3,1,2,3,3};

// 开机地址
const u8 code recv_address_startup[5]={LOLI3_ADDRESS_STARTUP};

//结构体未赋值成员默认为0
Loli3RecvSet xdata  recvSet = {0,LOLI3_HOPPING_STARTUP,LOLI3_ADDRESS_STARTUP,0,0,0,0,0,511,511,0,511,511,511,511,511};
Loli3RecvData   		recv  	= {0,1023,0,1023,0,1023,0,0};
Loli3RecvNrf  xdata nrf   	= {0};

STimer idata timer[NUM_TIMERS];

// 芯片电压、动力电池电压 ADC 缓存
u16 _adc_ic,_adc_batt;

/*----------------------------------------------------------------------------*/
#else
/*----------------------------------------------------------------------------*/

extern Loli3RecvSet  xdata recvSet;
extern Loli3RecvData recv;
extern Loli3RecvNrf  xdata nrf;
extern u16 _adc_ic;
extern u16 _adc_batt;

/*----------------------------------------------------------------------------*/
#endif
/*////////////////////////////////////////////////////////////////////////////*//*////////////////////////////////////////////////////////////////////////////*/

void DATA_read();
void DATA_save();

void callback_timer_signal_count(void);
void callback_timer_signal_lost(void);
void callback_timer_data_save(void);
void callback_timer_led_off(void);
void callback_timer_channel_lost(void);
void callback_timer_adc(void);
void timers_init();

/*////////////////////////////////////////////////////////////////////////////*/

void SBUS_data_push();

void recv_connect();
void recv_reconnect();
void recv_init();
void recv_begin(void);
#if 0
void recv_nrf_read(void);
void recv_nrf_write(void);
#endif
/*////////////////////////////////////////////////////////////////////////////*/



/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif

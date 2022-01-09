/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   APP.H-V1.0.0 (2021.Dec.25th)
*
********************************************************************************
*/

#ifndef __APP_H__
#define __APP_H__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "config.h"
#include "loli3_pact.h"

/*////////////////////////////////////////////////////////////////////////////*/

typedef struct
{
	u8 isValid;
	u8 hopping[LOLI3_NUM_HOPPING];		// 频道/channel数组，循环使用
	u8 address[5];		// 接收机地址
	u8 PPM;						// 是否PPM输出
	u16 out_control_data[8];
	u8 CH1_PWM,CH2_PWM,CH7_PWM;	// 是否PMW输出（CH7输出无效，此处仅为兼容Loli3发送机）
	u8 CH1_SW,CH2_SW,CH3_SW,CH4_SW,CH5_SW,CH6_SW,CH7_SW,CH8_SW;	// 是否开关量输出
	u8 SBUS;					// 是否SBUS输出

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
	u8  bModeChange;

	} Loli3RecvData;	// 接收机动态数据

typedef struct
{
	u8 rx[LOLI3_NRF_DATA_LENGTH];
	u8 tx[LOLI3_NRF_DATA_LENGTH];
	u8 rx_cnt;				// 发送机信号计数器
	u8 hopping_cnt;		// 信号丢失跳频计数器
	u8 channel_index;	// 频道索引

} Loli3RecvNrf;			// 无线模块数据


/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

#endif
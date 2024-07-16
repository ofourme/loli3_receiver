/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_RECV_ISR_STC.C-V1.0.0 (2023.Oct.25th)
*		硬件固定运行频率：12MHz
*
********************************************************************************
*/

#define __LOLI3_RECV_ISR_STC_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_recv.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// 以下变量在loli3_recv_sys中定义，仅在loli3_recv_sys与loli3_recv_isr中使用。
// 声明类型务必注意与loli3_recv_sys保持一致。

extern idata volatile u32 _sys_ms;				// 当前ms值
extern idata volatile u16 _sys_us_target;	// 中断发生目标us值

/*////////////////////////////////////////////////////////////////////////////*/
// 中断函数
// 注意事项1：中断函数中调用函数的可重入问题需关注。
// 注意事项2：如果使用“using 1”“using 2”等，能够在不同中断中使用不同寄存器组，提升响应速度……
//						但要注意中断中调用函数的所使用的寄存器地址的正确性。推荐使用#pragma NOAREGS/#pragma AREGS，使用相对寄存器寻址。
//						或者，是否可以使用内联函数来避免这个问题？
/*////////////////////////////////////////////////////////////////////////////*/

#if STC_15W408AS
// sys tick per ms	// PCA模块0每1ms中断一次
void isr_tick_PCA() interrupt 7
{
	CCF0 = 0;	//清除中断标志
	_sys_us_target += 1000;
	_sys_ms ++;
	CCAP0L = _sys_us_target;
	CCAP0H = _sys_us_target >> 8;

	timer_tick();
}

#elif STC_8H

void isr_tick_T1() interrupt 3
{
	_sys_ms ++;
	_sys_us_target += 1000;	// 在STC8H的实现中，用于缓存us计数

	timer_tick();
}

#endif

/*////////////////////////////////////////////////////////////////////////////*/
// 以下时间按定时器工作于12T模式@12MHz频率计算
#define TIMER_DELAY_11MS		(0x10000-FOSC/12/1000000*11000)
#define TIMER_DELAY_10MS		(0x10000-FOSC/12/1000000*10000)
#define TIMER_DELAY_120US		(0x10000-FOSC/12/1000000*120)
#define TIMER_DELAY_400US		(0x10000-FOSC/12/1000000*400)

void isr_output_T0()interrupt 1	//using 1
{
/*
	通道CHn输出中断函数，使用定时器0中断用于驱动信号输出，设置高优先级中断可提升输出波形时序准确性
	C51中断函数注意事项：注意hw_pwm_set()和UART_pushByte()的重入问题和using调用函数问题！！！
*/
/*
	STC定时器模式0(16位自动重装载模式): 
		当定时器运行时，对THn,TLn写入内容实际上是写入隐藏的寄存器RL_THn，RL_TLn中。
		定时器溢出时会自动将[RL_THn,RL_TLn]的内容重新装入[THn,TLn]。
		既当前中断响应中设置的定时时间，是下一中断响应相关操作的持续时间。
*/
/*
	原程序存在的问题：
		中断程序对输出模式切换没有进行处理，不可避免的会输出一小段错误数据。
		具体来说就是: 切换到不同输出模式（SBUS/PPM/舵量）时，具体阶段是不确定的，该阶段的持续时间也是不确定的。
*/
	
	static u8 step = 0;	// 输出状态 // 当SBUS，初始setp==25
	static u8 T_h=0,T_l=0;// 定时器装载值暂存

	u16 temp=0;
	static u16 temp0=0;
	
//----------------------------------------------------------------------------//
/*
	SBUS波特率100K，每位10us*每字节数据12位（1起始+8数据+1偶校核+2停止）=120us/字节
	SBUS每帧数据25字节，低速模式每帧间隔14ms（120us*25+11ms），高速模式每帧间隔7ms（120us*25+4ms）
*/
/*
	原程序SBUS这一段写的很简洁精准，就是不易理解。伪代码解读如下：
		PIN_SBUS=1;
		if(t_sbus>24) {	t_sbus=0;	}
		else 					{ PUTCH(sbus_data[t_sbus]); t_sbus++;	}
		if(t_sbus==25){ TIMER_DELAY_10MS  }	// 遵照SBUS低速模式协议，此处实际应为TIMER_DELAY_11MS
		else 					{ TIMER_DELAY_120US }

		t_sbus范围[0-25]对应26个阶段，其中[0-24]同时对应 sbus输出的25字节，[25]对应帧间时间间隔(11MS);
		[0-25]阶段设置的T0模式0时间，对应下一阶段[1-25,0]的延时；
		但状态已经提前转移（t_sbus++或t_sbus=0），故此时数据t_sbus实际指示的是下一阶段；
		既在t_sbus==25阶段设置的间隔时间，实际是在第24阶段已经设置好，且将在第25阶段生效。
*/
	if(recv.stateOUTPUT==STATE_OUTPUT_SBUS)	// SBUS
	{
		//if(reset) {	reset=0;	step=25;	}	// 跳转到输出SBUS[0]的前一阶段以设置SBUS[0]持续时间
		PIN_SBUS_SET();							// 强制将串口输出口设置为空闲状态，清除软件操作端口导致的错误。
		if(step>=25)	{	step=0; }	// 此状态无操作，低速模式时延时11ms
		else												// 此状态输出1字节数据，持续时间120us
		{
#ifndef SBUS_DEBUG
#if 0
			UART_pushByte(recv.SBUS_tx[step]);	// 是否存在重入/寄存器组不一致问题？
#else
			if(TI)TI=0;
			ACC=recv.SBUS_tx[step];
			TB8=P;
			SBUF=ACC;
#endif
			step++;
#endif
		}

		if(step>=25)	{	TL0=TIMER_DELAY_11MS;	TH0=TIMER_DELAY_11MS>>8;	}
		else 					{	TL0=TIMER_DELAY_120US;TH0=TIMER_DELAY_120US>>8; }

		if(recv.isOutputChanged && step==0)
		{
			recv.isOutputChanged = 0;
			if(recvSet.PPM)						{ recv.stateOUTPUT=STATE_OUTPUT_PPM;   step=0;}
			else if(recvSet.SBUS==0)	{ recv.stateOUTPUT=STATE_OUTPUT_SERVO; step=0;}
		}
	}
//----------------------------------------------------------------------------//
	else if(recv.stateOUTPUT==STATE_OUTPUT_PPM)	// PPM
	{
		//从PPM端口输出8通道数据，PPM信号总周期20ms
		//每通道固定0.4ms低电平开始，0.6到1.6ms（实际1.623ms）高电平结束
		//PPM_CH_PULSE()参数x无实际作用，用于标记对应0-7通道输出
		#define PPM_CH_PULSE(x)		do{PIN_PPM_CLR();TL0=T_l;TH0=T_h;}while(0)
		//PPM_CH_WIDTH()参数x无实际作用，用于标记对应0-7通道输出
		#define PPM_CH_WIDTH(x)	do{PIN_PPM_SET();TL0=TIMER_DELAY_400US;	TH0=TIMER_DELAY_400US>>8;}while(0)
		//PPM尾部直接延时10ms以节约计算时间，周期不保证50Hz		
		#define PPM_CH_END()		do{PIN_PPM_CLR();TL0=TIMER_DELAY_10MS;	TH0=TIMER_DELAY_10MS>>8;}while(0)
		#define PPM_CH_IDLE()		PPM_CH_WIDTH(8)
		#define	PPM_CH_WIDTH_TIME_GET(ch) do{temp=0x10000-600-recv.CH_data[ch]; T_l=temp;T_h=temp>>8;}while(0)

		switch(step)
		{
			case 0: PPM_CH_IDLE();
							PPM_CH_WIDTH_TIME_GET(0);
							break;
			case 1:	PPM_CH_PULSE(0);break;
			case 2:	PPM_CH_WIDTH(0);
							PPM_CH_WIDTH_TIME_GET(1);
							break;
			case 3:	PPM_CH_PULSE(1);break;
			case 4:	PPM_CH_WIDTH(1);
							PPM_CH_WIDTH_TIME_GET(2);
							break;
			case 5:	PPM_CH_PULSE(2);break;
			case 6:	PPM_CH_WIDTH(2);
							PPM_CH_WIDTH_TIME_GET(3);
							break;
			case 7:	PPM_CH_PULSE(3);break;
			case 8:	PPM_CH_WIDTH(3);
							PPM_CH_WIDTH_TIME_GET(4);
							break;
			case  9:PPM_CH_PULSE(4);break;
			case 10:PPM_CH_WIDTH(4);
							PPM_CH_WIDTH_TIME_GET(5);
							break;
			case 11:PPM_CH_PULSE(5);break;
			case 12:PPM_CH_WIDTH(5);
							PPM_CH_WIDTH_TIME_GET(6);
							break;
			case 13:PPM_CH_PULSE(6);break;
			case 14:PPM_CH_WIDTH(6);
							PPM_CH_WIDTH_TIME_GET(7);
							break;
			case 15:PPM_CH_PULSE(7);break;
			case 16:PPM_CH_WIDTH(7);
							break;
			case 17:PPM_CH_END();	 break;
			default:break;
		}

		if(recv.isOutputChanged && step==0)
		{
			recv.isOutputChanged = 0;
			if(recvSet.SBUS)				{ recv.stateOUTPUT=STATE_OUTPUT_SBUS;  step=25;}
			else if(recvSet.PPM==0)	{ recv.stateOUTPUT=STATE_OUTPUT_SERVO; step=0;}
		}

		step++; if(step>17) step=0;
	}
//----------------------------------------------------------------------------//
	else
	{	// 舵量输出周期20ms，8通道平分则每通道耗时2.5ms
		// PULSE_TIME+INTERVER_TIME==2.5ms，PULSE_TIME=0x10000-temp0, INTERVER_TIME=0x10000-temp
		// -> temp=0x10000-INTERVER_TIME=0x10000+PULSE_TIME-2500=0x10000-2500+0x10000-temp0=0x20000-2500-temp0
		// 经计算，temp=128572-temp0，loli3原程序temp=128600-temp0似乎有错？！
		#define SERVO_CH_PULSE_TIME_SET(ch)	\
				do{ temp0=0x10000-813-((recv.CH_data[ch]*LOLI3_RECV_SERVO_RANGE_MUL)>>LOLI3_RECV_SERVO_RANGE_R_SHIFT) ;\
						TL0=temp0,TH0=temp0>>8;}\
				while(0)
		//SERVO_CH_INTERVER_TIME_SET()参数x无实际作用，用于标记对应0-7通道输出
		#define SERVO_CH_INTERVER_TIME_SET(x)	\
				do{ temp=0x20000-2500-temp0; TL0=temp,TH0=temp>>8;}\
				while(0)
				
		switch(step)
		{
			case 0:
				if(recvSet.CH8_SW==0) CH8=0;
				SERVO_CH_PULSE_TIME_SET(0);
				break;

			case 1:
				if(recvSet.CH1_PWM)			hw_pwm_set(0,recv.CH_data[0]/4);
				else if(recvSet.CH1_SW){if(recv.CH_data[0]<500) CH1=0; else	CH1=1;}
				else										CH1=1;
				SERVO_CH_INTERVER_TIME_SET(0);
				break;
			case 2:
				if(recvSet.CH1_PWM==0 && recvSet.CH1_SW==0)	CH1=0;
				SERVO_CH_PULSE_TIME_SET(1);
				break;

			case 3:
				if(recvSet.CH2_PWM)			hw_pwm_set(1,recv.CH_data[1]/4);
				else if(recvSet.CH2_SW){if(recv.CH_data[1]<500) CH2=0; else CH2=1;}
				else 										CH2=1;
				SERVO_CH_INTERVER_TIME_SET(1);
				break;
			case 4:
				if(recvSet.CH2_PWM==0 && recvSet.CH2_SW==0)	CH2=0;
				SERVO_CH_PULSE_TIME_SET(2);
				break;

			case 5:
				if(recvSet.CH3_SW) {if(recv.CH_data[2]<500) CH3=0; else CH3=1;}
				else								CH3=1;
				SERVO_CH_INTERVER_TIME_SET(2);
				break;
			case 6:
				if(recvSet.CH3_SW==0) CH3=0;
				SERVO_CH_PULSE_TIME_SET(3);
				break;

			case 7:
				if(recvSet.CH4_SW) {if(recv.CH_data[3]<500) CH4=0; else CH4=1;}
				else 								CH4=1;
				SERVO_CH_INTERVER_TIME_SET(3);
				break;
			case 8:
				if(recvSet.CH4_SW==0) CH4=0;
				SERVO_CH_PULSE_TIME_SET(4);
				break;

			case 9:
				if(recvSet.CH5_SW) {if(recv.CH_data[4]<500) CH5=0; else CH5=1;}
				else								CH5=1;
				SERVO_CH_INTERVER_TIME_SET(4);
				break;
			case 10:
				if(recvSet.CH5_SW==0) CH5=0;
				SERVO_CH_PULSE_TIME_SET(5);
				break;
			// #if defined STC_15W408AS // bugfix20240518: STC_15W408AS为0但这个宏已define故恒成立，导致后面STC8H的内容没有编译进去，导致debug的时候CH7无输出
#if STC_15W408AS
			case 11:
				if(recvSet.CH6_SW) {if(recv.CH_data[5]<500) CH6=0; else CH6=1;}
				else								CH6=1;
				SERVO_CH_INTERVER_TIME_SET(5);
				break;
			case 12:
				if(recvSet.CH6_SW==0) CH6=0;
				SERVO_CH_PULSE_TIME_SET(6);
				break;
#elif STC_8H
//STC_8H_TSSOP20增加CH6通道pwm 输出功能
			case 11:
				if(recvSet.CH6_PWM)			hw_pwm_set(5,recv.CH_data[5]/4);
				else if(recvSet.CH6_SW){if(recv.CH_data[5]<500) CH6=0; else CH6=1;}
				else 										CH6=1;
				SERVO_CH_INTERVER_TIME_SET(5);
				break;
			case 12:
				if(recvSet.CH6_PWM==0 && recvSet.CH6_SW==0)	CH6=0;
				SERVO_CH_PULSE_TIME_SET(6);
				break;			
#endif

#if STC_15W408AS
			case 13:
				// 硬件串口用于调试：STC_15W408AS SBUS占用通道CH7
				#if (!defined SBUS_DEBUG)
				if(recvSet.CH7_SW) {if(recv.CH_data[6]<500) CH7=0; else CH7=1;}
				else 								CH7=1;
				#endif
				SERVO_CH_INTERVER_TIME_SET(6);
				break;
			case 14:
				#ifndef SBUS_DEBUG
				if(recvSet.CH7_SW==0) CH7=0;
				#endif
				SERVO_CH_PULSE_TIME_SET(7);
				break;
#elif STC_8H
			//STC_8H_TSSOP20增加CH6通道pwm 输出功能，且SBUS/PPM通道独立，硬件串口调试时无需屏蔽CH7通道输出
			case 13:
				if(recvSet.CH7_PWM)			hw_pwm_set(6,recv.CH_data[6]/4);
				else if(recvSet.CH7_SW){if(recv.CH_data[6]<500) CH7=0; else CH7=1;}
				else 										CH7=1;
				SERVO_CH_INTERVER_TIME_SET(6);
				break;
			case 14:
				if(recvSet.CH7_PWM==0 && recvSet.CH7_SW==0)	CH7=0;
				SERVO_CH_PULSE_TIME_SET(7);
				break;			
#endif
			case 15:
				if(recvSet.CH8_SW) {if(recv.CH_data[7]<500) CH8=0; else CH8=1;}
				else 								CH8=1;
				SERVO_CH_INTERVER_TIME_SET(7);
				break;
			default:break;
		}
		if(recv.isOutputChanged && step==0)
		{
			recv.isOutputChanged = 0;
			if(recvSet.SBUS) 			{ recv.stateOUTPUT=STATE_OUTPUT_SBUS; step=25;}
			else if(recvSet.PPM)	{ recv.stateOUTPUT=STATE_OUTPUT_PPM; step=0;}
		}

		step++; if(step>15) step=0;
	}
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

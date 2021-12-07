/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2008-2021, 微风山谷/ofourme@163.com
*           License: LGPL
*
*   MAIN.C-V0.5.0 (2021.Dec.5th)
*
********************************************************************************
*/

#define __MAIN_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "hw.h"
#include "config.h"
#include "lib.h"
#include "loli3_pact.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#define TIMERS_TICK_SETPS 3

/*////////////////////////////////////////////////////////////////////////////*/

typedef struct 
{
	u8 isValid;
	u8 hopping[5];
	u8 address[5];
	u8 PPM;
	u16 out_control_data[8];
	u8 CH1_PWM,CH2_PWM,CH7_PWM;
	u8 CH1_SW,CH2_SW,CH3_SW,CH4_SW,CH5_SW,CH6_SW,CH7_SW,CH8_SW;
	u8 SBUS;
} Loli3RecvSet;

typedef struct 
{
	u16 CH_data[8];
	u16 voltage_ic, voltage_batt;	//数值扩大100倍，如125表示1.25V
	u16 adc_ic, adc_batt;
	u8 	rx_num;
	u8 	SBUS_tx[25];
	u8  stateLED;

} Loli3RecvData;

typedef struct 
{
	u8 rx[11];
	u8 tx[11];
	u8 rx_cnt;
	u8 hopping_cnt;
	u8 hopping_index;

} Loli3RecvNrf;

typedef struct
{
	u8	isRuning;
	u8	isTimeout;
	u16 count;
	u16 reload;
	void (*callback)(void);
	
}	STimer;


u8 code random[100]={4,1,3,2,2,1,0,0,2,2,2,3,4,1,2,1,4,3,3,4,//随机跳频序列
										 2,0,2,2,3,1,2,3,2,2,2,4,2,4,0,3,4,2,3,1,
										 0,3,1,3,3,0,2,0,4,3,3,3,3,3,4,1,1,4,3,0,
	                   1,0,3,2,3,2,3,3,4,4,1,3,0,0,3,1,3,3,3,0,
	                   3,3,4,1,2,4,1,3,0,1,3,4,4,3,2,3,1,2,3,3};
u8 code 						recv_address_startup[5]={LOLI3_ADDRESS_STARTUP};										 

Loli3RecvSet xdata	recvSet = {0,LOLI3_HOPPING_STARTUP,LOLI3_ADDRESS_STARTUP,0,511,511,80,511,511,511,511,511};
Loli3RecvData				recv =		{0,1023,0,1023,0,1023,0,0};
Loli3RecvNrf  xdata	nrf =			{0};

enum { TIMER_LED_FLASH=0, TIMER_SIGNAL_COUNT, TIMER_SIGNAL_LOST, TIMER_DATA_SAVE,  TIMER_LED_OFF, TIMER_CHANNEL_LOST, TIMER_ADC, NUM_TIMERS} E_TIMER;
STimer idata timer[NUM_TIMERS];
enum {STATE_LED_OFF=0, STATE_LED_FLASH, STATE_LED_ON, STATE_LED_IDLE} E_STATE_LED;

u16 _adc_ic,_adc_batt;
u16 _tick_value;

/*////////////////////////////////////////////////////////////////////////////*/
//串口通信

#define UART_BPS_100K	0
#define UART_BPS_9600	1
#define UART_BPS_115200	2

#if defined( SBUS_BPS_115200 )
	#define UART_BPS	UART_BPS_115200
#elif	defined ( SBUS_BPS_9600 )
	#define	UART_BPS	UART_BPS_9600
#else
	#define UART_BPS	UART_BPS_100K
#endif

void UART_init(u8 bps)
{
	SCON = 0xD0;		//9位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T

	if(bps==UART_BPS_9600)	//9600bps@12.000MHz
	{
		T2L = 0xC7;
		T2H = 0xFE;
	}
	else if(bps==UART_BPS_115200)	//115200bps@12.000MHz
	{
		T2L = 0xE6;		//设置定时初始值
		T2H = 0xFF;		//设置定时初始值
	}
	else			//100000bps@12.000MHz
	{
		T2L = 0xE2;		//设定定时初值
		T2H = 0xFF;		//设定定时初值
	}

	PIN_SBUS = 1;		//上拉输出端口？有无必要？
	AUXR |= 0x10;		//定时器2开始计时
}

void UART_putch(char c)
{
		if(TI)TI=0;
		ACC=c;
		TB8=P;
		SBUF=ACC;	
}

void UART_puts(char* str)
{
	while(*str)
	{
		UART_putch(*str++);
		while(!TI);
	}
	UART_putch('\r');
	while(!TI);
	UART_putch('\n');
	while(!TI);
}

#ifdef SBUS_DEBUG
	#define DBG(str) UART_puts(str)
#else
	#define DBG(str)
#endif

/*////////////////////////////////////////////////////////////////////////////*/

void NRF_read(void)
{
	FIFO_read(nrf.rx, 11);
}

void NRF_write(void)
{
	FIFO_write(nrf.tx,11);
}

/*////////////////////////////////////////////////////////////////////////////*/

void timer_cbRegist(u8 index, void (*callback)(void))
{
//	timer[index].isRuning = 0;
//	timer[index].isTimeout = 0;
//	timer[index].count = 0;
//	timer[index].reload = 0;
	timer[index].callback = callback;
}

void timer_startOnce(u8 index, u16 ms)
{
	timer[index].isRuning = 0;	// stop the timer to prevent exception
	timer[index].isTimeout = 0;
	timer[index].count = ms;
	timer[index].reload = 0;
	timer[index].isRuning = 1;	// start the timer
}

void timer_startTimes(u8 index, u16 ms, u8 times)
{
	timer[index].isRuning = 0;	// stop the timer to prevent exception
	timer[index].isTimeout = 0;
	timer[index].count = timer[index].reload = ms;
	timer[index].isRuning = times;	// start the timer
}

void timer_startCycle(u8 index, u16 ms)
{
	timer_startTimes(index, ms, 0xff);
}

void timers_tick()	// called by ISR
{
	u8 i;
	
	for(i=0;i<NUM_TIMERS;i++)
	{
		if(timer[i].isRuning)
		{
			if(timer[i].count>TIMERS_TICK_SETPS) timer[i].count -= TIMERS_TICK_SETPS;
			else
			{
				timer[i].count = timer[i].reload;
				if(timer[i].isRuning != 0xFF) timer[i].isRuning--;
				if(timer[i].reload==0) timer[i].isRuning = 0;
				timer[i].isTimeout = 1;
			}
		}
	}
}

void timers_process()
{
	u8 i;

	for(i=0;i<NUM_TIMERS;i++)
	{
		if(timer[i].isTimeout)
		{
			timer[i].isTimeout = 0;
			if(timer[i].callback) timer[i].callback();
		}
	}
	
}

/*////////////////////////////////////////////////////////////////////////////*/

#define MAGIC_RECV_SET	0xab

void DATA_read()
{
	u8 i;
	u16 sum,sum_read;
	u8* set;

	recvSet.isValid = 0;
	EEPROM_open();
	if( EEPROM_read(0,0)!= MAGIC_RECV_SET )
	{
		EEPROM_close();
		return;
	}

	sum = MAGIC_RECV_SET;
	for(i=1,set=(u8*)&recvSet+1;i<sizeof(Loli3RecvSet);i++)
	{
		*set=EEPROM_read(0,i);	
		sum+=*set++;
	}

	sum_read=EEPROM_read(0,sizeof(Loli3RecvSet));//读校验和
	sum_read<<=8;
	sum_read+=EEPROM_read(0,sizeof(Loli3RecvSet)+1);

	EEPROM_close();

	if(sum!=sum_read) return;	//比较校验和
	recvSet.isValid = MAGIC_RECV_SET;
	return;
}
void DATA_save()
{
	u8 i;
	u16 sum;
	u8* set;
	

	EEPROM_open();
	EEPROM_clean(0);
	recvSet.isValid = MAGIC_RECV_SET;
	for(i=0,sum=0,set=(u8*)&recvSet;i<sizeof(Loli3RecvSet);i++)
	{
		EEPROM_write(0,i,*set);
		sum+=*set++;
	}
	EEPROM_write(0,sizeof(Loli3RecvSet),sum>>8);
	EEPROM_write(0,sizeof(Loli3RecvSet)+1,sum);

	EEPROM_close();
}

#undef MAGIC_RECV_SET

/*////////////////////////////////////////////////////////////////////////////*/

 void SBUS_data_push()
{
	u8 i;
	u8 buff[12];
	
	buff[1] = 												 (u8)(recv.CH_data[0])<<1;
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
	for(i=1	; i<=11; i++)	//CH1~CH8
	{
		DATA_mutex_get();		//防止修改SBUS数据的时候，中断程序在端口输出错误数据	
		recv.SBUS_tx[i]=buff[i];
		DATA_mutex_release();	
	}
	for(		; i<=22; i++)	//CH9~CH16
	{
		recv.SBUS_tx[i]=0;
	}
	recv.SBUS_tx[23]=0;		//flag
	recv.SBUS_tx[24]=0;		//End
}

/*////////////////////////////////////////////////////////////////////////////*/

void callback_timer_led_flash(void)
{
	static u8 t = 0;
	
	t++;
	if(t&0x01) 	recv.stateLED = STATE_LED_ON;
	else				recv.stateLED = STATE_LED_OFF;
}
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
	static u8 rand_index = 0;
	
	if( noRF )
	{
		nrf.hopping_cnt++;
		if(nrf.hopping_cnt>5)
		{
			if(rand_index>99)rand_index=0;
			nrf.hopping_index = random[rand_index];
			rand_index++;
		}
		else
		{
			nrf.hopping_index++;
			if(nrf.hopping_index>4)nrf.hopping_index=0;
		}
		NRF_channel(recvSet.hopping[nrf.hopping_index]);	
	}		
}

void callback_timer_adc(void)
{
	static u8 i=0;
	u16 adc;
	
	adc=ADC_RES;
	adc<<=2,adc+=ADC_RESL;

	i++;
	if(i&0x01)
	{
		_adc_ic = adc + ((_adc_ic*7)>>3);	//扩大8倍
		
		P1ASF=0x01<<3;		//P1口模拟功能控制寄存器：p1.3作为模拟功能A/D使用，下次读取电池电压保存在_adc_batt里
		ADC_CONTR= 1<<7 | 0<<4 | 1<<3 | 3<<0;		//ADC控制寄存器：打开AD转换器电源/清除转换结束标志位/启动AD转换/p1.3作AD输入
	}
	else
	{
		_adc_batt = adc + ((_adc_batt*7)>>3);	//扩大8倍
		
		P1ASF=0x00;	  		//不读取P1口，而是下次读取内部参考电压保存在_adc_ic里
		ADC_CONTR=0x88;		//启动转换
	}
}



void timers_init()
{
	u8 i;
	
	for(i=0; i<NUM_TIMERS; i++)	// stop all the timers to prevent exception
	{
		timer[i].isRuning  = 0;		// prevent timers change by <timers_tick>
		timer[i].isTimeout = 0;		// prevent timers call by <timers_process>
		timer[i].count 		 = 0;
		timer[i].reload 	 = 0;
		timer[i].callback  = 0;
	}
	timer_cbRegist(TIMER_LED_FLASH, callback_timer_led_flash);
	timer_cbRegist(TIMER_SIGNAL_COUNT,callback_timer_signal_count);
	timer_cbRegist(TIMER_SIGNAL_LOST, callback_timer_signal_lost);
	timer_cbRegist(TIMER_DATA_SAVE,		callback_timer_data_save);
	timer_cbRegist(TIMER_LED_OFF,			callback_timer_led_off);
	timer_cbRegist(TIMER_CHANNEL_LOST,callback_timer_channel_lost);
	timer_cbRegist(TIMER_ADC,					callback_timer_adc);
}


/*////////////////////////////////////////////////////////////////////////////*/

void recv_reconnect()
{
	u8 i,t;
	
	NRF_power(0);
	NRF_channel(LOLI3_CHANNEL_STARTUP);
	NRF_addr_tx(recv_address_startup);
	NRF_addr_rx(recv_address_startup);
	while(noRF);
	NRF_read();
	NRF_irq_clean();

	if(nrf.rx[0]==0xa0)
	{
		for(i=0,t=1; i<5; i++,t++)
		{
			recvSet.hopping[i]=nrf.rx[t];
		}
		for(i=0; i<5; i++,t++)
		{
			recvSet.address[i]=nrf.rx[t];
		}
	}
	nrf.tx[0]='O',nrf.tx[1]='K';

	while(1)
	{
		NRF_mode_tx();		
		NRF_channel(LOLI3_CHANNEL_STARTUP);
		NRF_addr_tx(recv_address_startup);
		NRF_addr_rx(recv_address_startup);
		NRF_write();
		Delay1ms();
		
		NRF_mode_rx();
		NRF_channel(recvSet.hopping[0]);
		NRF_addr_tx(recvSet.address);
		NRF_addr_rx(recvSet.address);
		for(t=100; t--; )
		{
			Delay1ms();
			if(noRF==0)	// 接受到信号
			{
				NRF_read();
				NRF_irq_clean();
				DATA_save();
//				NRF_addr_rx(recvSet.address);
//				NRF_addr_tx(recvSet.address);
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
	
	CH5=0;CH6=1;
	Delay1ms();
	if(CH6==0)		  	//如果CH5与CH6被短接，重新对码
	{
		P3M0=1<<3;;	  //插上舵机也会使CH6为0，所以将CH5（8通道为P3.3）设为推挽
		//P3M0=1<<1;	  //插上舵机也会使CH6为0，所以将CH5（6通道为P3.1）设为推挽
		CH5=1;
		Delay1ms();
		if(CH6)		LED_flash(20), restar=1;   //如果CH6被拉高，说明5/6通道短接，启动重新对码
	}
	CH5=0;CH6=0;
	P3M0=0xff;	//将通道输出IO口P3配置为推挽模式，保证正常驱动电调与舵机
	
	while(EEPROM_test())
	{
		LED_flash(3);
		DBG("EEPROM error!");
		delay_ms(500);
	}
	DATA_read();

	while(NRF_test())
	{
		LED_flash(10);
		DBG("NRF24L01 error!");
		delay_ms(500);
	}
	NRF_init(LOLI3_CHANNEL_CONNECT, recvSet.address);

	if(restar || recvSet.isValid==0)
	{
		recv_reconnect();
	}
	
	recv.stateLED = STATE_LED_OFF;
}

void port_init()
{
	//IO口默认准双向口模式，不修改PxM0和PxM1
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
	CH7=0;
	CH8=0;

	//将AD输入口设置为高阻输入，详见STC15手册P400要求
	P1M1 |=  1<<3;	//8通道接收机使用P1.3口做AD输入口
	P1M0 &=~(1<<3);
	//P1M1=  1<<1;		//6通道接收机使用P1.1口做AD输入口
}	

void hw_pwm_ch1_en()
{
	CCAPM2=0x42;	//PCA比较/捕获寄存器：允许比较器功能，允许脉宽调节输出
}
void hw_pwm_ch1_dis()
{
	CCAPM2 = 0;		//PCA比较/捕获寄存器：禁止比较器功能，禁止脉宽调节输出
}
void hw_pwm_ch2_en()
{
	CCAPM1=0x42;
}
void hw_pwm_ch2_dis()
{
	CCAPM1 = 0;
}

void hw_init()
{
	//禁止所有中断
	IE = 0;
	IE2= 0;

	//电压测量功能初始化
	ADC_CONTR=0x80;		//ADC上电
	delay_ms(2);			//延时等待ADC模块稳定

	P1ASF=0x00;	  					//不读取P1口，而是下次读取内部参考电压保存在_adc_ic里
	ADC_CONTR=0x88;					//启动转换
	while(ADC_CONTR&0x10==0);//等待转换完成
	
	//舵量输出控制定时器中断初始化，高优先级
	IP=0x02; 			//T0中断高优先级，其它低优先级
	TMOD &= 0x0F;	//T0设置为模式0：16位自动重装定时器，当溢出时将RL_TH0和RL_TL0存放的值自动重装入TH0和TL0中
	TMOD |= 0x00;
	IE=0x02; //允许T0中断

	//硬件PWM输出功能初始化
	P_SW1 &= ~(3<<4);	//外设功能切换寄存器1：PWM引脚切换到P3.5~P3.7
	P_SW1 |= 1<<4;
	PCA_PWM2=0x03;		//CH1（P3.7）8位PWM模式
	PCA_PWM1=0x03;		//CH2（P3.6）8位PWM模式
	//PCA_PWM0=0x03;	//CH3（P3.5）8位PWM模式
	hw_pwm_ch1_dis();
	hw_pwm_ch2_dis();

	//任务定时器中断初始化
	CCON =0x00;			//PCA控制寄存器：停止PCA定时器，清除中断标志
	CL=0;CH=0;			//复位PCA计数器
	CMOD=0x00;			//PCA工作模式寄存器：空闲模式下继续工作，系统时钟/12，计数溢出中断禁止
	_tick_value = (TIMERS_TICK_SETPS*1000);	//TIMERS_TICK_SETPS ms
	CCAP0L = _tick_value;
	CCAP0H = _tick_value >> 8;
	CCAPM0 = 0x49;	//PCA模块0的比较/捕获寄存器：允许比较器功能，匹配置位CCON标志位CCF0，使能CCF0中断

	//SBUS功能初始化
	UART_init(UART_BPS);
}

void recv_output_en(void)
{
	if(recvSet.CH1_PWM) hw_pwm_ch1_en();
	if(recvSet.CH2_PWM) hw_pwm_ch2_en();

	TR0=1;					//T0启动
	CCON =0x40;			//PCA控制寄存器：启动PCA定时器，清除中断标志
	EA =1;
}


/*////////////////////////////////////////////////////////////////////////////*/

void ET0_isr()interrupt 1	using 1	//定时器0用作信号输出	
{
	static u16 temp=0;
	static u16 temp1=0;
	static u16 temp2=0;
	static u8 state = 0;
	static u8 T_h=0,T_l=0;
	
	if(recvSet.PPM)			  //输出8通道PPM，PPM信号总周期20ms，
	{	
							//每通道固定0.4ms低电平开始，0.6到1.6ms高电平结束
		state++;
		switch(state)
		{
			case 1:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[0];
							T_l=temp;T_h=temp>>8;break;				
			case 2:CH1=0;TL0=T_l;TH0=T_h;break;
			case 3:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[1];
							T_l=temp;T_h=temp>>8;break;				
			case 4:CH1=0;TL0=T_l;TH0=T_h;break;
			case 5:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[2];
							T_l=temp;T_h=temp>>8;break;				
			case 6:CH1=0;TL0=T_l;TH0=T_h;break;
			case 7:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[3];
							T_l=temp;T_h=temp>>8;break;				
			case 8:CH1=0;TL0=T_l;TH0=T_h;break;
			case 9:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[4];
							T_l=temp;T_h=temp>>8;break;				
			case 10:CH1=0;TL0=T_l;TH0=T_h;break;
			case 11:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[5];
							T_l=temp;T_h=temp>>8;break;				
			case 12:CH1=0;TL0=T_l;TH0=T_h;break;
			case 13:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[6];
							T_l=temp;T_h=temp>>8;break;				
			case 14:CH1=0;TL0=T_l;TH0=T_h;break;
			case 15:CH1=1;TL0=0x70,TH0=0xfe;temp=64935-recv.CH_data[7];
							T_l=temp;T_h=temp>>8;break;				
			case 16:CH1=0;TL0=T_l;TH0=T_h;break;
			case 17:CH1=1;TL0=0x70,TH0=0xfe;break;				
			case 18:CH1=0;TL0=0xf0,TH0=0xd8;state=0;break;
			default:state=0;
		}
	}
	else if(recvSet.SBUS)
	{
		PIN_SBUS=1;
		if(state>24)
		{
			state=0;
		}
		else
		{
			UART_putch(recv.SBUS_tx[state]);			
			state++;
		}
		if(state==25)
		{
			TL0=0xF0;TH0=0xD8;//10ms
		}
		else 
		{
			TL0=0x88;TH0=0xFF;//120us
		}
	}
	else 
	{
		state++;
		switch(state)
		{
			case 1:if(recvSet.CH1_PWM)//输出PWM
							{
								if(recv.CH_data[0]<100)
								{
									PCA_PWM2=0x03;	
								}
								else
								{
									PCA_PWM2=0x00;
								}
								CCAP2H=255-recv.CH_data[0]/4;
							}
							else
							{
								if(recvSet.CH1_SW)//输出电平信号
								{
									if(recv.CH_data[0]<500)CH1=0;
									else CH1=1;
								}
								else CH1=1;	//输出舵量
							}
									
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;	
							
			case 2:if(recvSet.CH1_PWM==0)
							{
								if(recvSet.CH1_SW==0)CH1=0;
							}
							temp1=64725-recv.CH_data[1]*27/20;
							TL0=temp1,TH0=temp1>>8;break;				
			case 3:if(recvSet.CH2_PWM)
							{
								if(recv.CH_data[1]<100)
								{
									PCA_PWM1=0x03;	
								}
								else
								{
									PCA_PWM1=0x00;
								}
								CCAP1H=255-recv.CH_data[1]/4;
							}
							else
							{
								if(recvSet.CH2_SW)
								{
									if(recv.CH_data[1]<500)CH2=0;
									else CH2=1;
								}
								else CH2=1;
							}
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 4:if(recvSet.CH2_PWM==0)
							{
								if(recvSet.CH2_SW==0)CH2=0;
							}
							temp1=64725-recv.CH_data[2]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 5:if(recvSet.CH3_SW)
							{
								if(recv.CH_data[2]<500)CH3=0;
								else CH3=1;
							}
							else CH3=1;
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 6:if(recvSet.CH3_SW==0)CH3=0;
							temp1=64725-recv.CH_data[3]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 7:if(recvSet.CH4_SW)
							{
								if(recv.CH_data[3]<500)CH4=0;
								else CH4=1;
							}
							else CH4=1;
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 8:if(recvSet.CH4_SW==0)CH4=0;
							temp1=64725-recv.CH_data[4]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 9:if(recvSet.CH5_SW)
							{
								if(recv.CH_data[4]<500)CH5=0;
								else CH5=1;
							}
							else CH5=1;
								temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 10:if(recvSet.CH5_SW==0)CH5=0;
							temp1=64725-recv.CH_data[5]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 11:if(recvSet.CH6_SW)
							{
								if(recv.CH_data[5]<500)CH6=0;
								else CH6=1;
							}
							else CH6=1;
								temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 12:if(recvSet.CH6_SW==0)CH6=0;
							temp1=64725-recv.CH_data[6]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 13:
							#ifndef SBUS_DEBUG
							if(recvSet.CH7_SW)
							{
								if(recv.CH_data[6]<500)CH7=0;
								else CH7=1;
							}
							else CH7=1;
							#endif
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 14:
							#ifndef SBUS_DEBUG
							if(recvSet.CH7_SW==0)CH7=0;
							#endif
							temp1=64725-recv.CH_data[7]*27/20;
							TL0=temp1,TH0=temp1>>8;break;
			case 15:if(recvSet.CH8_SW)
							{
								if(recv.CH_data[7]<500)CH8=0;
								else CH8=1;
							}
							else CH8=1;
							temp2=128600-temp1;
							TL0=temp2,TH0=temp2>>8;break;
			case 16:if(recvSet.CH8_SW==0)CH8=0;
							temp1=64725-recv.CH_data[0]*27/20;
							TL0=temp1,TH0=temp1>>8;state=0;break;
			default:
				state = 0;
				break;
		}	
	}
}

void PCA_isr()interrupt 7
{
	CCF0 = 0;	//清除中断标志
	_tick_value += (TIMERS_TICK_SETPS*1000);
	CCAP0L = _tick_value;
	CCAP0H = _tick_value >> 8;

	timers_tick();

}

/*////////////////////////////////////////////////////////////////////////////*/

int main()
{
	u16 buff[8];
	u8 i;

	port_init();	
	LED_on();
	delay_ms(400);//开机延时以避过电源波动

	hw_init();

//sys_init	
	timers_init();
	timer_startCycle(TIMER_ADC, 50);
	timer_startCycle(TIMER_SIGNAL_COUNT,1000);
//	timer_startCycle(TIMER_LED_FLASH,2000);

	recv_init();
	LED_off();		//点亮指示灯再关闭，表示单片机正常工作
	DBG("LOLI3 recv init OK!");
	
	NRF_channel(recvSet.hopping[0]);
	while(noRF);
	recv_output_en();
	while(1)
	{	
		timers_process();
		
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

#if 0
		
#else		
		if(noRF)
		{
		}
		else
		{	
			if(recv.stateLED ==STATE_LED_ON) recv.stateLED = STATE_LED_OFF;

			nrf.rx_cnt++;		
			nrf.hopping_cnt=0;//收到有效信号后刷新跳频器

			timer_startOnce(TIMER_SIGNAL_LOST, 2000);
			timer_startOnce(TIMER_CHANNEL_LOST, 24);	//3*7~3*8
		
			NRF_read();		//读取接收数据
			NRF_irq_clean();
			NRF_mode_tx();
			nrf.tx[0]=recv.rx_num;
			#define BGV5	260
			recv.voltage_ic=(5ul*100*8)*BGV5/_adc_ic;	 // 详见《STC15》P898说明，此处输出voltage扩大100倍，输入adc值扩大8倍，故*100*8
			nrf.tx[1]=recv.voltage_ic>>8;
			nrf.tx[2]=recv.voltage_ic;
			recv.voltage_batt=((long)_adc_batt*recv.voltage_ic*3)>>13;
			nrf.tx[3]=recv.voltage_batt>>8;
			nrf.tx[4]=recv.voltage_batt;
			NRF_write();
			Delay1ms();
			NRF_mode_rx();
						
			nrf.hopping_index++;
			if(nrf.hopping_index>4)nrf.hopping_index=0;
			NRF_channel(recvSet.hopping[nrf.hopping_index]);

			if(nrf.rx[0]==0xa2)
			{
				if(nrf.rx[1]&0x80)	{	recvSet.PPM=1;	}
				else  									{	recvSet.PPM=0;	}
				if(nrf.rx[1]&0x40)	{	recvSet.SBUS=1;			}
				else  									{	recvSet.SBUS=0;			}
				if(nrf.rx[1]&0x08)	{	recvSet.CH1_PWM=1; hw_pwm_ch1_en();	}
				else 										{	recvSet.CH1_PWM=0; hw_pwm_ch1_dis();}
				if(nrf.rx[1]&0x04)	{	recvSet.CH2_PWM=1; hw_pwm_ch2_en();	}
				else 										{	recvSet.CH2_PWM=0; hw_pwm_ch2_dis();}
				if(nrf.rx[1]&0x02)	{	recvSet.CH7_PWM=1;	}//hw_pwm_ch7_en(); }
				else 										{	recvSet.CH7_PWM=0;	}//hw_pwm_ch7_dis();}
				if(nrf.rx[2]&0x80)	{ recvSet.CH1_SW=1;		}
				else 										{ recvSet.CH1_SW=0;		}
				if(nrf.rx[2]&0x40)	{ recvSet.CH2_SW=1;		}
				else 										{ recvSet.CH2_SW=0;		}
				if(nrf.rx[2]&0x20)	{	recvSet.CH3_SW=1;		}
				else										{	recvSet.CH3_SW=0;		}
				if(nrf.rx[2]&0x10)	{	recvSet.CH4_SW=1;		}
				else										{	recvSet.CH4_SW=0;		}
				if(nrf.rx[2]&0x08)	{	recvSet.CH5_SW=1;		}
				else										{	recvSet.CH5_SW=0;		}
				if(nrf.rx[2]&0x04)	{	recvSet.CH6_SW=1;		}
				else										{	recvSet.CH6_SW=0;		}
				if(nrf.rx[2]&0x02)	{	recvSet.CH7_SW=1;		}
				else										{	recvSet.CH7_SW=0;		}
				if(nrf.rx[2]&0x01)	{	recvSet.CH8_SW=1;		}
				else										{ recvSet.CH8_SW=0;		}
				
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
		}
#endif		
	}
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
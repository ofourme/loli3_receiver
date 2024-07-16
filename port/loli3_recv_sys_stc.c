/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LOLI3_RECV_SYS_STC.C-V1.0.0 (2023.Oct.25th)
*		程序匹配硬件：STC15W / STC8H
*		硬件固定运行频率：12MHz
*
********************************************************************************
*/

#define __LOLI3_RECV_SYS_STC_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_recv_port.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

idata volatile u32 _sys_ms;					// 当前ms值
idata volatile u16 _sys_us_target;	// 中断发生目标us值

#if defined( SBUS_BPS_115200 )
	#define UART_BPS	UART_BPS_115200
#elif	defined ( SBUS_BPS_9600 )
	#define	UART_BPS	UART_BPS_9600
#else
	#define UART_BPS	UART_BPS_100K
#endif

/*////////////////////////////////////////////////////////////////////////////*/
//延时函数，仅依赖系统时钟，无需其他模块初始化
void hw_delay_1ms()		//@12.000MHz
{
	unsigned char data i, j;
	i = (unsigned char)(FOSC/1000000UL);
	j = 169;
	do
	{
		while (--j);
	} while (--i);
}

void hw_delay_ms(u8 ms)
{
	while(ms--) hw_delay_1ms();
}

/*////////////////////////////////////////////////////////////////////////////*/
void hw_io_chs_init()
{
#if STC_15W408AS
	//stc15w IO口默认准双向口模式，无需修改PxM0和PxM1
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
	CH7=0;
	CH8=0;
#elif STC_8H
	// P1+P3+P5.4 由高阻改为双向模式
	P1M0 = 0x00;
	P1M1 = 0x00;
	P3M0 = 0x00;
	P3M1 = 0x00;
	P5M0 = 0x00;
	P5M1 = 0x00;
	// STC8H手册附录T.1建议不使用的端口双向模式并输出1
	P1 = 0xFF;
	P3 = 0xFF;
	P5 = 0xFF;
	
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
	CH7=0;
	CH8=0;
	
#endif
}

void hw_io_cho_init()
{
// CH5 设为推挽输出模式
#if defined 	PIN_CH_OUT_P31
	P3M0=1<<1;
#elif defined PIN_CH_OUT_P33
	P3M0=1<<3;
#elif defined PIN_CH_OUT_P37
	P3M0=1<<7;
#else
		#err "CH out pin not set OUTPUT."
#endif
}
void hw_io_cho_clr()
{
#if defined	PIN_CH_OUT_P31
	P31 = 0;
#elif defined PIN_CH_OUT_P33
	P33 = 0;
#elif defined PIN_CH_OUT_P37
	P37 = 0;
#else
	#err "PIN_CH_OUT not CLR."
#endif
}
void hw_io_cho_set()
{
#if defined	PIN_CH_OUT_P31
	P31 = 1;
#elif defined PIN_CH_OUT_P33
	P33 = 1;
#elif defined PIN_CH_OUT_P37
	P37 = 1;
#else
	#err "PIN_CH_OUT not SET."
#endif
}

void hw_io_chi_init()
{
	// 默认已是准双向口模式，在此设置1弱上拉，且允许外部拉低到0。
#if defined	PIN_CH_IN_P32
	P32 = 1;
#elif defined PIN_CH_IN_P36
	P36 = 1;
#else
	#err "PIN_CH_IN not set INPUT."
#endif
}

u8 hw_io_chi_get()
{
#if defined	PIN_CH_IN_P32
	return (u8)(P32);
#elif defined PIN_CH_IN_P36
	return (u8)(P36);
#else
	#err "PIN_CH_IN not GET."
#endif
}

void hw_io_chs_pushpull()
{
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
//stc51 uart 口输出0将导致串口无法输出//CH7=0;//PIN_SBUS=1;
	CH8=0;

	//接下来可将通道输出IO口配置为推挽模式，保证正常驱动电调与舵机
	//不配置推挽似乎也可以？配置推挽会否影响串口传输数据？
#if STC_15W408AS
	// P3口全部作为通道输出
	P3M0=0xFF;
#elif STC_8H
	CH7=0;
	// P54-P17-P10-P37.6.4.3.0+P31(CHx)
	P5M0=0x10;
	P1M0=0x81;
	P3M0=0xD9+0x02;
#endif
}

/*////////////////////////////////////////////////////////////////////////////*/
// 串口通讯，使用定时器2
void hw_uart_init(u8 bps)
{
	SCON = 0xD0;		//9位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T

	if(bps==UART_BPS_9600)	//9600bps@12.000MHz
	{
//		T2L = 0xC7;
//		T2H = 0xFE;
		T2L = (65536-FOSC/4/9600);
		T2H = (65536-FOSC/4/9600)>>8;
	}
	else if(bps==UART_BPS_115200)	//115200bps@12.000MHz
	{
//		T2L = 0xE6;		//设置定时初始值@12.000MHz
//		T2H = 0xFF;		//设置定时初始值@12.000MHz
		T2L = (65536-FOSC/4/115200);
		T2H = (65536-FOSC/4/115200)>>8;
	}
	else			//100000bps@12.000MHz
	{
//		T2L = 0xE2;		//设定定时初值@12.000MHz
//		T2H = 0xFF;		//设定定时初值@12.000MHz
		T2L = (65536-FOSC/4/100000);
		T2H = (65536-FOSC/4/100000)>>8;
	}

	PIN_SBUS_SET();
	AUXR |= 0x10;		//定时器2开始计时
}

void UART_pushByte(char c)
{
		if(TI)TI=0;
		ACC=c;
		TB8=P;
		SBUF=ACC;
}

void hw_uart_puts(const char* str)
{
	while(*str)
	{
		UART_pushByte(*str++);
		while(!TI);	// bugfix(20211220): 将此句移到UART_pushByte()后面，防止死循环。副作用是函数外部之前使用UART_pushByte()可能未发送完成就被终止。
	}
	UART_pushByte('\r');
	while(!TI);
	UART_pushByte('\n');
	while(!TI);
}

/*////////////////////////////////////////////////////////////////////////////*/
//电压测量模块：接收机程序对ADC返回数据按10位处理，移植不同硬件需注意保持一致
//电压测量功能初始化
void hw_adc_init()
{
	//将AD输入口设置为高阻输入，详见STC15手册P400要求
#if defined 	PIN_ADC_P1X
	P1M1 |=  1<<PIN_ADC_P1X;	//8通道接收机使用P1.x口做ADC输入口
#else
		#err "ADC pin not inited."
#endif

#if STC_15W408AS
	ADC_CONTR=0x80;		//ADC上电
	hw_delay_ms(2);		//延时等待ADC模块稳定

	P1ASF=0x00;	  		//不读取P1口，而是下次读取内部参考电压保存在_adc_ic里
	ADC_CONTR=0x88;		//启动转换
	while(ADC_CONTR&0x10==0);//等待转换完成
#elif STC_8H
	// ADC控制寄存器ADC_CONTR配置位：打开AD转换器电源/启动AD转换/转换结束标志位
	#define	ADC_POWER 7
	#define	ADC_START 6
	#define	ADC_FLAG  5
	ADCTIM = 0x3F;
	ADC_CONTR = (1<<ADC_POWER | 0<<ADC_START | 0<<ADC_FLAG);
	hw_delay_ms(2);		//延时等待ADC模块稳定

	ADC_CONTR = (1<<ADC_POWER | 1<<ADC_START | 0<<ADC_FLAG)|15;	// 读取内部参考电压保存在_adc_ic里
	while (!(ADC_CONTR & (1<<ADC_FLAG)));	////等待转换完成
#endif
}
//AD转换预启动
void hw_adc_open(u8 i)	//0：内部参考电压；1：外部电压
{
#if STC_15W408AS
	if(i&0x01)
	{
#if defined 	PIN_ADC_P1X
		P1ASF=0x01<<PIN_ADC_P1X;		//P1口模拟功能控制寄存器：p1.x作为模拟功能A/D使用，下次读取电池电压保存在_adc_batt里
		ADC_CONTR= 1<<7 | 0<<4 | 1<<3 | PIN_ADC_P1X<<0;		//ADC控制寄存器：打开AD转换器电源/清除转换结束标志位/启动AD转换/p1.x作AD输入
#else
		#err "ADC pin not P1.x"
#endif
	}
	else
	{
		P1ASF=0x00;	  		//不读取P1口，而是下次读取内部参考电压保存在_adc_ic里
		ADC_CONTR=0x88;		//启动转换
	}
#elif STC_8H
	if(i&0x01)
	{
#if defined 	PIN_ADC_P1X
		ADC_CONTR = (1<<ADC_POWER | 1<<ADC_START | 0<<ADC_FLAG) | PIN_ADC_P1X ;
#else
		#err "ADC pin not P1.x"
#endif
	}
	else
	{
		ADC_CONTR = (1<<ADC_POWER | 1<<ADC_START | 0<<ADC_FLAG) | 15 ;
	}
#endif
}

//读取上一次启动AD转换获得的值。
//本函数实现没有判断转换是否完成，使用时需注意时间间隔。
u16 hw_adc_read()
{
	u16 adc;

#if STC_15W408AS
	// ADC_RES[7:0] & ADC_RESL[1:0] 组成10位结果	// 20230813
	adc=ADC_RES;
	adc<<=2,adc|=ADC_RESL;
#elif STC_8H
	adc = ((u16)ADC_RES<<2)|(ADC_RESL>>6);
#endif	
	return adc;
}

/*////////////////////////////////////////////////////////////////////////////*/

void hw_tick_init()
{
#if STC_15W408AS
	//任务定时器中断初始化（使用PCA定时器模块0中断）
	_sys_ms = 0;
	CCON =0x00;			//PCA控制寄存器：停止PCA定时器，清除中断标志
	CL=0;CH=0;			//复位PCA计数器
	CMOD=0x00;			//PCA工作模式寄存器：空闲模式下继续工作，系统时钟/12，计数溢出中断禁止
	_sys_us_target = 1000;	//1 ms
	CCAP0L = _sys_us_target;
	CCAP0H = _sys_us_target >> 8;
	CCAPM0 = 0x49;	//PCA模块0的比较/捕获寄存器：允许比较器功能，匹配置位CCON标志位CCF0，使能CCF0中断

#elif STC_8H
	//任务定时器中断初始化（使用定时器1中断）
	// 系统计时变量初始化
	_sys_ms = 0;
	_sys_us_target = 0;	// 在STC8H的实现中，用于缓存us计数
	//T1设置为模式0：16位自动重装定时器，当溢出时将RL_TH1和RL_TL1存放的值自动重装入TH1和TL1中
	TMOD &= 0x0F;	// 高4位清零
	TMOD |= 0x00;	// 虚拟赋值，高4位赋值0
	AUXR &= ~(0x40);//默认12T模式
	IE=0x08; //允许T1中断
	TH1 = (0x10000-FOSC/12/1000000*1000)>>8;
	TL1 = (0x10000-FOSC/12/1000000*1000);

#endif
}

#if FUNC_MILLIS_EN
u32 millis(void)
{
	u32 temp;

#if STC_15W408AS
	CCAPM0 = 0x48;	// bugfix(20230902): disable CCF0 interrupt, not disable T0 interrupt
	temp = _sys_ms;
	CCAPM0 = 0x49;	//enable PCA interrupt.
#elif STC_8H
	ET1 = 0;
	temp = _sys_ms;
	ET1 = 1;
#endif
	return temp;
}
#endif

#if FUNC_MICROS_EN
u16 micros(void)
{
#if STC_15W408AS

	u8 H0,H1,L;

	H0 = CH;	// read PCA clock time.
	L	 = CL;
	H1 = CH;
	if(H1!=H0) L=0;

	return ((u16)H1<<8) | L;

#elif STC_8H

	u8 H0,H1,L;
	u16 T16, Tus;

	ET1 = 0;
	Tus = _sys_us_target;
	H0 = TH1;	// read T1 clock time.
	L	 = TL1;
	H1 = TH1;
	ET1 = 1;

	if(H1!=H0) L=0;
	T16 = (((u16)H1<<8)|L) - (0x10000-FOSC/12/1000000*1000);

	return (Tus+T16);
	
#endif
}
#endif

#if FUNC_DELAY_EN
void delay(u16 i)
{
	#if 1
	while(i--)
	hw_delay_1ms();
	#else
	
	#endif
}
#endif

#if FUNC_DELAYMICROSECONDS_EN
void delayMicroseconds(u16 us)
{
	//v20220107: cur-start<us <<-->> (cur+0x10000)-start<us
	u16 start = micros();
	while(micros()-start<us);
}
#endif

/*////////////////////////////////////////////////////////////////////////////*/
//通道舵量输出/pwm输出功能初始化
void hw_output_init()
{
#if 1
//舵量输出控制定时器中断初始化，高优先级（使用定时器0）
	IP=0x02; 			//T0中断高优先级，其它低优先级
	//T0设置为模式0：16位自动重装定时器，当溢出时将RL_TH0和RL_TL0存放的值自动重装入TH0和TL0中
	TMOD &= 0xF0;	//bugfix(20230812): 0x0F改为0xF0
	TMOD |= 0x00;
	ET0 = 1; //允许T0中断
#endif

#if STC_15W408AS
	//硬件PWM输出功能初始化（使用PCA定时器模块2/1的8位pwm功能，PCA输入时钟频率1MHz，pwm频率=1MHz/256~=4k）
	P_SW1 &= ~(3<<4);	//外设功能切换寄存器1：PWM引脚切换到P3.5~P3.7
	P_SW1 |= 1<<4;
	PCA_PWM2=0x03;		//（P3.7）8位PWM模式
	PCA_PWM1=0x03;		//（P3.6）8位PWM模式
	//PCA_PWM0=0x03;	//（P3.5）8位PWM模式
	hw_pwm_dis(0);
	hw_pwm_dis(1);
#elif STC_8H
	//硬件CH1/CH2/CH6/CH7对应PWM6_2/PWM5_2/PWM8_2/PWM7_2
	//stc8h PWM输出使能
	#define ENO5P       			0x01
	#define ENO6P       			0x04
	#define ENO7P       			0x10
	#define ENO8P       			0x40
	//pwm 分辨率、频率 寄存器配置值计算
	#define ANALOG_PWM_ARR_8BITS	(256-2)
	//#define ANALOG_PWM_ARR_10BITS	(1024-2)
	#define ANALOG_PWM_RANGE			(ANALOG_PWM_ARR_8BITS+1)
	#define ANALOG_PWM_PSCR_1KHZ	((FOSC/ANALOG_PWM_RANGE/1000)-1)
	//stc8h PWM引脚选择
	#define	PWM5_SW_P17				1
	#define	PWM6_SW_P54				(1<<2)
	#define	PWM7_SW_P33				(1<<4)
	#define	PWM8_SW_P34				(1<<6)

	PWMB_BKR = 0x00; 					//关闭主输出，寄存器无写保护
	PWMB_CR2 = 0x00;	
	PWMB_CR1 = 0x00;					//关闭计数器
	PWMB_CCER1 = 0x00;  			//关闭通道：PWM5/PWM6
	PWMB_CCER2 = 0x00;  			//关闭通道：PWM7/PWM8
	PWMB_CCMR1 = 0x60|0x08;		//设置通道5模式：PWM模式1 | PWMn_CCRx预装载使能 + 输出
	PWMB_CCMR2 = 0x60|0x08;		//设置通道6模式：
	PWMB_CCMR3 = 0x60|0x08;		//设置通道7模式：
	PWMB_CCMR4 = 0x60|0x08;		//设置通道8模式：
	PWMB_CCER1 = 0x11;				//开启通道：PWM5/PWM6
	PWMB_CCER2 = 0x11;				//开启通道：PWM7/PWM8
	PWMB_CCR5 = 0;						//设置占空比时间
	PWMB_CCR6 = 0;						//设置占空比时间
	PWMB_CCR7 = 0;						//设置占空比时间
	PWMB_CCR8 = 0;						//设置占空比时间
	//设置周期时间，必须先写高字节PWMB_ARRH；向上计数范围为[0~ARR]
	PWMB_ARR = ANALOG_PWM_ARR_8BITS;;
	//对系统时钟分频，必须先写高字节PWMB_PSCRH；PWMB_PSCR对系统时钟分频得到PWMB时钟，PWMB_PSCR+1个系统时钟输出一个PWMB时钟
	PWMB_PSCR = ANALOG_PWM_PSCR_1KHZ;
	PWMB_PS = PWM5_SW_P17 | PWM6_SW_P54	| PWM7_SW_P33	| PWM8_SW_P34; //高级 PWM 通道输出脚选择位
	PWMB_DTR = 0;							//死区时间
	PWMB_RCR = 0;							//重复计数器
	PWMB_ENO = 0;  						//PWMB所有端口禁止输出 //	hw_pwm_dis(0/1/5/6);
	PWMB_BKR = 0x80; 					//使能主输出
	PWMB_EGR = 0x01;					//重新初始化计数器，并产生一个更新事件。
	PWMB_CR1 = 0x01;					//ARR不缓冲，向上计数模式，开始向上计数

//PWMB_BKR = 0x81; 					//使能主输出+写保护。经测试，设置写保护后无法输出pwm，原因未明。

#endif
}

#if STC_15W408AS

void hw_pwm_en(u8 ch)
{
	if(ch==0)				CCAPM2=0x42;	//PCA比较/捕获寄存器：允许比较器功能，允许脉宽调节输出
	else if(ch==1)	CCAPM1=0x42;
}

void hw_pwm_dis(u8 ch)
{
	if(ch==0) 			CCAPM2 = 0;		//PCA比较/捕获寄存器：禁止比较器功能，禁止脉宽调节输出
	else if(ch==1)	CCAPM1 = 0;
}

#elif STC_8H

void hw_pwm_en(u8 ch)
{
	if(ch==0)				PWMB_ENO |= ENO6P;
	else if(ch==1)	PWMB_ENO |= ENO5P;
	else if(ch==5)	PWMB_ENO |= ENO8P;
	else if(ch==6)	PWMB_ENO |= ENO7P;
}

void hw_pwm_dis(u8 ch)
{
	if(ch==0)				PWMB_ENO &= ~ENO6P;
	else if(ch==1)	PWMB_ENO &= ~ENO5P;
	else if(ch==5)	PWMB_ENO &= ~ENO8P;
	else if(ch==6)	PWMB_ENO &= ~ENO7P;
}

#endif

// 设置pragma NOAREGS，寄存器组不使用绝对地址，以便在使用using的中断函数中调用
#pragma NOAREGS

// ch[0,7], pwm[0,255]
void hw_pwm_set(u8 ch, u8 pwm)
{
#if STC_15W408AS
/*-----------------------------------------------------------------------------/
比较值：{EPCnL,CCAPnL[7:0]}
当前值：{0,CL[7:0]}，由CCP计数器低8位CL与0组成9位数据，数据范围[0,255]

8位pwm模式划分256精度，根据STC15手册P942，当前值大于等于比较值时输出1 --> 比较值==低电平数量
当CCAPnL==255-pwm --> pwm[0-255]==CCAPnL[255-0]==低电平[255/256-0/256]==高电平[1/256-256/256]
为输出全低电平，当pwm<MININUM时做特殊处理，CCAPnL与EPCnL(此时为1)组成9位比较值，以提供全0输出。
/----------------------------------------------------------------------------*/

// stc15w HW_PWM_MINNUM最低值需为1才能保证输出全低电平。以下宏对HW_PWM_MINNUM进行处理。
#if (LOLI3_RECV_PWM_MINNUM/4 >= 1)
	#define HW_PWM_MINNUM		(LOLI3_RECV_PWM_MINNUM/4)
#else
	#define HW_PWM_MINNUM		1
#endif

	if(ch==0)
	{
		if( pwm>=HW_PWM_MINNUM )	PCA_PWM2=0x00;	// CCAPxH&CCAPnL第9位数为0
		else											PCA_PWM2=0x03;	// CCAPxH&CCAPnL第9位数为1
		CCAP2H = 255-pwm;					// CL溢出时EPCnH&CCAPnH数据才载入EPCnL&CCAPnL
	}
	else if(ch==1)
	{
		if( pwm>=HW_PWM_MINNUM )	PCA_PWM1=0x00;
		else											PCA_PWM1=0x03;
		CCAP1H = 255-pwm;
	}
#elif STC_8H
/*-----------------------------------------------------------------------------/

硬件CH1/CH2/CH6/CH7对应PWM6_2/PWM5_2/PWM8_2/PWM7_2
	
/----------------------------------------------------------------------------*/
#if (LOLI3_RECV_PWM_MINNUM/4 >= 1)
	#define HW_PWM_MINNUM		(LOLI3_RECV_PWM_MINNUM/4)
#else
	#define HW_PWM_MINNUM		1
#endif
	
	if(ch==0)
	{
		if( pwm>=HW_PWM_MINNUM )	PWMB_CCR6 = pwm;
		else											PWMB_CCR6 = 0;
	}
	else if(ch==1)
	{
		if( pwm>=HW_PWM_MINNUM )	PWMB_CCR5 = pwm;
		else											PWMB_CCR5 = 0;
	}
	else if(ch==5)
	{
		if( pwm>=HW_PWM_MINNUM )	PWMB_CCR8 = pwm;
		else											PWMB_CCR8 = 0;
	}
	else if(ch==6)
	{
		if( pwm>=HW_PWM_MINNUM )	PWMB_CCR7 = pwm;
		else											PWMB_CCR7 = 0;
	}
	
/*-----------------------------------------------------------------------------/
/----------------------------------------------------------------------------*/
#endif
}
#pragma AREGS

/*////////////////////////////////////////////////////////////////////////////*/

void LED_on(void)
{
	LED_ON();
}

void LED_off(void)
{
	LED_OFF();
}

/*////////////////////////////////////////////////////////////////////////////*/
// NRF24L01 SPI端口IO模式初始化
void SPI_init_0()
{
	// set SCK/MOSI/NSS output mode, MISO input mode
	//默认准双向IO，无需配置代码
}

void SPI_init_1(void)
{
	// then set SCK/MOSI 0, set NNS 1.
 	SPI_SLAVE_DIS();
	SPI_SCK_CLR();
	SPI_MOSI_CLR();
}

/*////////////////////////////////////////////////////////////////////////////*/
//ROM存储
u8 EEPROM_read(u16 addr)	//读数据
{
	IAP_CMD=0x01;

	IAP_ADDRH=addr>>8;
	IAP_ADDRL=addr;

	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
	_nop_();

	return IAP_DATA;
}

void EEPROM_write(u16 addr,u8 byte)	//写入数据
{
	IAP_CMD=0x02;
	IAP_DATA=byte;

	IAP_ADDRH=addr>>8;
	IAP_ADDRL=addr;

	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
	_nop_();
}

void EEPROM_cleanPage(u8 addrPage)	//擦除扇区数据
{
	IAP_CMD=0x03;

	IAP_ADDRH=(SIZE_EEPROM_PAGE*addrPage)>>8;
	IAP_ADDRL=0;

	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
	_nop_();
}

/*////////////////////////////////////////////////////////////////////////////*/

void hw_sys_init()
{
	//禁止所有中断
	IE = 0;
	IE2= 0;
#if STC_8H
	EAXSFR();	//扩展SFR
#endif
	
	//端口功能初始化
  hw_io_chs_init();
	//开机延时以避过电源波动
  hw_delay_ms(200);	// 参数类型u8，不大于255。
  hw_delay_ms(200);
	//SBUS功能初始化-------------------（使用定时器2）
	hw_uart_init(UART_BPS);	//init for debug and SBUS output
	//电压测量功能初始化
	hw_adc_init();
	//系统心跳功能初始化---------------（STC15W使用PCA定时器模块0中断 / STC8H使用定时器1）
	hw_tick_init();
	//通道输出功能初始化---------------（使用定时器0）
	hw_output_init();
	
	// addin(20240515): 后续使用delay的实现可能依赖tick中断，故打开tick中断；或者把delay替换成hw_delay_ms?
	hw_tick_en();
  INTERRUPT_en();
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

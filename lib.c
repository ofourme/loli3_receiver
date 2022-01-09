/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LIB.C-V0.6.0 (2021.Dec.26th)
*
********************************************************************************
*/

#define __LIB_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "lib.h"
#include "app.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

static idata u32 _sys_ms;
static idata u16 _sys_us;

extern Loli3RecvSet  xdata recvSet;
extern Loli3RecvData recv;
extern STimer idata timer[];

/*////////////////////////////////////////////////////////////////////////////*/
//串口通信
#if defined( SBUS_BPS_115200 )
	#define UART_BPS	UART_BPS_115200
#elif	defined ( SBUS_BPS_9600 )
	#define	UART_BPS	UART_BPS_9600
#else
	#define UART_BPS	UART_BPS_100K
#endif

/*////////////////////////////////////////////////////////////////////////////*/

void port_init()
{
	//51 IO口默认准双向口模式，无需修改PxM0和PxM1
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

void port_ch5_mode_OUTPUT()
{
		P3M0=1<<3;;	  //CH5（8通道为P3.3）设为推挽
		//P3M0=1<<1;	//CH5（6通道为P3.1）设为推挽
}
void port_ch5_clr()
{
	CH5 = 0;
}
void port_ch5_set()
{
	CH5 = 1;
}

void port_ch6_mode_INPUT_PULLUP()
{
	CH6=1;	// 默认准双向口模式，设置1上拉？
}

u8   port_ch6_get()
{
	return (u8)(CH6);
}

void port_chs_mode_OUTPUT()
{
	CH1=0;
	CH2=0;
	CH3=0;
	CH4=0;
	CH5=0;
	CH6=0;
//stc51 uart 口输出0将导致串口无法输出//CH7=0;//PIN_SBUS=1;
	CH8=0;
	P3M0=0xff;	//将通道输出IO口P3配置为推挽模式，保证正常驱动电调与舵机
}

/*////////////////////////////////////////////////////////////////////////////*/

void pwm_en(u8 ch)
{
	if(ch==1) 		CCAPM2=0x42;	//PCA比较/捕获寄存器：允许比较器功能，允许脉宽调节输出
	else if(ch==2)CCAPM1=0x42;
}

void pwm_dis(u8 ch)
{
	if(ch==1) 		CCAPM2 = 0;		//PCA比较/捕获寄存器：禁止比较器功能，禁止脉宽调节输出
	else if(ch==2)CCAPM1 = 0;
}

void pwm_set(u8 ch, u8 pwm)
{
	if(ch==1)
	{
		if(pwm>20) PCA_PWM2=0x00;	// CCAPxH/CCAPxL第9位数为0 //pwm>x，当x过小，无法关闭LED灯pwm输出，原因未明
		else		PCA_PWM2=0x03;	// CCAPxH/CCAPxL第9位数为1
		CCAP2H=255-pwm;					// CL溢出时CCAPxH[0~8]数据载入CCAPxL[0~8]
	}
	else if(ch==2)
	{
		if(pwm>20) PCA_PWM1=0x00;
		else		PCA_PWM1=0x03;
		CCAP1H=255-pwm;
	}
}

/*////////////////////////////////////////////////////////////////////////////*/

void hw_init()
{
	//禁止所有中断
	IE = 0;
	IE2= 0;

	//电压测量功能初始化
	ADC_CONTR=0x80;		//ADC上电
	delay(2);			//延时等待ADC模块稳定

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
	pwm_dis(1);
	pwm_dis(2);

	//任务定时器中断初始化
	_sys_ms = 0;
	CCON =0x00;			//PCA控制寄存器：停止PCA定时器，清除中断标志
	CL=0;CH=0;			//复位PCA计数器
	CMOD=0x00;			//PCA工作模式寄存器：空闲模式下继续工作，系统时钟/12，计数溢出中断禁止
	_sys_us = 1000;	//1 ms
	CCAP0L = _sys_us;
	CCAP0H = _sys_us >> 8;
	CCAPM0 = 0x49;	//PCA模块0的比较/捕获寄存器：允许比较器功能，匹配置位CCON标志位CCF0，使能CCF0中断

	//SBUS功能初始化
	UART_init(UART_BPS);
}

/*////////////////////////////////////////////////////////////////////////////*/

u16 adc_read()
{
	u16 adc;
	
	adc=ADC_RES;
	adc<<=2,adc+=ADC_RESL;

	return adc;
}

void adc_open(u8 i)	//0：内部参考电压；1：外部电压
{
	if(i&0x01)
	{
		P1ASF=0x01<<3;		//P1口模拟功能控制寄存器：p1.3作为模拟功能A/D使用，下次读取电池电压保存在_adc_batt里
		ADC_CONTR= 1<<7 | 0<<4 | 1<<3 | 3<<0;		//ADC控制寄存器：打开AD转换器电源/清除转换结束标志位/启动AD转换/p1.3作AD输入
	}
	else
	{
		P1ASF=0x00;	  		//不读取P1口，而是下次读取内部参考电压保存在_adc_ic里
		ADC_CONTR=0x88;		//启动转换
	}
}


/*////////////////////////////////////////////////////////////////////////////*/
// CHx output
void ET0_isr()interrupt 1	using 1	//定时器0用作信号输出
{
	static u16 temp=0;
	static u16 temp1=0;
	static u16 temp2=0;
	static u8 state = 0;
	static u8 T_h=0,T_l=0;

/*	
	if(recv.bModeChange)
	{
		recv.bModeChange = 0;
		state = 0;
	}
*/	
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
			default:CH1=0;TL0=0xf0,TH0=0xd8;state=0;break;
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
			UART_pushByte(recv.SBUS_tx[state]);
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
			case 1:
					if(recvSet.CH1_PWM)//输出PWM
					{
						pwm_set(1,recv.CH_data[0]/4);
					}
					else if(recvSet.CH1_SW)//输出电平信号
					{
						if(recv.CH_data[0]<500) CH1=0;
						else	CH1=1;
					}
					else
					{
						CH1=1;	//输出舵量
					}
					temp2=128600-temp1;
					TL0=temp2,TH0=temp2>>8;
					break;
 
			case 2:
					if(recvSet.CH1_PWM==0 && recvSet.CH1_SW==0)	//输出舵量
					{
						CH1=0;
					}
					temp1=64725-recv.CH_data[1]*27/20;
					TL0=temp1,TH0=temp1>>8;
					break;

			case 3:
					if(recvSet.CH2_PWM)
					{
						pwm_set(2,recv.CH_data[1]/4);
					}
					else if(recvSet.CH2_SW)
					{
							if(recv.CH_data[1]<500)CH2=0;
							else CH2=1;
					}
					else CH2=1;
					temp2=128600-temp1;
					TL0=temp2,TH0=temp2>>8;
					break;
					
			case 4:
					if(recvSet.CH2_PWM==0 && recvSet.CH2_SW==0)
					{
							CH2=0;
					}
					temp1=64725-recv.CH_data[2]*27/20;
					TL0=temp1,TH0=temp1>>8;
					break;
				
			case 5:
					if(recvSet.CH3_SW)
					{
						if(recv.CH_data[2]<500)CH3=0;
						else CH3=1;
					}
					else CH3=1;
					temp2=128600-temp1;
					TL0=temp2,TH0=temp2>>8;
					break;
				
			case 6:
					if(recvSet.CH3_SW==0)CH3=0;
					temp1=64725-recv.CH_data[3]*27/20;
					TL0=temp1,TH0=temp1>>8;
					break;
			
			case 7:
				if(recvSet.CH4_SW)
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
			default:if(recvSet.CH8_SW==0)CH8=0;
							temp1=64725-recv.CH_data[0]*27/20;
							TL0=temp1,TH0=temp1>>8;state=0;break;
		}
	}
}

/*////////////////////////////////////////////////////////////////////////////*/
// 1ms tick
void isr_PCA() interrupt 7
{
	CCF0 = 0;	//清除中断标志
	_sys_us += 1000;
	_sys_ms ++;
	CCAP0L = _sys_us;
	CCAP0H = _sys_us >> 8;

	timer_tick();
}

/*////////////////////////////////////////////////////////////////////////////*/
//延时函数
void delay_1_ms()		//@12.000MHz
{
#if 1
	unsigned char i, j;
	i = 12;
	j = 168;
	do
	{
		while (--j);
	} while (--i);
#else
	delayMicroseconds(1000);	
#endif
}

void delay(u16 i)
{
	while(i--)
	delay_1_ms();
}

#if 1
u16 micros(void)
{
	u8 H0,H1,L;
	
	H0 = CH;
	L	 = CL;
	H1 = CH;
	
	if(H1!=H0) L=0;
	
	return ((u16)H1<<8) | L;
}

#if 0
u32 millis(void)
{
	u32 temp;

	ET0 = 0;
	temp = _sys_ms;
	ET0 = 1;

	return temp;
}
#endif

#if 0
void delayMicroseconds(u16 us)
{
	//v20220107
	u16 start = micors();
	while(micros()-start<us);
}
#endif

#endif

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

u8 EEPROM_test(void)
{
	u8 addrPage = 1;
	
	EEPROM_begin();
	
	EEPROM_cleanPage(addrPage);
	EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0x88);
	if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0x88)	{EEPROM_end();	return 1;	}//err
	EEPROM_cleanPage(addrPage);
	EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0x55);
	if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0x55)	{EEPROM_end();	return 1;	}//err

	return 0;	//ok
}


/*////////////////////////////////////////////////////////////////////////////*/
// soft SPI
#define SPI_MODE_0		(0<<0)
#define SPI_MODE_1		(1<<0)
#define SPI_MODE_2		(2<<0)
#define SPI_MODE_3		(3<<0)
#define SPI_MSBFIRST	(0<<2)
#define SPI_LSBFIRST	(1<<2)
#define SPI_SPEED_SOF	(0<<3)

u8 SPI_begin(u8 setting)
{
	if(setting!=(SPI_MODE_0|SPI_MSBFIRST)) return 0;
	
	// set SCK/MOSI/NSS output mode, MISO input mode
	// then set SCK/MOSI 0, set NNS 1.
	SPI_SLAVE_DIS();
	SPI_SCK_CLR();
	SPI_MOSI_CLR();
	return 1;
}

u8 SPI_transfer(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(byte&0x80)
		{
			SPI_MOSI_SET();
		}
		else
		{
			SPI_MOSI_CLR();
		}
		SPI_SCK_SET();
		byte<<=1;
		byte|=MISO;
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

void NRF_REG_write(u8 address,u8 command)
{
	SPI_slaveEnable();
	SPI_transfer(0x20+address);
	SPI_transfer(command);
	SPI_slaveDisable();
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

void  NRF_power(u8 P)				//发射功率设置 250k
{														
	NRF_CE_CLR();
	if(P==3)		 NRF_REG_write(0x06,0x27);		  //0db 修正之前注释错误
	else if(P==2)NRF_REG_write(0x06,0x25);	  //-6db
	else if(P==1)NRF_REG_write(0x06,0x23);	  //-12db
	else if(P==0)NRF_REG_write(0x06,0x21);    //-18db
	NRF_CE_SET();
}

void NRF_size(u8 l)
{
	NRF_CE_CLR();
	NRF_REG_write(0x11,l);  
	NRF_CE_SET();
}

void NRF_channel(u8 c)
{
	NRF_CE_CLR();
	NRF_REG_write(0x05,c);  
	NRF_CE_SET();
}

void NRF_init_0(void)
{
	NRF_CE_CLR();
	//	NRF_REG_write(0x00,0x0a); //CRC使能，上电
	NRF_REG_write(0x01,0x00); //禁止 自动应答
	NRF_REG_write(0x02,0x01); //允许 P0信道
	NRF_REG_write(0x04,0x00); //禁止 自动重发
	NRF_CE_SET();
}

void NRF_irq_clean(void)
{
		NRF_CE_CLR();
		//NRF_REG_write(0x07,0x40);	//清除无线模块接收数据中断信号
		NRF_REG_write(0x07,0x70);	//清除无线模块（重发失败+发送完成+数据接受）中断信号

		NRF_CE_SET();
}

void NRF_init(void)	//u8 ch,u8 address[])
{
	
	SPI_begin(0);
	
	NRF_init_0();
	NRF_mode_rx();
//	NRF_channel(ch);
	NRF_power(0);
	NRF_size(LOLI3_NRF_DATA_LENGTH);
//	NRF_addr_rx(address);
//	NRF_addr_tx(address);
}

#define NRF_BAD_CSNCLK	0x01
#define NRF_BAD_MOSI		0x02
#define NRF_BAD_CE			0x04
#define NRF_BAD_IRQ			0x08
#define NRF_BAD_MISO		0x10


u8 NRF_test()	//无线模块终极测试
{	
	u8 reset_err=0;
	u8 NRF_error = 0;
	u8 tx[1] = {'T'};	// test?

	SPI_begin(0);
	NRF_CE_CLR();

	SPI_slaveEnable();
	if(SPI_transfer(0x20)!=0x0e){reset_err=1;}
	SPI_transfer(0x0a);
	SPI_slaveDisable();
	
	SPI_slaveEnable();
	SPI_transfer(0x00);
	if(SPI_transfer(0x00)!=0x0a){NRF_error|=NRF_BAD_MOSI;}//MOSI bad
	SPI_slaveDisable();

	NRF_REG_write(0x01,0x00);
	NRF_REG_write(0x04,0x00);
	NRF_REG_write(0x11,1);
	NRF_FIFO_write(tx,1);

	NRF_CE_SET();

	delay(2);
	
	SPI_slaveEnable();
	if(SPI_transfer(0x00)!=0x2e){NRF_error|=NRF_BAD_CE;}//CE bad
	SPI_slaveDisable();
		

	if(noRF) NRF_error|=NRF_BAD_IRQ;	//IRQ bad. bug fix?: 0x18->0x08
	else 
	{
		if(NRF_error&0x04==0)NRF_error|=NRF_BAD_MISO;		//MISO bad
	}
	NRF_CE_SET();
	
	if(reset_err&&NRF_error>1)NRF_error|=NRF_BAD_CSNCLK;//CSN,CLK bad
	
	NRF_irq_clean();
	
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
		delay(50);
		LED_OFF();		
		delay(50);
		t--;
	}
}

/*////////////////////////////////////////////////////////////////////////////*/

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

void UART_pushByte(u8 c)
{
		if(TI)TI=0;
		ACC=c;
		TB8=P;
		SBUF=ACC;
}

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

void UART_puts(char* str)
{
	while(*str)
	{
		UART_pushByte(*str++);
		while(!TI);	// bugfix:20211220: 将此句移到UART_pushByte()后面，防止死循环。副作用是函数外部之前使用UART_pushByte()可能未发送完成就被终止。
	}
	UART_pushByte('\r');
	while(!TI);
	UART_pushByte('\n');
	while(!TI);
}

/*////////////////////////////////////////////////////////////////////////////*/

static STimer* _pTimers;
static u8	_numTimers;

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
//	_pTimers[index].isRuning = 0;
//	_pTimers[index].isTimeout = 0;
//	_pTimers[index].count = 0;
//	_pTimers[index].reload = 0;
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
	_pTimers[index].count = timer[index].reload = ms;
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
	if(step < TIMERS_TICK_SETPS) return;
	step = 0;

	for(i=0;i<_numTimers;i++)
	{
		if(_pTimers[i].isRuning)
		{
			if(_pTimers[i].count>TIMERS_TICK_SETPS) _pTimers[i].count -= TIMERS_TICK_SETPS;
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
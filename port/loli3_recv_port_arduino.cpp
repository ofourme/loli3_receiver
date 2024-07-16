/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   LIB_ARDUINO.CPP-V0.6.0 (2021.Dec.26th)
*
********************************************************************************
*/

#define __LIB_ARDUINO_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "lib.h"
#include "app.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <arduino.h>
#include <SPI.h>

static idata u32 _sys_ms;

extern Loli3RecvSet  xdata recvSet;
extern Loli3RecvData recv;

/*////////////////////////////////////////////////////////////////////////////*/
//串口通信
#if defined( SBUS_BPS_115200 )
    #define UART_BPS    UART_BPS_115200
#elif    defined ( SBUS_BPS_9600 )
    #define    UART_BPS    UART_BPS_9600
#else
    #define UART_BPS    UART_BPS_100K
#endif

/*////////////////////////////////////////////////////////////////////////////*/

void port_init()
{
}

void port_ch5_mode_OUTPUT()
{
}
void port_ch5_clr()
{
}
void port_ch5_set()
{
}

void port_ch6_mode_INPUT_PULLUP()
{
}

u8   port_ch6_get()
{
  return 1;
}

void port_chs_mode_OUTPUT()
{
}

/*////////////////////////////////////////////////////////////////////////////*/

void pwm_en(u8 ch)
{
}

void pwm_dis(u8 ch)
{
}

void pwm_set(u8 ch, u8 pwm)
{
}

/*////////////////////////////////////////////////////////////////////////////*/

void hw_init()
{
  UART_init(0);
}

/*////////////////////////////////////////////////////////////////////////////*/

u16 adc_read()
{
}

void adc_open(u8 i)    //0：内部参考电压；1：外部电压
{
}


/*////////////////////////////////////////////////////////////////////////////*/
// CHx output

/*////////////////////////////////////////////////////////////////////////////*/
// 1ms tick

/*////////////////////////////////////////////////////////////////////////////*/
//延时函数
void delay_1_ms()        //@12.000MHz
{
#if 0
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



/*////////////////////////////////////////////////////////////////////////////*/
//ROM存储
#include <EEPROM.h>

u8 EEPROM_read(u16 addr)    //读数据
{
  return EEPROM.read (addr);
}

void EEPROM_write(u16 addr,u8 byte)    //写入数据
{
    EEPROM.write(addr, byte);
}

void EEPROM_cleanPage(u8 addrPage)    //擦除扇区数据
{
  int i;
  for(i=SIZE_EEPROM_PAGE*addrPage; i<SIZE_EEPROM_PAGE*(addrPage+1);i++)
    EEPROM.write(i, 0);
}

u8 EEPROM_test(void)
{
    u8 addrPage = 1;

    EEPROM_begin();

    EEPROM_cleanPage(addrPage);
    EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0x88);
    if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0x88)    {EEPROM_end();    return 1;    }//err
    EEPROM_cleanPage(addrPage);
    EEPROM_write(SIZE_EEPROM_PAGE*addrPage,0x55);
    if(EEPROM_read(SIZE_EEPROM_PAGE*addrPage)!=0x55)    {EEPROM_end();    return 1;    }//err

    return 0;    //ok
}


/*////////////////////////////////////////////////////////////////////////////*/
// soft SPI
void SPI_init(void)
{
    pinMode(IRQ, INPUT);
    pinMode(CE, OUTPUT);
    pinMode(CSN,OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO,INPUT);
}

void SPI_init2(u8 setting)
{
 #if 1
//#ifdef DEF_SPI_SOFT
	SPI_SLAVE_DIS();
	SPI_SCK_CLR();
	SPI_MOSI_CLR();
#else
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
//    SPI.setBitOrder(MSBFIRST);
//    SPI.setDataMode(SPI_MODE0);
//    SPI.setClockDivider(SPI_CLOCK_DIV16);
#endif
}


/*////////////////////////////////////////////////////////////////////////////*/

void UART_init(u8 bps)
{
  Serial.begin(115200);
}

void UART_pushByte(u8 c)
{
}


void UART_puts(const char* str)
{
  Serial.println(str);
}


/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

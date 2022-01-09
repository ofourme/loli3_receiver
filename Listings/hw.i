
#line 1 "hw.c" /0









 
 
 
 
 
 
 
  
#line 1 "hw.h" /0









 
 
 
 
 
 
 
 
  
#line 1 "C:\bin\Keil_v5\C51\Inc\reg52.h" /0






 
 
 
 
 
 
 sfr P0    = 0x80;
 sfr P1    = 0x90;
 sfr P2    = 0xA0;
 sfr P3    = 0xB0;
 sfr PSW   = 0xD0;
 sfr ACC   = 0xE0;
 sfr B     = 0xF0;
 sfr SP    = 0x81;
 sfr DPL   = 0x82;
 sfr DPH   = 0x83;
 sfr PCON  = 0x87;
 sfr TCON  = 0x88;
 sfr TMOD  = 0x89;
 sfr TL0   = 0x8A;
 sfr TL1   = 0x8B;
 sfr TH0   = 0x8C;
 sfr TH1   = 0x8D;
 sfr IE    = 0xA8;
 sfr IP    = 0xB8;
 sfr SCON  = 0x98;
 sfr SBUF  = 0x99;
 
 
 sfr T2CON  = 0xC8;
 sfr RCAP2L = 0xCA;
 sfr RCAP2H = 0xCB;
 sfr TL2    = 0xCC;
 sfr TH2    = 0xCD;
 
 
 
 
 sbit CY    = PSW^7;
 sbit AC    = PSW^6;
 sbit F0    = PSW^5;
 sbit RS1   = PSW^4;
 sbit RS0   = PSW^3;
 sbit OV    = PSW^2;
 sbit P     = PSW^0;  
 
 
 sbit TF1   = TCON^7;
 sbit TR1   = TCON^6;
 sbit TF0   = TCON^5;
 sbit TR0   = TCON^4;
 sbit IE1   = TCON^3;
 sbit IT1   = TCON^2;
 sbit IE0   = TCON^1;
 sbit IT0   = TCON^0;
 
 
 sbit EA    = IE^7;
 sbit ET2   = IE^5;  
 sbit ES    = IE^4;
 sbit ET1   = IE^3;
 sbit EX1   = IE^2;
 sbit ET0   = IE^1;
 sbit EX0   = IE^0;
 
 
 sbit PT2   = IP^5;
 sbit PS    = IP^4;
 sbit PT1   = IP^3;
 sbit PX1   = IP^2;
 sbit PT0   = IP^1;
 sbit PX0   = IP^0;
 
 
 sbit RD    = P3^7;
 sbit WR    = P3^6;
 sbit T1    = P3^5;
 sbit T0    = P3^4;
 sbit INT1  = P3^3;
 sbit INT0  = P3^2;
 sbit TXD   = P3^1;
 sbit RXD   = P3^0;
 
 
 sbit SM0   = SCON^7;
 sbit SM1   = SCON^6;
 sbit SM2   = SCON^5;
 sbit REN   = SCON^4;
 sbit TB8   = SCON^3;
 sbit RB8   = SCON^2;
 sbit TI    = SCON^1;
 sbit RI    = SCON^0;
 
 
 sbit T2EX  = P1^1;  
 sbit T2    = P1^0;  
 
 
 sbit TF2    = T2CON^7;
 sbit EXF2   = T2CON^6;
 sbit RCLK   = T2CON^5;
 sbit TCLK   = T2CON^4;
 sbit EXEN2  = T2CON^3;
 sbit TR2    = T2CON^2;
 sbit C_T2   = T2CON^1;
 sbit CP_RL2 = T2CON^0;
 
 
#line 18 "hw.h" /0
 
  
#line 1 "C:\bin\Keil_v5\C51\Inc\intrins.h" /0






 
 
 
 
 
 #pragma SAVE
 
 
#line 15 "C:\bin\Keil_v5\C51\Inc\intrins.h" /1
 
 
 
#line 18 "C:\bin\Keil_v5\C51\Inc\intrins.h" /0
 
 extern void          _nop_     (void);
 extern bit           _testbit_ (bit);
 extern unsigned char _cror_    (unsigned char, unsigned char);
 extern unsigned int  _iror_    (unsigned int,  unsigned char);
 extern unsigned long _lror_    (unsigned long, unsigned char);
 extern unsigned char _crol_    (unsigned char, unsigned char);
 extern unsigned int  _irol_    (unsigned int,  unsigned char);
 extern unsigned long _lrol_    (unsigned long, unsigned char);
 extern unsigned char _chkfloat_(float);
 
#line 29 "C:\bin\Keil_v5\C51\Inc\intrins.h" /1
 
 
 
#line 32 "C:\bin\Keil_v5\C51\Inc\intrins.h" /0
 
 extern void          _push_    (unsigned char _sfr);
 extern void          _pop_     (unsigned char _sfr);
 
 
 #pragma RESTORE
 
 
 
#line 19 "hw.h" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 sfr IAP_DATA=0xc2;
 sfr IAP_ADDRH=0xc3;
 sfr IAP_ADDRL=0xc4;
 sfr IAP_CMD=0xc5;
 sfr IAP_TRIG=0xc6;
 sfr IAP_CONTR=0xc7;
 
 sfr AUXR=0x8e;
 sfr T2H=0xd6;
 sfr T2L=0xd7;
 sfr IE2=0xaf;
 sfr P1ASF=0x9d;
 sfr ADC_CONTR=0xbc;
 sfr ADC_RES=0xbd;
 sfr ADC_RESL=0xbe;
 
 sfr P5=0xc8;
 sfr P0M0=0x94;
 sfr P1M0=0x92;
 sfr P2M0=0x96;
 sfr P3M0=0xb2;
 sfr P4M0=0xb4;
 sfr P1M1=0x91;
 
 sfr P_SW1 = 0xA2;
 sfr CMOD=0xd9;
 sfr CCON=0xd8;
 sfr CCAPM0=0xda;
 sfr CCAPM1=0xdb;
 sfr CCAPM2=0xdc;
 
 sfr CCAP0L=0xea;
 sbit CCF0 = CCON^0;
 
 sfr CCAP0H=0xfa;
 sfr CCAP1H=0xfb;
 sfr CCAP2H=0xfc;
 sfr CL=0xe9;
 sfr CH=0xf9;
 sfr PCA_PWM0=0xf2;
 sfr PCA_PWM1=0xf3;
 sfr PCA_PWM2=0xf4;
 
 
 
 
 void delay(unsigned int ms);
 void delayMicroseconds(unsigned int us);
  unsigned int micros(void);
  unsigned long millis(void);
 
 void port_init();
 void hw_pwm_en(unsigned char ch);
 void hw_pwm_dis(unsigned char ch);
 void hw_pwm_set(unsigned char ch, unsigned char pwm);
 void hw_init();
  unsigned int hw_adc_read();
 void hw_adc_open(unsigned char i);
 
 
 
 
 
#line 17 "hw.c" /0
 
  
#line 1 "lib.h" /0









 
 
 
 
 
 
 
 
  
#line 1 "hw.h" /0









 
 
 
#line 13 "hw.h" /1
  
 
 
 
 
  
  
 
 
 
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 18 "lib.h" /0
#line 18 "lib.h" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 typedef struct
 {
  unsigned char	isRuning;
  unsigned char	isTimeout;
  unsigned int count;
  unsigned int reload;
 void (*callback)(void);
 
 }	STimer;
 
 
 
 void delay_1_ms();
 void delay(unsigned int i);
 
 
 
  unsigned char		EEPROM_read (unsigned int addr);
 void	EEPROM_write(unsigned int addr,unsigned char byte);
 void	EEPROM_cleanPage(unsigned char addPage);
  unsigned char		EEPROM_test (void);
 
 
 
 void NRF_FIFO_write(unsigned char DATA_OUT[],unsigned char lengh);
 void NRF_FIFO_read(unsigned char DATA_IN[],unsigned char lengh);
 void NRF_addr_tx(unsigned char DATA_IN[]);
 void NRF_addr_rx(unsigned char DATA_IN[]);
 void NRF_mode_rx();
 void NRF_mode_tx();
 void NRF_power(unsigned char P);
 
 void NRF_channel(unsigned char c);
 void NRF_init(void); 
  unsigned char   NRF_test();
 void NRF_irq_clean(void);
 
 
 
 void LED_on(void);
 void LED_off(void);
 void LED_flash(unsigned char t);
 
 
 
 void UART_init(unsigned char bps);
 void UART_pushByte(unsigned char c);	 
 void UART_puts(unsigned char* str);
 void UART_putData(unsigned int i);
 
 
 
 void timer_init(STimer* pTimers, unsigned char numTimers);
 void timer_cbRegist(unsigned char index, void (*callback)(void));
 void timer_startOnce(unsigned char index, unsigned int ms);
 void timer_startTimes(unsigned char index, unsigned int ms, unsigned char times);
 void timer_startCycle(unsigned char index, unsigned int ms);
 void timer_tick();	 
 void timer_process();
 
 
 
 
 
#line 18 "hw.c" /0
 
  
#line 1 "config.h" /0









 
 
 
 
 
 
 
 
 
 
 sbit LED=P1^0;
 
 
 sbit CH1=P3^7;
 sbit CH2=P3^6;
 sbit CH3=P3^5;
 sbit CH4=P3^4;
 sbit CH5=P3^3;
 sbit CH6=P3^2;
 sbit CH7=P3^1;
 sbit CH8=P3^0;
 
 
 sbit CE  =P5^5;
 sbit SCK =P5^4;
 sbit MISO=P1^7;
 sbit IRQ =P1^6;
 sbit MOSI=P1^5;
 sbit CSN =P1^4;
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 62 "config.h" /1
  
 
#line 64 "config.h" /0
 
 
 
 
#line 19 "hw.c" /0
 
 
 
 
 static idata unsigned long _sys_ms;
 
 
 
 
 
 
#line 30 "hw.c" /1
  
 
  
 
#line 34 "hw.c" /0
 
 
 
 void port_init()
 {
 
 CH1=0;
 CH2=0;
 CH3=0;
 CH4=0;
 CH5=0;
 CH6=0;
 CH7=0;
 CH8=0;
 
 
 P1M1 |=  1<<3;	 
 P1M0 &=~(1<<3);
 
 }
 
 
 
 void hw_pwm_en(unsigned char ch)
 {
 if(ch==1) 		CCAPM2=0x42;	 
 else if(ch==2)CCAPM1=0x42;
 }
 
 void hw_pwm_dis(unsigned char ch)
 {
 if(ch==1) 		CCAPM2 = 0;		 
 else if(ch==2)CCAPM1 = 0;
 }
 
 void hw_pwm_set(unsigned char ch, unsigned char pwm)
 {
 if(ch==1)
 {
 if(pwm) PCA_PWM2=0x00;	 
 else		PCA_PWM2=0x03;	 
 CCAP2H=255-pwm;					 
 }
 else if(ch==2)
 {
 if(pwm) PCA_PWM1=0x00;
 else		PCA_PWM1=0x03;
 CCAP1H=255-pwm;
 }
 }
 
 
 
 void hw_init()
 {
 
 IE = 0;
 IE2= 0;
 
 
 ADC_CONTR=0x80;		 
 delay(2);			 
 
 P1ASF=0x00;	  					 
 ADC_CONTR=0x88;					 
 while(ADC_CONTR&0x10==0); 
 
 
 IP=0x02; 			 
 TMOD &= 0x0F;	 
 TMOD |= 0x00;
 IE=0x02;  
 
 
 P_SW1 &= ~(3<<4);	 
 P_SW1 |= 1<<4;
 PCA_PWM2=0x03;		 
 PCA_PWM1=0x03;		 
 
 hw_pwm_dis(1);
 hw_pwm_dis(2);
 
 
 CCON =0x00;			 
 CL=0;CH=0;			 
 CMOD=0x00;			 
 _sys_ms = 1000;	 
 CCAP0L = _sys_ms;
 CCAP0H = _sys_ms >> 8;
 CCAPM0 = 0x49;	 
 
 
 UART_init(2);
 }
 
 
 
  unsigned int hw_adc_read()
 {
  unsigned int adc;
 
 adc=ADC_RES;
 adc<<=2,adc+=ADC_RESL;
 
 return adc;
 }
 
 void hw_adc_open(unsigned char i)	 
 {
 if(i&0x01)
 {
 P1ASF=0x01<<3;		 
 ADC_CONTR= 1<<7 | 0<<4 | 1<<3 | 3<<0;		 
 }
 else
 {
 P1ASF=0x00;	  		 
 ADC_CONTR=0x88;		 
 }
 }
 
 
 
#line 157 "hw.c" /1
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 184 "hw.c" /0
  unsigned int micros(void)
 {
  unsigned char H0,H1,L;
 
 H0 = CH;
 L	 = CL;
 H1 = CH;
 
 if(H1!=H0) L=0;
 
 return ((unsigned int)H1<<8) | L;
 }
 
  unsigned long millis(void)
 {
  unsigned long temp;
 
 ET0 = 0;
 temp = _sys_ms;
 ET0 = 1;
 
 return temp;
 }
 
 void isr_PCA() interrupt 7
 {
 CCF0 = 0;	 
 _sys_ms += 1000;
 CCAP0L = _sys_ms;
 CCAP0H = _sys_ms >> 8;
 
 timer_tick();
 }
 
 
 

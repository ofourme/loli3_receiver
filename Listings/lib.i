
#line 1 "lib.c" /0









 
 
 
 
 
 
 
  
#line 1 "lib.h" /0









 
 
 
 
 
 
 
 
  
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
 
 
 
 
 
#line 17 "lib.c" /0
 
  
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
 
 
 
 
#line 18 "lib.c" /0
 
  
#line 1 "spi.h" /0









 
 
 
 
 
 
 
 
 
  
#line 1 "hw.h" /0









 
 
 
#line 13 "hw.h" /1
  
 
 
 
 
  
  
 
 
 
  
  
  
  
  
  
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 19 "spi.h" /0
#line 19 "spi.h" /0
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 typedef struct
 {
   unsigned char  (*begin)(unsigned char w);
   unsigned char (*transfer)(unsigned char);
 void		(*slaveEnable)(void);
 void		(*slaveDisable)(void);
 void    (*end)(void);
 
 }   SSpi;
 
 
 
 
 
#line 19 "lib.c" /0
 
  
#line 1 "loli3_pact.h" /0









 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 20 "lib.c" /0
 
 
 
 
 extern STimer idata timer[];
 
 
 
 
 
 void delay_1_ms()		 
 {
 unsigned char i, j;
 i = 12;
 j = 168;
 do
 {
 while (--j);
 } while (--i);
 }
 
 void delay(unsigned int i)
 {
 while(i--)
 delay_1_ms();
 }
 
 
 
 
  unsigned char EEPROM_read(unsigned int addr)	 
 {
 IAP_CMD=0x01;
 
 IAP_ADDRH=addr>>8;
 IAP_ADDRL=addr;	
 
 IAP_TRIG=0x5a;			
 IAP_TRIG=0xa5;
 _nop_();
 
 return IAP_DATA;
 }
 
 void EEPROM_write(unsigned int addr,unsigned char byte)	 
 {
 IAP_CMD=0x02;
 IAP_DATA=byte;
 
 IAP_ADDRH=addr>>8;
 IAP_ADDRL=addr;	
 
 IAP_TRIG=0x5a;
 IAP_TRIG=0xa5;
 _nop_();
 }
 
 void EEPROM_cleanPage(unsigned char addPage)	 
 {
 IAP_CMD=0x03;
 IAP_ADDRH=(512*addPage)>>8;
 IAP_ADDRL=0;
 IAP_TRIG=0x5a;
 IAP_TRIG=0xa5;
 _nop_();
 }
 
  unsigned char EEPROM_test(void)
 {
  unsigned char addrPage = 1;
 
 IAP_CONTR=0x83;
 
 EEPROM_cleanPage(addrPage);
 EEPROM_write(512*addrPage,0x88);
 if(EEPROM_read(512*addrPage)!=0x88) return 1;	 
 
 EEPROM_cleanPage(addrPage);
 EEPROM_write(512*addrPage,0x55);
 if(EEPROM_read(512*addrPage)!=0x55) return 1;	 
 
 return 0;	 
 }
 
 
 
 
 
  unsigned char SPI_begin(unsigned char setting)
 {
 if(setting!=((0<<0)|(0<<2))) return 0;
 
 
 
  do{CSN=1;}while(0);
  do{SCK=0;}while(0);
  do{MOSI=0;}while(0);
 return 1;
 }
 
  unsigned char SPI_transfer(unsigned char byte)
 {
  unsigned char i;
 for(i=0;i<8;i++)
 {
 if(byte&0x80)
 {
  do{MOSI=1;}while(0);
 }
 else
 {
  do{MOSI=0;}while(0);
 }
  do{SCK=1;}while(0);
 byte<<=1;
 byte|=MISO;
  do{SCK=0;}while(0);
 }
 return byte;
 }
 
 void SPI_slaveEnable(void)
 {
  do{CSN=0;}while(0);
 }
 
 void SPI_slaveDisable(void)
 {
  do{CSN=1;}while(0);
 }
 
 void SPI_end(void)
 {
 
 }
 
 SSpi code SPI = {SPI_begin, SPI_transfer, SPI_slaveEnable, SPI_slaveDisable,SPI_end};
 
 
 
 
 void NRF_REG_write(unsigned char address,unsigned char command)
 {
 SPI_slaveEnable();
 SPI_transfer(0x20+address);
 SPI_transfer(command);
 SPI_slaveDisable();
 }
 
 void NRF_FIFO_write(unsigned char DATA_OUT[],unsigned char lengh)
 {
  unsigned char i;
 
 SPI_slaveEnable();
 SPI_transfer(0xa0);
 for(i=0;i<lengh;i++) SPI_transfer(DATA_OUT[i]);
 SPI_slaveDisable();
 }
 void NRF_FIFO_read(unsigned char DATA_IN[],unsigned char lengh)
 {
  unsigned char i;
 
 SPI_slaveEnable();
 SPI_transfer(0x61);	 
 for(i=0;i<lengh;i++) DATA_IN[i]=SPI_transfer(0);	   
 SPI_slaveDisable();
 }
 
 void NRF_addr_tx(unsigned char DATA_IN[])
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
 void NRF_addr_rx(unsigned char DATA_IN[])
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
  do{CE=0;}while(0);
 NRF_REG_write(0x00,0x3b);  
  do{CE=1;}while(0);
 } 				   
 
 void NRF_mode_tx()				 
 {
  do{CE=0;}while(0);
 NRF_REG_write(0x00,0x0a);
  do{CE=1;}while(0);
 }
 
 void  NRF_power(unsigned char P)				 
 {														
  do{CE=0;}while(0);
 if(P==3)		 NRF_REG_write(0x06,0x27);		   
 else if(P==2)NRF_REG_write(0x06,0x25);	   
 else if(P==1)NRF_REG_write(0x06,0x23);	   
 else if(P==0)NRF_REG_write(0x06,0x21);     
  do{CE=1;}while(0);
 }
 
 void NRF_size(unsigned char l)
 {
  do{CE=0;}while(0);
 NRF_REG_write(0x11,l);  
  do{CE=1;}while(0);
 }
 
 void NRF_channel(unsigned char c)
 {
  do{CE=0;}while(0);
 NRF_REG_write(0x05,c);  
  do{CE=1;}while(0);
 }
 
 void NRF_init_0(void)
 {
  do{CE=0;}while(0);
 NRF_REG_write(0x01,0x00);  
 NRF_REG_write(0x02,0x01);  
 NRF_REG_write(0x04,0x00);  
  do{CE=1;}while(0);
 }
 
 void NRF_irq_clean(void)
 {
  do{CE=0;}while(0);
 
 NRF_REG_write(0x07,0x70);	 
 
  do{CE=1;}while(0);
 }
 
 void NRF_init(void)	 
 {
 
 SPI_begin(0);
 
 NRF_init_0();
 NRF_mode_rx();
 
 NRF_power(0);
 NRF_size(11);
 
 
 }
 
 
 
 
 
 
 
 
  unsigned char NRF_test()	 
 {	
  unsigned char reset_err=0;
  unsigned char NRF_error = 0;
  unsigned char tx[1] = {'T'};	 
 
 SPI_begin(0);
  do{CE=0;}while(0);
 
 SPI_slaveEnable();
 if(SPI_transfer(0x20)!=0x0e){reset_err=1;}
 SPI_transfer(0x0a);
 SPI_slaveDisable();
 
 SPI_slaveEnable();
 SPI_transfer(0x00);
 if(SPI_transfer(0x00)!=0x0a){NRF_error|=0x02;} 
 SPI_slaveDisable();
 
 NRF_REG_write(0x01,0x00);
 NRF_REG_write(0x04,0x00);
 NRF_REG_write(0x11,1);
 NRF_FIFO_write(tx,1);
 
  do{CE=1;}while(0);
 
 delay(2);
 
 SPI_slaveEnable();
 if(SPI_transfer(0x00)!=0x2e){NRF_error|=0x04;} 
 SPI_slaveDisable();
 
 
 if(IRQ)NRF_error|=0x08;	 
 else 
 {
 if(NRF_error&0x04==0)NRF_error|=0x10;		 
 }
  do{CE=1;}while(0);
 
 if(reset_err&&NRF_error>1)NRF_error|=0x01; 
 
 NRF_irq_clean();
 
 return NRF_error;
 }
 
 
 
 void LED_on(void)
 {
  do{LED = 1;}while(0);
 }
 
 void LED_off(void)
 {
  do{LED = 0;}while(0);
 }
 
 void LED_flash(unsigned char t)
 {
 while(t)
 {
  do{LED = 1;}while(0);		
 delay(50);
  do{LED = 0;}while(0);		
 delay(50);
 t--;
 }
 }
 
 
 
 void UART_init(unsigned char bps)
 {
 SCON = 0xD0;		 
 AUXR |= 0x01;		 
 AUXR |= 0x04;		 
 
 if(bps==1)	 
 {
 T2L = 0xC7;
 T2H = 0xFE;
 }
 else if(bps==2)	 
 {
 T2L = 0xE6;		 
 T2H = 0xFF;		 
 }
 else			 
 {
 T2L = 0xE2;		 
 T2H = 0xFF;		 
 }
 
  CH7 = 1;		 
 AUXR |= 0x10;		 
 }
 
 void UART_pushByte(unsigned char c)
 {
 if(TI)TI=0;
 ACC=c;
 TB8=P;
 SBUF=ACC;
 }
 
 void UART_putData(unsigned int i)
 {
  unsigned char t;
 
 UART_pushByte('0');		while(!TI);
 UART_pushByte('x');		while(!TI);
 
 t=i>>12;
 if(t<10) {UART_pushByte(t+'0');		while(!TI);}
 else		 {UART_pushByte(t+'A'-10);while(!TI);}
 t=(i>>8)&0x0F;
 if(t<10) {UART_pushByte(t+'0');		while(!TI);}
 else		 {UART_pushByte(t+'A'-10);while(!TI);}
 t=(i>>4)&0x0F;
 if(t<10) {UART_pushByte(t+'0');		while(!TI);}
 else		 {UART_pushByte(t+'A'-10);while(!TI);}
 t=(i>>0)&0x0F;
 if(t<10) {UART_pushByte(t+'0');		while(!TI);}
 else		 {UART_pushByte(t+'A'-10);while(!TI);}
 
 UART_pushByte('\r');
 while(!TI);
 UART_pushByte('\n');
 while(!TI);
 }
 
 void UART_puts(unsigned char* str)
 {
 while(*str)
 {
 UART_pushByte(*str++);
 while(!TI);	 
 }
 UART_pushByte('\r');
 while(!TI);
 UART_pushByte('\n');
 while(!TI);
 }
 
 
 
 static STimer* _pTimers;
 static unsigned char	_numTimers;
 
 void timer_init(STimer* pTimers, unsigned char numTimers)
 {
  unsigned char i;
 
 _pTimers = pTimers;
 _numTimers = numTimers;
 
 for(i=0; i<_numTimers; i++)	 
 {
 _pTimers[i].isRuning  = 0;		 
 _pTimers[i].isTimeout = 0;		 
 _pTimers[i].count 		 = 0;
 _pTimers[i].reload 	 = 0;
 _pTimers[i].callback  = 0;
 }
 }
 
 void timer_cbRegist(unsigned char index, void (*callback)(void))
 {
 
 
 
 
 _pTimers[index].callback = callback;
 }
 
 void timer_startOnce(unsigned char index, unsigned int ms)
 {
 _pTimers[index].isRuning = 0;	 
 _pTimers[index].isTimeout = 0;
 _pTimers[index].count = ms;
 _pTimers[index].reload = 0;
 _pTimers[index].isRuning = 1;	 
 }
 
 void timer_startTimes(unsigned char index, unsigned int ms, unsigned char times)
 {
 _pTimers[index].isRuning = 0;	 
 _pTimers[index].isTimeout = 0;
 _pTimers[index].count = timer[index].reload = ms;
 _pTimers[index].isRuning = times;	 
 }
 
 void timer_startCycle(unsigned char index, unsigned int ms)
 {
 timer_startTimes(index, ms, 0xff);
 }
 
 void timer_tick()	 
 {
 static unsigned char step=0;
  unsigned char i;
 
 step++;
 if(step < 3) return;
 step = 0;
 
 for(i=0;i<_numTimers;i++)
 {
 if(_pTimers[i].isRuning)
 {
 if(_pTimers[i].count>3) _pTimers[i].count -= 3;
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
  unsigned char i;
 
 for(i=0;i<_numTimers;i++)
 {
 if(_pTimers[i].isTimeout)
 {
 _pTimers[i].isTimeout = 0;
 if(_pTimers[i].callback) _pTimers[i].callback();
 }
 }
 
 }
 
 
 

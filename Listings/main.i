
#line 1 "main.c" /0









 
 
 
 
 
 
 
  
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
 
 
 
 
 
#line 17 "main.c" /0
 
  
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
 
 
 
 
 
#line 18 "main.c" /0
 
 
 
 
  
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
 
 
 
 
#line 22 "main.c" /0
 
  
#line 1 "loli3_pact.h" /0









 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 23 "main.c" /0
 
 
 
 
 typedef struct
 {
  unsigned char isValid;
  unsigned char hopping[5];		 
  unsigned char address[5];		 
  unsigned char PPM;						 
  unsigned int out_control_data[8];
  unsigned char CH1_PWM,CH2_PWM,CH7_PWM;	 
  unsigned char CH1_SW,CH2_SW,CH3_SW,CH4_SW,CH5_SW,CH6_SW,CH7_SW,CH8_SW;	 
  unsigned char SBUS;					 
 
 } Loli3RecvSet;		 
 
 typedef struct
 {
  unsigned int CH_data[8];		 
  unsigned int voltage_ic, voltage_batt;	 
  unsigned int adc_ic, adc_batt;					 
  unsigned char 	rx_num;				 
  unsigned char 	SBUS_tx[25];	 
  unsigned char  stateLED;			 
  unsigned char  bModeChange;
 
 } Loli3RecvData;	 
 
 typedef struct
 {
  unsigned char rx[11];
  unsigned char tx[11];
  unsigned char rx_cnt;				 
  unsigned char hopping_cnt;		 
  unsigned char channel_index;	 
 
 } Loli3RecvNrf;			 
 
 
  unsigned char code _random_hopping_index[100]={
 4,1,3,2,2,1,0,0,2,2,2,3,4,1,2,1,4,3,3,4, 
 2,0,2,2,3,1,2,3,2,2,2,4,2,4,0,3,4,2,3,1,
 0,3,1,3,3,0,2,0,4,3,3,3,3,3,4,1,1,4,3,0,
 1,0,3,2,3,2,3,3,4,4,1,3,0,0,3,1,3,3,3,0,
 3,3,4,1,2,4,1,3,0,1,3,4,4,3,2,3,1,2,3,3};
 
  unsigned char code recv_address_startup[5]={'L','O','V','E','!'};
 
 Loli3RecvSet xdata	recvSet = {0,10,35,60,85,110,'L','O','V','E','!',0,511,511,80,511,511,511,511,511};
 Loli3RecvData		recv 	= {0,1023,0,1023,0,1023,0,0};
 Loli3RecvNrf  xdata	nrf 	= {0};
 
 enum {STATE_LED_OFF=0, STATE_LED_FLASH, STATE_LED_ON, STATE_LED_IDLE} E_STATE_LED;
 enum {TIMER_LED_FLASH=0, TIMER_SIGNAL_COUNT, TIMER_SIGNAL_LOST, TIMER_DATA_SAVE,  TIMER_LED_OFF, TIMER_CHANNEL_LOST, TIMER_ADC, NUM_TIMERS} E_TIMER;
 
 STimer idata timer[NUM_TIMERS];
 
  unsigned int _adc_ic,_adc_batt;
 
 
 
 
 
 
 void DATA_read()
 {
  unsigned char i;
  unsigned int sum,sum_read;
  unsigned char* set;
 
 recvSet.isValid = 0;
  do{IAP_CONTR=0x83;}while(0);
 if( EEPROM_read(0)!= 0xab )
 {
  do{IAP_CONTR=0x00;}while(0);
 return;
 }
 
 sum = 0xab;
 for(i=1,set=(unsigned char*)&recvSet+1;i<sizeof(Loli3RecvSet);i++)
 {
 *set=EEPROM_read(i);
 sum+=*set++;
 }
 
 sum_read=EEPROM_read(sizeof(Loli3RecvSet)); 
 sum_read<<=8;
 sum_read+=EEPROM_read(sizeof(Loli3RecvSet)+1);
 
  do{IAP_CONTR=0x00;}while(0);
 
 if(sum!=sum_read) return;	 
 recvSet.isValid = 0xab;
 return;
 }
 
 void DATA_save()
 {
  unsigned char i;
  unsigned int sum;
  unsigned char* set;
 
  do{IAP_CONTR=0x83;}while(0);
 EEPROM_cleanPage(0);
 recvSet.isValid = 0xab;
 for(i=0,sum=0,set=(unsigned char*)&recvSet;i<sizeof(Loli3RecvSet);i++)
 {
 EEPROM_write(i,*set);
 sum+=*set++;
 }
 EEPROM_write(sizeof(Loli3RecvSet),sum>>8);
 EEPROM_write(sizeof(Loli3RecvSet)+1,sum);
 
  do{IAP_CONTR=0x00;}while(0);
 }
 
 
 
 
 
 void SBUS_data_push()
 {
  unsigned char i;
  unsigned char buff[12];
 
 buff[1] = 						   (unsigned char)(recv.CH_data[0])<<1;
 buff[2] = (unsigned char)(recv.CH_data[0]>>7)+(unsigned char)(recv.CH_data[1])<<4;
 buff[3] = (unsigned char)(recv.CH_data[1]>>4)+(unsigned char)(recv.CH_data[2])<<7;
 buff[4] = (unsigned char)(recv.CH_data[2]>>1);
 buff[5] = (unsigned char)(recv.CH_data[2]>>9)+(unsigned char)(recv.CH_data[3])<<2;
 buff[6] = (unsigned char)(recv.CH_data[3]>>6)+(unsigned char)(recv.CH_data[4])<<5;
 buff[7] = (unsigned char)(recv.CH_data[4]>>3)+(unsigned char)(recv.CH_data[5])<<8;
 buff[8] = (unsigned char)(recv.CH_data[5]);
 buff[9] = (unsigned char)(recv.CH_data[5]>>8)+(unsigned char)(recv.CH_data[6])<<3;
 buff[10]= (unsigned char)(recv.CH_data[6]>>5)+(unsigned char)(recv.CH_data[7])<<6;
 buff[11]= (unsigned char)(recv.CH_data[7]>>2);
 
 recv.SBUS_tx[0]=0x0f;	 
  do{EA = 0;}while(0);			 
 for(i=1	; i<=11; i++)	 
 {
 recv.SBUS_tx[i]=buff[i];
 }
  do{EA = 1;}while(0);
 for(		; i<=22; i++)	 
 {
 recv.SBUS_tx[i]=0;
 }
 recv.SBUS_tx[23]=0;		 
 recv.SBUS_tx[24]=0;		 
 }
 
 
 
 void callback_timer_signal_count(void)
 {
 recv.rx_num=nrf.rx_cnt;
 nrf.rx_cnt=0;
 }
 
 void callback_timer_signal_lost(void)
 {
  unsigned char i;
 
  UART_puts("LOLI3 recv signal lost!!!");
 recv.stateLED = STATE_LED_ON;
 
  do{EA = 0;}while(0);
 for(i=0; i<8; i++)
 {
 recv.CH_data[i]=recvSet.out_control_data[i];
 }
  do{EA = 1;}while(0);
 
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
 static unsigned char index = 0;
 
 if( IRQ )
 {
 nrf.hopping_cnt++;
 if(nrf.hopping_cnt>5)
 {
 if(index>=   (sizeof(_random_hopping_index)/sizeof(_random_hopping_index[0])) ) index=0;
 nrf.channel_index =  _random_hopping_index[index++];
 }
 else
 {
 nrf.channel_index++;
 if(nrf.channel_index>=5)nrf.channel_index=0;
 }
 NRF_channel(recvSet.hopping[nrf.channel_index]);
 }
 }
 
 void callback_timer_adc(void)
 {
 static unsigned char i=0;
  unsigned int adc;
 
 adc = hw_adc_read();
 i++;
 if(i&0x01)
 {
 _adc_ic = adc + ((_adc_ic*7)>>3);	 
 hw_adc_open(1);
 }
 else
 {
 _adc_batt = adc + ((_adc_batt*7)>>3);	 
 hw_adc_open(0);
 }
 }
 
 void timers_init()
 {
 timer_init(&timer, NUM_TIMERS);
 
 timer_cbRegist(TIMER_SIGNAL_COUNT,callback_timer_signal_count);
 timer_cbRegist(TIMER_SIGNAL_LOST, callback_timer_signal_lost);
 timer_cbRegist(TIMER_DATA_SAVE,		callback_timer_data_save);
 timer_cbRegist(TIMER_LED_OFF,			callback_timer_led_off);
 timer_cbRegist(TIMER_CHANNEL_LOST,callback_timer_channel_lost);
 timer_cbRegist(TIMER_ADC,					callback_timer_adc);
 }
 
 
 
 
 void NRF_read(void)
 {
 NRF_FIFO_read(nrf.rx, 11);
 }
 
 void NRF_write(void)
 {
 NRF_FIFO_write(nrf.tx,11);
 }
 
 
 
 void recv_connect()
 {
 NRF_channel(66);
 NRF_addr_rx(recvSet.address);
 NRF_addr_tx(recvSet.address);
 NRF_power(3);
 }
 
 void recv_reconnect()
 {
  unsigned char i,t;
 
 NRF_channel(33);
 NRF_addr_tx(recv_address_startup);
 NRF_addr_rx(recv_address_startup);
 
 do
 {
 while(IRQ);
 NRF_read();
 NRF_irq_clean();
 
 }	while(nrf.rx[0]!=0xa0);
 
 
 for(i=0,t=1; i<5; i++,t++)
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
 NRF_channel(33);
 NRF_addr_tx(recv_address_startup);
 NRF_addr_rx(recv_address_startup);
 NRF_write();
 delay_1_ms();
 
 NRF_mode_rx();
 NRF_channel(recvSet.hopping[0]);
 NRF_addr_tx(recvSet.address);
 NRF_addr_rx(recvSet.address);
 for(t=100; t--; )
 {
 delay_1_ms();
 if(IRQ==0)	 
 {
 NRF_read();
 NRF_irq_clean();
 DATA_save();
 NRF_power(3);
 return;
 }
 }
 }
 }
 
 
 
 void recv_init()
 {
  unsigned char restar = 0;
  unsigned char temp;
 
 CH5=0;CH6=1;
 delay_1_ms();
 if(CH6==0)		  	 
 {
 P3M0=1<<3;;	   
 
 CH5=1;
 delay_1_ms();
 if(CH6)		LED_flash(20), restar=1;    
 }
 CH5=0;CH6=0;
 P3M0=0xff;	 
 
 while(EEPROM_test())
 {
 LED_flash(3);
  UART_puts("EEPROM error!");
 delay(1000);
 }
 DATA_read();
 
 delay(1000);
 temp = NRF_test();
 while(temp)
 {
 UART_putData(temp);
 LED_flash(10);
  UART_puts("NRF24L01 error!");
 delay(1000);
 temp = NRF_test();
 }
 NRF_init();
 if(restar || recvSet.isValid==0)
 {
 recv_reconnect();
 }
 else
 {
 recv_connect();
 }
 
 recv.stateLED = STATE_LED_OFF;
 }
 
 
 void recv_output_en(void)
 {
 if(recvSet.CH1_PWM) hw_pwm_en(1);
 if(recvSet.CH2_PWM) hw_pwm_en(2);
 
 TR0=1;					 
 CCON =0x40;			 
 EA =1;
 }
 
 
 
 
 void ET0_isr()interrupt 1	using 1	 
 {
 static unsigned int temp=0;
 static unsigned int temp1=0;
 static unsigned int temp2=0;
 static unsigned char state = 0;
 static unsigned char T_h=0,T_l=0;
 
 
 if(recv.bModeChange)
 {
 recv.bModeChange = 0;
 state = 0;
 }
 
 if(recvSet.PPM)			   
 {
 
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
  CH7=1;
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
 TL0=0xF0;TH0=0xD8; 
 }
 else
 {
 TL0=0x88;TH0=0xFF; 
 }
 }
 else
 {
 state++;
 switch(state)
 {
 case 1:
 if(recvSet.CH1_PWM) 
 {
 hw_pwm_set(1,recv.CH_data[0]/4);
 }
 else if(recvSet.CH1_SW) 
 {
 if(recv.CH_data[0]<500) CH1=0;
 else	CH1=1;
 }
 else
 {
 CH1=1;	 
 }
 temp2=128600-temp1;
 TL0=temp2,TH0=temp2>>8;
 break;
 
 case 2:
 if(recvSet.CH1_PWM==0 && recvSet.CH1_SW==0)	 
 {
 CH1=0;
 }
 temp1=64725-recv.CH_data[1]*27/20;
 TL0=temp1,TH0=temp1>>8;
 break;
 
 case 3:
 if(recvSet.CH2_PWM)
 {
 hw_pwm_set(2,recv.CH_data[0]/4);
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
 
#line 586 "main.c" /1
 
 
 
 
 
 
 
#line 593 "main.c" /0
 temp2=128600-temp1;
 TL0=temp2,TH0=temp2>>8;break;
 case 14:
 
#line 597 "main.c" /1
 
 
#line 599 "main.c" /0
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
 
 
 
 
 
 int main()
 {
  unsigned int buff[8];
  unsigned char i;
  unsigned int t1,t2;
 
 port_init();
 LED_on();
 delay(500); 
 
 hw_init();
 
 
 timers_init();
 timer_startCycle(TIMER_ADC, 50);
 timer_startCycle(TIMER_SIGNAL_COUNT,1000);
 
 recv_init();
 LED_off();		 
  UART_puts("LOLI3 recv init OK!");
 
 NRF_channel(recvSet.hopping[0]);
 while(IRQ);
 recv_output_en();
 while(1)
 {
 timer_process();
 
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
 
 
#line 669 "main.c" /1
 
 
#line 671 "main.c" /0
 if(IRQ)
 {
 }
 else
 {
 if(recv.stateLED ==STATE_LED_ON) recv.stateLED = STATE_LED_OFF;
 
 nrf.rx_cnt++;
 nrf.hopping_cnt=0; 
 
 timer_startOnce(TIMER_SIGNAL_LOST, 2000);
 timer_startCycle(TIMER_CHANNEL_LOST, 24);	 
 
 NRF_read();		 
 NRF_irq_clean();
 NRF_mode_tx();
 nrf.tx[0]=recv.rx_num;
 
 recv.voltage_ic=(5ul*100*8)*260/_adc_ic;	  
 nrf.tx[1]=recv.voltage_ic>>8;
 nrf.tx[2]=recv.voltage_ic;
 recv.voltage_batt=((long)_adc_batt*recv.voltage_ic*3)>>13;
 nrf.tx[3]=recv.voltage_batt>>8;
 nrf.tx[4]=recv.voltage_batt;
 NRF_write();
 
#line 697 "main.c" /1
 
 
 
#line 700 "main.c" /0
 t1 = micros();
 while(IRQ);	 
 t2 = micros();
 t2-=t1;
 UART_putData(t2);
 NRF_mode_rx();	 
 
 NRF_mode_rx();
 
 nrf.channel_index++;
 if(nrf.channel_index>4)nrf.channel_index=0;
 NRF_channel(recvSet.hopping[nrf.channel_index]);
 
 if(nrf.rx[0]==0xa2)
 {
 if(nrf.rx[1]&0x80)	{	recvSet.PPM=1;	}
 else  							{	recvSet.PPM=0;	}
 if(nrf.rx[1]&0x40)	{	recvSet.SBUS=1;			}
 else  							{	recvSet.SBUS=0;			}
 if(nrf.rx[1]&0x08)	{	recvSet.CH1_PWM=1; hw_pwm_en(1);	}
 else 								{	recvSet.CH1_PWM=0; hw_pwm_dis(1);}
 if(nrf.rx[1]&0x04)	{	recvSet.CH2_PWM=1; hw_pwm_en(2);	}
 else 								{	recvSet.CH2_PWM=0; hw_pwm_dis(2);}
 if(nrf.rx[1]&0x02)	{	recvSet.CH7_PWM=1;	} 
 else 								{	recvSet.CH7_PWM=0;	} 
 if(nrf.rx[2]&0x80)	{ recvSet.CH1_SW=1;		}
 else 								{ recvSet.CH1_SW=0;		}
 if(nrf.rx[2]&0x40)	{ recvSet.CH2_SW=1;		}
 else 								{ recvSet.CH2_SW=0;		}
 if(nrf.rx[2]&0x20)	{	recvSet.CH3_SW=1;		}
 else								{	recvSet.CH3_SW=0;		}
 if(nrf.rx[2]&0x10)	{	recvSet.CH4_SW=1;		}
 else								{	recvSet.CH4_SW=0;		}
 if(nrf.rx[2]&0x08)	{	recvSet.CH5_SW=1;		}
 else								{	recvSet.CH5_SW=0;		}
 if(nrf.rx[2]&0x04)	{	recvSet.CH6_SW=1;		}
 else								{	recvSet.CH6_SW=0;		}
 if(nrf.rx[2]&0x02)	{	recvSet.CH7_SW=1;		}
 else								{	recvSet.CH7_SW=0;		}
 if(nrf.rx[2]&0x01)	{	recvSet.CH8_SW=1;		}
 else								{ recvSet.CH8_SW=0;		}
 
 recv.bModeChange = 1;
 timer_startOnce(TIMER_DATA_SAVE, 1000);	 
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
 
 timer_startOnce(TIMER_DATA_SAVE, 1000);	 
 }
 else	 
 {
 for(i=0; i<8; i++)
 {
  do{EA = 0;}while(0);		 
 recv.CH_data[i]=buff[i];
  do{EA = 1;}while(0);
 }
 SBUS_data_push();
 }
 }
 }
 
 }
 }
 
 
 

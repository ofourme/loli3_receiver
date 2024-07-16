/*
********************************************************************************
*                             ---ofme---
*           Copyleft (c) 2021-2023, 微风山谷 / ofourme@163.com
*           License: LGPL
*
*   APP.C-V1.0.0 (2023.Oct.27th)
*
********************************************************************************
*/

#define __APP_C__

/*-DO NOT ADD YOUR CODE BEFORE HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/
/******************************************************************************/

#include "loli3_recv.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/*////////////////////////////////////////////////////////////////////////////*/

void setup(void)
{
	hw_sys_init();

  LED_on();
  delay(200);

  timers_init();
  timer_startCycle(TIMER_ADC, 50);
  timer_startCycle(TIMER_SIGNAL_COUNT,1000);
  DBG("LOLI3 recv start up.");

  recv_init();
  LED_off();    //点亮指示灯再关闭，表示单片机正常工作

  NRF_channel(recvSet.hopping[0]);
  while(noRF());
  DBG("LOLI3 recv connected.");
  recv_begin();
#if 0
	//空间不够的时候注释掉	
	DBG("ADDR:");
	DBG(Int16ToStr(recvSet.address[0]));
	DBG(Int16ToStr(recvSet.address[1]));
	DBG(Int16ToStr(recvSet.address[2]));
	DBG(Int16ToStr(recvSet.address[3]));
	DBG(Int16ToStr(recvSet.address[4]));
	DBG("HOPPING:");
	DBG(Int16ToStr(recvSet.hopping[0]));
	DBG(Int16ToStr(recvSet.hopping[1]));
	DBG(Int16ToStr(recvSet.hopping[2]));
	DBG(Int16ToStr(recvSet.hopping[3]));
	DBG(Int16ToStr(recvSet.hopping[4]));
	DBG("REG stat:");
	DBG(Int16ToStr(NRF_REG_read(0x00)));
	DBG(Int16ToStr(NRF_REG_read(0x01)));
	DBG(Int16ToStr(NRF_REG_read(0x02)));
	DBG(Int16ToStr(NRF_REG_read(0x03)));
	DBG(Int16ToStr(NRF_REG_read(0x04)));
	DBG(Int16ToStr(NRF_REG_read(0x05)));
	DBG(Int16ToStr(NRF_REG_read(0x06)));
	DBG(Int16ToStr(NRF_REG_read(0x07)));
	DBG("REG +8");
	DBG(Int16ToStr(NRF_REG_read(0x08)));
	DBG(Int16ToStr(NRF_REG_read(0x09)));
	DBG(Int16ToStr(NRF_REG_read(0x0a)));
	DBG(Int16ToStr(NRF_REG_read(0x0b)));
	DBG(Int16ToStr(NRF_REG_read(0x0c)));
	DBG(Int16ToStr(NRF_REG_read(0x0d)));
	DBG(Int16ToStr(NRF_REG_read(0x0e)));
	DBG(Int16ToStr(NRF_REG_read(0x0f)));
	DBG("REG +8");
	DBG(Int16ToStr(NRF_REG_read(0x10)));
	DBG(Int16ToStr(NRF_REG_read(0x11)));
	DBG(Int16ToStr(NRF_REG_read(0x12)));
	DBG(Int16ToStr(NRF_REG_read(0x13)));
	DBG("REG +4");
	DBG("REG end.");
#endif
}

/*////////////////////////////////////////////////////////////////////////////*/
	
#if 1
u8 temp=0;
#endif

void loop (void)
{
  u16 buff[8];
  u8 i;
  static u16 t1,t2;

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

    switch(recv.stateNRF)
    {
      case STATE_NRF_RX:
        if(noRF()) break;

        if(recv.stateLED ==STATE_LED_ON) recv.stateLED = STATE_LED_OFF;

        nrf.rx_cnt++;
        nrf.hopping_cnt=0;//收到有效信号后刷新跳频器

        timer_startOnce(TIMER_SIGNAL_LOST, 2000);
        timer_startCycle(TIMER_CHANNEL_LOST, 22); //3*7~3*8

        NRF_FIFO_read(nrf.rx, LOLI3_NRF_DATA_LENGTH);   //读取接收数据
        NRF_irq_clean();

        NRF_mode_tx();
        recv.stateNRF = STATE_NRF_TX;
        //break;
      case STATE_NRF_TX:
        nrf.tx[0]=recv.rx_num;
        // nrf.tx[0]=20;	// for test
#if 1
        if(temp!=recv.rx_num)
        {
          temp = recv.rx_num;
          DBG("rx num: ");
          DBG(Int16ToStr(temp));
        }
#endif

        recv.voltage_ic=VOLTAGE_IC_ADJUST/_adc_ic;
        // recv.voltage_ic=500;	// for test
        nrf.tx[1]=recv.voltage_ic>>8;
        nrf.tx[2]=recv.voltage_ic;
        // >>13 表示10位ADC占比再除以8（_adc_batt放大系数）
				recv.voltage_batt=((long)_adc_batt*recv.voltage_ic*VOLTAGE_BAT_MULTIPLE)>>13;
        // recv.voltage_batt=500;	// for test
        nrf.tx[3]=recv.voltage_batt>>8;
        nrf.tx[4]=recv.voltage_batt;
        NRF_FIFO_write(nrf.tx, LOLI3_NRF_DATA_LENGTH);

        recv.stateNRF = STATE_NRF_TX_WAIT;
        t1 = micros();
        //break;
      case STATE_NRF_TX_WAIT:
        t2 = micros()-t1;
        if(!noRF() || t2>=1000) // wait about 0.72ms for TX_DS irq
        {
          //if(!noRF())  DBG("tx finished.");
          if(t2>=1000) DBG(Int16ToStr(t2)); // timeout
          NRF_mode_rx();  // NRF_irq_clean(); change mode would clean irq.
          recv.stateNRF = STATE_NRF_DATA_PROC;
        }
        break;	// 未收到数据终端且未超时，让出CPU时间。
      case STATE_NRF_DATA_PROC:
        nrf.channel_index++;
        if(nrf.channel_index>4)nrf.channel_index=0;
        NRF_channel(recvSet.hopping[nrf.channel_index]);

        if(nrf.rx[0]==0xa2)
        {
          if(nrf.rx[1]&0x80)  { recvSet.PPM=1;  }
          else                { recvSet.PPM=0;  }
          if(nrf.rx[1]&0x40)  { recvSet.SBUS=1;     }
          else                { recvSet.SBUS=0;     }
          if(nrf.rx[1]&0x08)  { recvSet.CH1_PWM=1; hw_pwm_en(0); }
          else                { recvSet.CH1_PWM=0; hw_pwm_dis(0);}
          if(nrf.rx[1]&0x04)  { recvSet.CH2_PWM=1; hw_pwm_en(1); }
          else                { recvSet.CH2_PWM=0; hw_pwm_dis(1);}
#if STC_8H
          if(nrf.rx[1]&0x02)  { recvSet.CH7_PWM=1; hw_pwm_en(6); }//recvSet.CH7_PWM=1;  }	// STC15W取消，STC8H恢复
          else                { recvSet.CH7_PWM=0; hw_pwm_dis(6); }//recvSet.CH7_PWM=0;  }
          if(nrf.rx[1]&0x01)  { recvSet.CH6_PWM=1; hw_pwm_en(5); }//recvSet.CH6_PWM=1;  }	// STC8H扩展，需要修改发射机程序
          else                { recvSet.CH6_PWM=0; hw_pwm_dis(5); }//recvSet.CH6_PWM=0;  }
#endif
          if(nrf.rx[2]&0x80)  { recvSet.CH1_SW=1;   }
          else                { recvSet.CH1_SW=0;   }
          if(nrf.rx[2]&0x40)  { recvSet.CH2_SW=1;   }
          else                { recvSet.CH2_SW=0;   }
          if(nrf.rx[2]&0x20)  { recvSet.CH3_SW=1;   }
          else                { recvSet.CH3_SW=0;   }
          if(nrf.rx[2]&0x10)  { recvSet.CH4_SW=1;   }
          else                { recvSet.CH4_SW=0;   }
          if(nrf.rx[2]&0x08)  { recvSet.CH5_SW=1;   }
          else                { recvSet.CH5_SW=0;   }
          if(nrf.rx[2]&0x04)  { recvSet.CH6_SW=1;   }
          else                { recvSet.CH6_SW=0;   }
          if(nrf.rx[2]&0x02)  { recvSet.CH7_SW=1;   }
          else                { recvSet.CH7_SW=0;   }
          if(nrf.rx[2]&0x01)  { recvSet.CH8_SW=1;   }
          else                { recvSet.CH8_SW=0;   }

          recv.isOutputChanged = 1;
					DBG("OUTPUT mode changed");
          timer_startOnce(TIMER_DATA_SAVE, 1000); // save data 1000ms later
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

            timer_startOnce(TIMER_DATA_SAVE, 1000); // save data 1000ms later
          }
          else  // nrf.rx[0]==0xa1
          {
            for(i=0; i<8; i++)
            {
              DATA_mutex_get();   // 防止中断程序输出错误数据
              recv.CH_data[i]=buff[i];
              DATA_mutex_release();
            }
            SBUS_data_push();
          }
      }

      recv.stateNRF = STATE_NRF_RX;
      break;
    default:
      //NRF_mode_rx();
      recv.stateNRF = STATE_NRF_RX;
      break;
    }
}

/******************************************************************************/
/*-DO NOT ADD YOUR CODE AFTER HERE!-*\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-WIDTH: 80-*/

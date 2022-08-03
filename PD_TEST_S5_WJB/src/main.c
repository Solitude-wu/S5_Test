/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/05/18 5:01p $
 * @brief    NUC230_240 Series I2C Driver Sample Code (Master)
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NUC230_240.h"
#include "bsp.h"
#include "common.h"
#include "gb_t19056_def.h"
#include "AD9837.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

extern volatile uint8_t print_onoff;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

	/* Enable Internal RC 22.1184 MHz clock */
	CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

	/* Waiting for Internal RC clock ready */
	CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

	/* Enable external 12 MHz XTAL */
	CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

	/* Waiting for clock ready */
	CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    
    /* Enable WDT module clock */
	CLK_EnableModuleClock(WDT_MODULE);

	/* Enable clock output */
	CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 3, 0);
    
    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);
    
    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    //CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HCLK, CLK_CLKDIV_ADC(250));
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_HXT, CLK_CLKDIV_ADC(119)); /* 12M ext/120=100KHz采样 */

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

    SysTick_Config(1000*CyclesPerUs);
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 3);
    NVIC_EnableIRQ(SysTick_IRQn);
}


uint8_t g_sleep_flag = 0;
uint8_t g_last_sleep_flag = 0;
int prepare_wakeup_delay = 0;
int prepare_sleep_delay = 0;

static uint8_t heapbuf[10*1024];

uint16_t sleep_current[8];
uint32_t sleep_current_cnt;
static uint32_t send_sleep_current_cnt = 0;
static uint16_t sleep_curr;

void send_power_info_by_can(void)
{
	CanMsgQueue_t *pCanMsg;
	uint16_t volt, curr, i;

	volt = ADC_GetChanVolt(ADC_CH_VIN);
	curr = ADC_GetChanVolt(ADC_CH_IOUT);
	
	if (g_sleep_flag == 1) {													
		sleep_current[(sleep_current_cnt++) & 0x7] = curr;
	}
	
	if (g_last_sleep_flag != g_sleep_flag) {						
		if (g_sleep_flag == 0) { /* wakeup from sleep  */
			send_sleep_current_cnt = 10;
			sleep_curr = 0;
			if (sleep_current_cnt < 8) {
				for (i=0; i<sleep_current_cnt; i++) {
					sleep_curr += sleep_current[i];
				}
				sleep_curr /= sleep_current_cnt;
			} else {
				for (i=0; i<8; i++) {
					sleep_curr += sleep_current[i];
				}
				sleep_curr >>= 3;
			}
		} else { /* enter sleep */									
			sleep_curr = 0;							
			sleep_current_cnt = 0;
			memset(sleep_current, 0, sizeof(sleep_current));
			send_sleep_current_cnt = 0;
		}
		g_last_sleep_flag = g_sleep_flag;
	}
	
	pCanMsg = CanAllocMsg(0, &pCanSendQHead[0]);
	if (pCanMsg) {
		pCanMsg->msg.Id = 0x234;
		pCanMsg->msg.FrameType = CAN_DATA_FRAME;
		pCanMsg->msg.IdType = CAN_STD_ID;
		pCanMsg->msg.DLC = 8;
		
		pCanMsg->msg.Data[0] = send_sleep_current_cnt ? 1:0;
		pCanMsg->msg.Data[1] = volt >> 8;
		pCanMsg->msg.Data[2] = volt;
		if (send_sleep_current_cnt) {
			curr = sleep_curr;
		}
		pCanMsg->msg.Data[3] = curr >> 8;
		pCanMsg->msg.Data[4] = curr;
		pCanMsg->msg.Data[5] = 0;
		pCanMsg->msg.Data[6] = 0;
		pCanMsg->msg.Data[7] = 0;

		CanMsgQueueSortedInsert(&pCanSendQHead[0], pCanMsg);
		CAN_TriggerQueueSend(0);
	} else {
		printf("CAN1 send queue is full, drop packet\n");
	}
	if (send_sleep_current_cnt > 0)
		send_sleep_current_cnt--;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	uint32_t sys_time, last_time;
	uint16_t radar_snd_cnt = 0, can_snd_cnt = 5;
	uint8_t ign_stat = 0, last_ign_stat = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); 

    /* Lock protected registers */
    SYS_LockReg();
	
	/* WDT时钟10KHz，超时时间2^14 / 10k = 1.64s */
    WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_130CLK, TRUE, FALSE);
	
    TimersInit();

    BoardIoInit();

    /* 加点延时避免RS232电平转换芯片刚上电时串口打印乱码 */
    mdelay(5);

    BoardComInit();

    printf("\n\n\n\nSystem startup\n");

    ADC_SampleInit();
	
	/* CAN Strobe */
	//GPIO_SetMode(PE, BIT2|BIT3, GPIO_PMD_OUTPUT);
	//PE2 = 0;
	//PE3 = 0;

    CAN_Init(0, 500000);
    //CAN_SetRecvId(0, COMMUNICATION_CAN_ID|0x80000000);
	//CAN_Init(1, 500000);
	
	//LedBlink(0, 50, 50, 0xffff, 0);
	//LedBlink(1, 50, 50, 0xffff, 0);

    printf("OK!\n");

	WDT_Feed();
    while (1) {
		sys_time = GetRunTime();
		if (sys_time - last_time > 10) {
			last_time = sys_time;								/******************序号代表程序执行到第几次*******************/
			if (IGN_IN_SIG == IGN_IN_SIG_LEVEL) {				//1、IGN_IN_SIG = 0( 接地 )												  		  //152、IGN_IN_SIG = 0( 再次接地 )
				IGN_OUT_SIG = IGN_OUT_SIG_LEVEL;				//1、点火( IGN_OUT_SIG = 1 )													  //152、再次点火( IGN_OUT_SIG = 1 )
			} else {																				//(2~151)、IGN_IN_SIG = 1( 取消接地 )
				IGN_OUT_SIG = !IGN_OUT_SIG_LEVEL;													//(2~151)、停止点火( IGN_OUT_SIG = 0 )
				if (prepare_sleep_delay == 0)
					prepare_sleep_delay = 150;														//(2~151)、装载 准备休眠计数器计数值
			}
			ign_stat = IGN_IN_SIG == IGN_IN_SIG_LEVEL ? 1:0;  	//1、ign_stat = 1(IGN_IN_SIG = 0 )	//(2~151)、ign_stat = 0( IGN_IN_SIG = 1 )	  //152、ign_stat = 1(IGN_IN_SIG = 0 )
			if (last_ign_stat != ign_stat) {					//1、	0 != 1						//(2~151)、	1 != 0						  	  //152、0 != 1
				//last_ign_stat = ign_stat;
				if (ign_stat == 1) {
					g_sleep_flag = 0;							//1、休眠标志置0																  //152、休眠标志置0
					last_ign_stat = ign_stat;					//1、last_ign_stat = 1															  //152、last_ign_stat = 1
				} else {
					if (prepare_sleep_delay > 0) {													//(2~151)、减 准备休眠计数器计数值
						prepare_sleep_delay--;
						if (prepare_sleep_delay == 0) {																			//151、若计数器计数值减为0
							g_sleep_flag = 1;																					//151、则置 休眠标志位为1
							last_ign_stat = ign_stat;																			//151、last_ign_stat = 0
						}
					}
				}
			}
			
			if (radar_snd_cnt++ > 10) {
				SC2SendRadarData();
				radar_snd_cnt = 0;
			}
			
			if (can_snd_cnt++ > 10) {
				send_power_info_by_can();
				can_snd_cnt = 0;
			}
			WDT_Feed();
		}
    }
}



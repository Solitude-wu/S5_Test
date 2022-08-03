#ifndef __BSP_H
#define __BSP_H


#define _STR(s) #s
#define STR(s) _STR(s)

#define MCU_SOFTWARE_VERSION HARDWARE_MODEL"_"STR(HARDWARE_VERSION)"."STR(SOFTWARE_VERSION)

#define PA_0             ((0<<24) | (0 <<16) | (0x00000001)) /*!< Specify PA.0 Pin Data Input/Output */
#define PA_1             ((0<<24) | (1 <<16) | (0x00000002)) /*!< Specify PA.1 Pin Data Input/Output */
#define PA_2             ((0<<24) | (2 <<16) | (0x00000004)) /*!< Specify PA.2 Pin Data Input/Output */
#define PA_3             ((0<<24) | (3 <<16) | (0x00000008)) /*!< Specify PA.3 Pin Data Input/Output */
#define PA_4             ((0<<24) | (4 <<16) | (0x00000010)) /*!< Specify PA.4 Pin Data Input/Output */
#define PA_5             ((0<<24) | (5 <<16) | (0x00000020)) /*!< Specify PA.5 Pin Data Input/Output */
#define PA_6             ((0<<24) | (6 <<16) | (0x00000040)) /*!< Specify PA.6 Pin Data Input/Output */
#define PA_7             ((0<<24) | (7 <<16) | (0x00000080)) /*!< Specify PA.7 Pin Data Input/Output */
#define PA_8             ((0<<24) | (8 <<16) | (0x00000100)) /*!< Specify PA.8 Pin Data Input/Output */
#define PA_9             ((0<<24) | (9 <<16) | (0x00000200)) /*!< Specify PA.9 Pin Data Input/Output */
#define PA_10            ((0<<24) | (10<<16) | (0x00000400)) /*!< Specify PA.10 Pin Data Input/Output */
#define PA_11            ((0<<24) | (11<<16) | (0x00000800)) /*!< Specify PA.11 Pin Data Input/Output */
#define PA_12            ((0<<24) | (12<<16) | (0x00001000)) /*!< Specify PA.12 Pin Data Input/Output */
#define PA_13            ((0<<24) | (13<<16) | (0x00002000)) /*!< Specify PA.13 Pin Data Input/Output */
#define PA_14            ((0<<24) | (14<<16) | (0x00004000)) /*!< Specify PA.14 Pin Data Input/Output */
#define PA_15            ((0<<24) | (15<<16) | (0x00008000)) /*!< Specify PA.15 Pin Data Input/Output */
#define PB_0             ((1<<24) | (0 <<16) | (0x00000001)) /*!< Specify PB.0 Pin Data Input/Output */
#define PB_1             ((1<<24) | (1 <<16) | (0x00000002)) /*!< Specify PB.1 Pin Data Input/Output */
#define PB_2             ((1<<24) | (2 <<16) | (0x00000004)) /*!< Specify PB.2 Pin Data Input/Output */
#define PB_3             ((1<<24) | (3 <<16) | (0x00000008)) /*!< Specify PB.3 Pin Data Input/Output */
#define PB_4             ((1<<24) | (4 <<16) | (0x00000010)) /*!< Specify PB.4 Pin Data Input/Output */
#define PB_5             ((1<<24) | (5 <<16) | (0x00000020)) /*!< Specify PB.5 Pin Data Input/Output */
#define PB_6             ((1<<24) | (6 <<16) | (0x00000040)) /*!< Specify PB.6 Pin Data Input/Output */
#define PB_7             ((1<<24) | (7 <<16) | (0x00000080)) /*!< Specify PB.7 Pin Data Input/Output */
#define PB_8             ((1<<24) | (8 <<16) | (0x00000100)) /*!< Specify PB.8 Pin Data Input/Output */
#define PB_9             ((1<<24) | (9 <<16) | (0x00000200)) /*!< Specify PB.9 Pin Data Input/Output */
#define PB_10            ((1<<24) | (10<<16) | (0x00000400)) /*!< Specify PB.10 Pin Data Input/Output */
#define PB_11            ((1<<24) | (11<<16) | (0x00000800)) /*!< Specify PB.11 Pin Data Input/Output */
#define PB_12            ((1<<24) | (12<<16) | (0x00001000)) /*!< Specify PB.12 Pin Data Input/Output */
#define PB_13            ((1<<24) | (13<<16) | (0x00002000)) /*!< Specify PB.13 Pin Data Input/Output */
#define PB_14            ((1<<24) | (14<<16) | (0x00004000)) /*!< Specify PB.14 Pin Data Input/Output */
#define PB_15            ((1<<24) | (15<<16) | (0x00008000)) /*!< Specify PB.15 Pin Data Input/Output */
#define PC_0             ((2<<24) | (0 <<16) | (0x00000001)) /*!< Specify PC.0 Pin Data Input/Output */
#define PC_1             ((2<<24) | (1 <<16) | (0x00000002)) /*!< Specify PC.1 Pin Data Input/Output */
#define PC_2             ((2<<24) | (2 <<16) | (0x00000004)) /*!< Specify PC.2 Pin Data Input/Output */
#define PC_3             ((2<<24) | (3 <<16) | (0x00000008)) /*!< Specify PC.3 Pin Data Input/Output */
#define PC_4             ((2<<24) | (4 <<16) | (0x00000010)) /*!< Specify PC.4 Pin Data Input/Output */
#define PC_5             ((2<<24) | (5 <<16) | (0x00000020)) /*!< Specify PC.5 Pin Data Input/Output */
#define PC_6             ((2<<24) | (6 <<16) | (0x00000040)) /*!< Specify PC.6 Pin Data Input/Output */
#define PC_7             ((2<<24) | (7 <<16) | (0x00000080)) /*!< Specify PC.7 Pin Data Input/Output */
#define PC_8             ((2<<24) | (8 <<16) | (0x00000100)) /*!< Specify PC.8 Pin Data Input/Output */
#define PC_9             ((2<<24) | (9 <<16) | (0x00000200)) /*!< Specify PC.9 Pin Data Input/Output */
#define PC_10            ((2<<24) | (10<<16) | (0x00000400)) /*!< Specify PC.10 Pin Data Input/Output */
#define PC_11            ((2<<24) | (11<<16) | (0x00000800)) /*!< Specify PC.11 Pin Data Input/Output */
#define PC_12            ((2<<24) | (12<<16) | (0x00001000)) /*!< Specify PC.12 Pin Data Input/Output */
#define PC_13            ((2<<24) | (13<<16) | (0x00002000)) /*!< Specify PC.13 Pin Data Input/Output */
#define PC_14            ((2<<24) | (14<<16) | (0x00004000)) /*!< Specify PC.14 Pin Data Input/Output */
#define PC_15            ((2<<24) | (15<<16) | (0x00008000)) /*!< Specify PC.15 Pin Data Input/Output */
#define PD_0             ((3<<24) | (0 <<16) | (0x00000001)) /*!< Specify PD.0 Pin Data Input/Output */
#define PD_1             ((3<<24) | (1 <<16) | (0x00000002)) /*!< Specify PD.1 Pin Data Input/Output */
#define PD_2             ((3<<24) | (2 <<16) | (0x00000004)) /*!< Specify PD.2 Pin Data Input/Output */
#define PD_3             ((3<<24) | (3 <<16) | (0x00000008)) /*!< Specify PD.3 Pin Data Input/Output */
#define PD_4             ((3<<24) | (4 <<16) | (0x00000010)) /*!< Specify PD.4 Pin Data Input/Output */
#define PD_5             ((3<<24) | (5 <<16) | (0x00000020)) /*!< Specify PD.5 Pin Data Input/Output */
#define PD_6             ((3<<24) | (6 <<16) | (0x00000040)) /*!< Specify PD.6 Pin Data Input/Output */
#define PD_7             ((3<<24) | (7 <<16) | (0x00000080)) /*!< Specify PD.7 Pin Data Input/Output */
#define PD_8             ((3<<24) | (8 <<16) | (0x00000100)) /*!< Specify PD.8 Pin Data Input/Output */
#define PD_9             ((3<<24) | (9 <<16) | (0x00000200)) /*!< Specify PD.9 Pin Data Input/Output */
#define PD_10            ((3<<24) | (10<<16) | (0x00000400)) /*!< Specify PD.10 Pin Data Input/Output */
#define PD_11            ((3<<24) | (11<<16) | (0x00000800)) /*!< Specify PD.11 Pin Data Input/Output */
#define PD_12            ((3<<24) | (12<<16) | (0x00001000)) /*!< Specify PD.12 Pin Data Input/Output */
#define PD_13            ((3<<24) | (13<<16) | (0x00002000)) /*!< Specify PD.13 Pin Data Input/Output */
#define PD_14            ((3<<24) | (14<<16) | (0x00004000)) /*!< Specify PD.14 Pin Data Input/Output */
#define PD_15            ((3<<24) | (15<<16) | (0x00008000)) /*!< Specify PD.15 Pin Data Input/Output */
#define PE_0             ((4<<24) | (0 <<16) | (0x00000001)) /*!< Specify PE.0 Pin Data Input/Output */
#define PE_1             ((4<<24) | (1 <<16) | (0x00000002)) /*!< Specify PE.1 Pin Data Input/Output */
#define PE_2             ((4<<24) | (2 <<16) | (0x00000004)) /*!< Specify PE.2 Pin Data Input/Output */
#define PE_3             ((4<<24) | (3 <<16) | (0x00000008)) /*!< Specify PE.3 Pin Data Input/Output */
#define PE_4             ((4<<24) | (4 <<16) | (0x00000010)) /*!< Specify PE.4 Pin Data Input/Output */
#define PE_5             ((4<<24) | (5 <<16) | (0x00000020)) /*!< Specify PE.5 Pin Data Input/Output */
#define PE_6             ((4<<24) | (6 <<16) | (0x00000040)) /*!< Specify PE.6 Pin Data Input/Output */
#define PE_7             ((4<<24) | (7 <<16) | (0x00000080)) /*!< Specify PE.7 Pin Data Input/Output */
#define PE_8             ((4<<24) | (8 <<16) | (0x00000100)) /*!< Specify PE.8 Pin Data Input/Output */
#define PE_9             ((4<<24) | (9 <<16) | (0x00000200)) /*!< Specify PE.9 Pin Data Input/Output */
#define PE_10            ((4<<24) | (10<<16) | (0x00000400)) /*!< Specify PE.10 Pin Data Input/Output */
#define PE_11            ((4<<24) | (11<<16) | (0x00000800)) /*!< Specify PE.11 Pin Data Input/Output */
#define PE_12            ((4<<24) | (12<<16) | (0x00001000)) /*!< Specify PE.12 Pin Data Input/Output */
#define PE_13            ((4<<24) | (13<<16) | (0x00002000)) /*!< Specify PE.13 Pin Data Input/Output */
#define PE_14            ((4<<24) | (14<<16) | (0x00004000)) /*!< Specify PE.14 Pin Data Input/Output */
#define PE_15            ((4<<24) | (15<<16) | (0x00008000)) /*!< Specify PE.15 Pin Data Input/Output */
#define PF_0             ((5<<24) | (0 <<16) | (0x00000001)) /*!< Specify PF.0 Pin Data Input/Output */
#define PF_1             ((5<<24) | (1 <<16) | (0x00000002)) /*!< Specify PF.1 Pin Data Input/Output */
#define PF_2             ((5<<24) | (2 <<16) | (0x00000004)) /*!< Specify PF.2 Pin Data Input/Output */
#define PF_3             ((5<<24) | (3 <<16) | (0x00000008)) /*!< Specify PF.3 Pin Data Input/Output */

#define GpioToNum(x)	(((x)>>16)&0x0ff)
#define GpioToBit(x)    ((x)&0x0FFFF)


/* 时钟相关 */
#define PLLCON_SETTING          CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK               72000000
#define RTC_TICKS_PER_SECOND    32

/* LED 相关定义 */
#define LED_NUM 2

/* 蜂鸣器配置 */
#define BUZZER_PWM_CHN		PWM_CH1
#define BUZZER_PWM_MODULE	PWM45_MODULE
#define BUZZER_PWM_CLK_SRC	CLK_CLKSEL2_PWM45_EXT_HXT


#define IO_IN1_SIG			PD2
#define IO_IN1_PIN			PD_2
#define IO_IN2_SIG			PD3
#define IO_IN2_PIN			PD_3
#define IGN_OUT_SIG			PC6
#define IGN_OUT_PIN			PC_6
#define IGN_OUT_SIG_LEVEL	1
#define IGN_IN_SIG			PA11
#define IGN_IN_PIN			PA_11
#define IGN_IN_SIG_LEVEL	0

#define PWR_TRIGGER			PC3
#define PWR_TRIGGER_PIN		PC_3
#define PWR_TRIGGER_LEVEL	0


/* ADC引脚配置 */
/* 使能连续转换,ADC启动后会消耗大约1.5mA电流，考虑到低功耗情况暂不开启连续转换 */
#define CONFIG_ADC_CONTINUOUS 1

#define ADC_CH_VCC1V8		3
#define ADC_CH_VCC3V3		4
#define ADC_CH_VCC1V		5
#define ADC_CH_IOUT			6
#define ADC_CH_VIN			7
#define ADC_CH_VCC5V		1
#define ADC_CH_MCU3V3		0
#define ADC_CH_VCC1V5		2

#define ADC_VREF			3300
#define ADC_SCALE			((1<<12)-1)
#define ADC_RATIO_CH0		2
#define ADC_RATIO_CH1		2
#define ADC_RATIO_CH2		2
#define ADC_RATIO_CH3		6
#define ADC_RATIO_CH4		6
#define ADC_RATIO_CH5		6
#define ADC_RATIO_CH6		1
#define ADC_RATIO_CH7		10


#define HW_DELAY_TIMER			TIMER1
#define HW_DELAY_TIMER_MODULE	TMR1_MODULE

#define HW_CLOCK_TIMER			TIMER2
#define HW_CLOCK_TIMER_MODULE	TMR2_MODULE

#define SW_TIMER_NUM			5		/* 软件定时器的个数 */
#define IsValidTimerId(id)		(id < SW_TIMER_NUM)

#define IO_CHANNEL_ACC		0
#define IO_CHANNEL_BRAKE	1
#define IO_CHANNEL_SOS		2
#define IO_CHANNEL_LTURN	3
#define IO_CHANNEL_RTURN	4
#define IO_CHANNEL_LOWBEAM	5
#define IO_CHANNEL_HIGHBEAM	6
#define IO_CHANNEL_IOIN1	7
#define IO_CHANNEL_IOIN2	8
#define IO_CHANNEL_IOOUT1	9
#define IO_CHANNEL_IOOUT2	10

#endif



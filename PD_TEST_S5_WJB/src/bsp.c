#include "NUC230_240.h"
#include "stdio.h"
#include "string.h"
#include "common.h"
#include "bsp.h"
#include "gb_t19056_def.h"

#ifndef BUILD_BOOTLOADER

extern uint8_t test_mode;

/***********************************
// 时间相关接口
************************************/

/* 定于软件定时器结构体变量 */
static SOFT_TMR s_tTmr[SW_TIMER_NUM] = {0};

static volatile uint32_t s_msCounter = 0;
static volatile uint32_t s_uiDelayCounter = 0;
static volatile uint8_t s_ucDelayTimeoutFlag = 0;

volatile uint8_t g_rtcInitFailedFlag = 0;
volatile uint32_t g_u32RTCTickINT;
volatile uint32_t g_u32SecondCnt;
S_RTC_TIME_DATA_T g_sCurRtcTime;
S_RTC_TIME_DATA_T g_NewRtcTime;

struct ledBlinkCfg ledCfg[LED_NUM] = {
		//{&TST_SIGNAL, 1, 0},
		//{&BRAKE_SIG, 1, 0},
		//{&SOS_SIG, 1, 0},
		//{&LEFT_TURN_SIG, 1, 0},
		//{&RIGHT_TURN_SIG, 1, 0},
		//{&DBL_SIG, 1, 0},
		//{&HBL_SIG, 1, 0},
		{&IO_IN1_SIG, 1, 0},
		{&IO_IN2_SIG, 1, 0},
		//{&SPD_PULSE_SIG, 1, 0},
		//{&LED_RUN, LED_RUN_ON_LEVEL, 0},
		//{&LED_STAT, LED_STAT_ON_LEVEL, 0},
};

static inline void SoftTimerDec(SOFT_TMR *tmr);

static uint32_t g_interruptDisableCount = 0;
void DisableInterrupt(void)
{
	__disable_irq();
	g_interruptDisableCount++;
}

void EnableInterrupt(void)
{
	/* Check and update */
    if (g_interruptDisableCount > 0)
    {
        g_interruptDisableCount--;

        if (g_interruptDisableCount <= 0)
        {
            /* Enable the global interrupt*/
            __enable_irq();
        }
    }
}
void SysTick_Handler(void)
{
    uint8_t i;

	s_msCounter++;

	if (s_uiDelayCounter > 0) {
		if (--s_uiDelayCounter == 0) {
			s_ucDelayTimeoutFlag = 1;
		}
	}
	for (i = 0; i < SW_TIMER_NUM; i++) {
		SoftTimerDec(&s_tTmr[i]);
	}
	for (i = 0; i < LED_NUM; i++) {
		if (ledCfg[i].cycles > 0) {
			if (ledCfg[i].counter < ledCfg[i].on_time) {
				*(ledCfg[i].pin) = ledCfg[i].on_level;
			} else if(ledCfg[i].counter < ledCfg[i].off_time) {
				*(ledCfg[i].pin) = !ledCfg[i].on_level;
			} else {
				if (ledCfg[i].on_time == 0 || ledCfg[i].off_time == 0)
					*(ledCfg[i].pin) = !ledCfg[i].on_level;
				else
					*(ledCfg[i].pin) = ledCfg[i].on_level;
				/* 如果cycles == 0xffff则常亮 */
				if (ledCfg[i].cycles != 0xffff) {
					ledCfg[i].cycles--;
					if (ledCfg[i].cycles == 0)
						*(ledCfg[i].pin) = ledCfg[i].last_stat ? ledCfg[i].on_level : !ledCfg[i].on_level;
				}
				ledCfg[i].counter = 0;
			}
			ledCfg[i].counter++;
		}
	}
}

void mdelay(uint32_t ms)
{
    if (ms == 0)
        return;
    
    __disable_irq();
    s_uiDelayCounter = ms;
    s_ucDelayTimeoutFlag = 0;
    __enable_irq();
    while(!s_ucDelayTimeoutFlag){}
}

#ifdef UDELAY_USE_SYSTICK
void udelay(uint32_t us)
{
    uint32_t start;
    uint32_t end;
    start = SysTick->VAL;
    us *= CyclesPerUs;
    
    if (start >= us) {
        end = start - us;
    } else {
        end = SysTick->LOAD - (us - start);
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
    while (SysTick->VAL > end);
}
#else
void udelay(uint32_t us)
{
	if (us == 0) return;
	TIMER_SET_CMP_VALUE(HW_DELAY_TIMER, us*CyclesPerUs);
	TIMER_Start(HW_DELAY_TIMER);
	/* wait for timeout flag */
	while(!(HW_DELAY_TIMER->TISR & TIMER_TISR_TIF_Msk));
	/* Wirte to clear interrupt flag */
	HW_DELAY_TIMER->TISR |= TIMER_TISR_TIF_Msk;
	TIMER_Stop(HW_DELAY_TIMER);
}

#endif

static inline void SoftTimerDec(SOFT_TMR *tmr)
{
	if (tmr->Inuse && tmr->Count > 0)
	{
		/* 如果定时器变量减到1则设置定时器到达标志 */
		if (--tmr->Count == 0)
		{
			tmr->Flag = 1;
            tmr->callback(tmr->callback_param);
			/* 如果是自动模式，则自动重装计数器 */
			if(tmr->Mode == TMR_AUTO_MODE)
			{
				tmr->Count = tmr->PreLoad;
			}
		}
	}
}

void SoftTimerSetPrelaodValue(uint8_t id, uint32_t val)
{
    if (!IsValidTimerId(id))
	{
		/* 打印出错的源代码文件名、函数名称 */
		printf("%s() invalid id=%d\r\n", __FUNCTION__, id);
		return;
	}
    s_tTmr[id].PreLoad = val;
}


void SoftTimerInit(void)
{
	uint8_t i;

	/* 清零所有的软件定时器 */
	for (i = 0; i < SW_TIMER_NUM; i++)
	{
		s_tTmr[i].Inuse = 0;
		s_tTmr[i].Count = 0;
		s_tTmr[i].PreLoad = 0;
		s_tTmr[i].Flag = 0;
		s_tTmr[i].Mode = TMR_ONCE_MODE;	/* 缺省是1次性工作模式 */
		s_tTmr[i].callback = 0;
	}
}

uint8_t SoftTimerAlloc(void)
{
	uint8_t ret = 0xff, i;
	__disable_irq();
	for (i = 0; i < SW_TIMER_NUM; i++) {
		if (0 == s_tTmr[i].Inuse) {
			ret = i;
			s_tTmr[i].Inuse = 1;
			break;
		}
	}
	__enable_irq();
	return ret;
}

void SoftTimerFree(uint8_t *id)
{
	if (*id > SW_TIMER_NUM)
		return;

	__disable_irq();
	memset(&s_tTmr[*id], 0, sizeof(SOFT_TMR));
	*id = 0xff;
	__enable_irq();
}

void SoftTimerStart(uint8_t id, uint32_t period, TMR_MODE_E mode, TimerCallback_t func, int param)
{
	if (!IsValidTimerId(id))
	{
		/* 打印出错的源代码文件名、函数名称 */
		printf("%s() invalid id=%d\r\n", __FUNCTION__, id);
		return;
	}
	
	if (NULL != s_tTmr[id].callback && s_tTmr[id].PreLoad != 0) {
		/* already started */
		return;
	}

	__disable_irq(); 			/* 关中断 */

	s_tTmr[id].Count = period;		/* 实时计数器初值 */
	s_tTmr[id].PreLoad = period;		/* 计数器自动重装值，仅自动模式起作用 */
	s_tTmr[id].Flag = 0;				/* 定时时间到标志 */
	s_tTmr[id].Mode = mode;	/* 1次性工作模式 */
    s_tTmr[id].callback = func;
    s_tTmr[id].callback_param = param;

	__enable_irq();  				/* 开中断 */
}

void SoftTimerStop(uint8_t id)
{
	if (!IsValidTimerId(id))
	{
		/* 打印出错的源代码文件名、函数名称 */
		printf("%s() invalid id=%d\r\n", __FUNCTION__, id);
		return;
	}

	__disable_irq();  	/* 关中断 */

	memset(&s_tTmr[id], 0, sizeof(s_tTmr[id]));
	
	__enable_irq();  		/* 开中断 */
}

void SoftTimerReload(uint8_t id)
{
	if (!IsValidTimerId(id))
	{
		/* 打印出错的源代码文件名、函数名称 */
		printf("%s() invalid id=%d\r\n", __FUNCTION__, id);
		return;
	}

	__disable_irq();  	/* 关中断 */

	s_tTmr[id].Count = s_tTmr[id].PreLoad;
	s_tTmr[id].Flag = 0;
	__enable_irq();  		/* 开中断 */
}

int SoftTimerCheckTimeout(uint8_t id)
{
	if (!IsValidTimerId(id))
	{
		/* 打印出错的源代码文件名、函数名称 */
		printf("%s() invalid id=%d\r\n", __FUNCTION__, id);
		return 1;
	}

	if (s_tTmr[id].Flag == 1)
	{
		s_tTmr[id].Flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

uint32_t GetRunTime(void)
{
	uint32_t runtime;

	__disable_irq();  	/* 关中断 */

	runtime = s_msCounter;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

	__enable_irq();  		/* 开中断 */

	return runtime;
}

uint32_t CompareRunTime(uint32_t _LastTime)
{
	uint32_t now_time;
	uint32_t time_diff;

	__disable_irq();  	/* 关中断 */

	now_time = s_msCounter;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

	__enable_irq();  		/* 开中断 */
	
	if (now_time >= _LastTime)
	{
		time_diff = now_time - _LastTime;
	}
	else
	{
		time_diff = 0xFFFFFFFF - _LastTime + now_time;
	}

	return time_diff;
}

void HardwareTimerInit(void)
{
	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Enable timer module clock */
	CLK_EnableModuleClock(HW_DELAY_TIMER_MODULE);

	/* Select timer module clock source */
	CLK_SetModuleClock(HW_DELAY_TIMER_MODULE, CLK_CLKSEL1_TMR1_S_HCLK, 0);

	/* Initial delay timer to default setting */
	TIMER_Open(HW_DELAY_TIMER, TIMER_ONESHOT_MODE, 1000000);

	
	/* Enable timer module clock */
	//CLK_EnableModuleClock(HW_CLOCK_TIMER_MODULE);

	/* Select timer module clock source */
	//CLK_SetModuleClock(HW_CLOCK_TIMER_MODULE, CLK_CLKSEL1_TMR2_S_HCLK, 0);

	/* Initial delay timer to default setting */
	//TIMER_Open(HW_CLOCK_TIMER, TIMER_ONESHOT_MODE, 1000000);
	
	/* Lock protected registers */
	SYS_LockReg();
}

void TimersInit(void)
{
	SoftTimerInit();
	HardwareTimerInit();
	
	SYS_UnlockReg();
	
	SYS->GPB_MFP &= ~SYS_GPB_MFP_PB8_Msk;
	SYS->GPB_MFP |= SYS_GPB_MFP_PB8_CLKO;
	SYS->ALT_MFP &= ~SYS_ALT_MFP_PB8_Msk;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB8_CLKO;
	
	CLK_EnableCKO(CLK_CLKSEL2_FRQDIV_S_HCLK, 3, 0);
	SYS_LockReg();
	
}

//月份数据表											 
const uint8_t table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  
//平年的月份日期表
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

//判断是否是闰年函数
//输入:年份
//输出:该年份是不是闰年.1,是.0,不是
uint8_t IsLeapYear(uint16_t year)
{			  
	if ((year&0x3) == 0) { //必须能被4整除
		if (year%100 == 0) {
			if (year%400 == 0)
                return 1;//如果以00结尾,还要能被400整除 	   
			else
                return 0;   
		} else {
            return 1;
        }
	} else {
        return 0;
    }
}

uint8_t GetDayOfWeek(uint16_t year, uint8_t month, uint8_t day)
{
	uint16_t temp2;
	uint8_t yearH, yearL;
	
	yearH = year / 100;
    yearL = year % 100; 
	// 如果为21世纪,年份数加100  
	if (yearH > 19) yearL += 100;
	// 所过闰年数只算1900年之后的  
	temp2 = yearL + yearL / 4;
	temp2 = temp2 % 7; 
	temp2 = temp2 + day + table_week[month-1];
	if ((yearL&3) == 0 && month < 3) temp2--;
	return(temp2%7);
}			  

/* 注意时区问题 */
void TimestampToRtcTime(uint32_t time, S_RTC_TIME_DATA_T *rtctime)
{
	uint32_t temp = 0;
	uint16_t temp1 = 0;	   
 	temp = time / 86400;   //得到天数(秒钟数对应的)

		temp1 = 1970;	//从1970年开始
		while(temp >= 365) {				 
			if(IsLeapYear(temp1)) {//是闰年
				if (temp >= 366)
                    temp -= 366;//闰年的秒钟数
				else {temp1++;break;}  
			} else {
                temp -= 365;	  //平年
            }
			temp1++;  
		}   
		rtctime->u32Year = temp1;//得到年份
		temp1 = 0;
		while(temp >= 28)//超过了一个月
		{
			if (IsLeapYear(rtctime->u32Year) && temp1==1)//当年是不是闰年/2月份
			{
				if (temp >= 29)
                    temp-=29;//闰年的秒钟数
				else
                    break; 
			} else {
				if (temp >= mon_table[temp1])
                    temp -= mon_table[temp1];//平年
				else
                    break;
			}
			temp1++;  
		}
		rtctime->u32Month = temp1 + 1;	//得到月份
		rtctime->u32Day = temp + 1;  	//得到日期 
        
	temp = time % 86400;     		//得到秒钟数   	   
	rtctime->u32Hour = temp / 3600;     //小时
    temp1 = temp % 3600;
	rtctime->u32Minute = temp1 / 60; 	//分钟	
	rtctime->u32Second = temp1 % 60; 	//秒钟
	rtctime->u32DayOfWeek = GetDayOfWeek(rtctime->u32Year, rtctime->u32Month, rtctime->u32Day);//获取星期
}

/***********************************
// Flash 相关接口
************************************/
#if 0
int  SetDataFlashBase(uint32_t u32DFBA)
{
    uint32_t au32Config[2];
    int ret = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
    
    /* Read current User Configuration */
    FMC_ReadConfig(au32Config, 1);

    /* Just return when Data Flash has been enabled */
    if(!(au32Config[0] & 0x1))
        return 0;

    /* Enable User Configuration Update */
    FMC_EnableConfigUpdate();

    /* Erase User Configuration */
    FMC_Erase(FMC_CONFIG_BASE);

    /* Write User Configuration to Enable Data Flash */
    au32Config[0] &= ~0x1;
    au32Config[1] = u32DFBA;

    if(FMC_WriteConfig(au32Config, 2)) {
        printf("Failed to set Data Flash base address!\n");
        ret = -1;
        goto exit;
    }

    printf("\nSet Data Flash base as 0x%x.\n", FMC_ReadDataFlashBaseAddr());

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRSTC1 |= SYS_IPRSTC1_CHIP_RST_Msk;

exit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
    
    return ret;
}

static int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32Data;

    for(u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4) {
        u32Data = FMC_Read(u32Addr);
        if(u32Data != u32Pattern) {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
            return -1;
        }
    }
    return 0;
}

int FlashWritePage(uint32_t addr, void *buffer)
{
    uint32_t offset;
    uint32_t data;
    uint32_t *pbuf = buffer;
    int ret = 0;
    
    if ((addr & 0x1FF) != 0) {
        printf("Flash write address should be aligned to 512 page size!\n");
        return -1;
    }
    
    __disable_irq();
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
    
    // Erase page
    FMC_Erase(addr);
    
    // Verify if page contents are all 0xFFFFFFFF
    if(VerifyData(addr, addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0) {
        printf("\nPage 0x%x erase verify failed!\n", addr);
        ret = -1;
        goto lexit;
    }
    
    for(offset = 0; offset < FMC_FLASH_PAGE_SIZE; offset += 4) {
        FMC_Write(addr+offset, *pbuf);
        pbuf++;
    }
    
    pbuf = buffer;
    for(offset = 0; offset < FMC_FLASH_PAGE_SIZE; offset += 4) {
        data = FMC_Read(addr+offset);
        if(data != *pbuf) {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", offset+addr, data, *pbuf);
            ret = -1;
            goto lexit;
        }
        pbuf++;
    }
    
lexit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    __enable_irq();
    return ret;
}


int FlashRead(uint32_t addr, uint32_t size, void *buffer)
{
    uint32_t end = addr + size;
    uint32_t *pbuf = buffer;
    int ret = -1;
    
    if (addr & 0x3) {
        printf("Flash write address should be aligned to 4 bytes!\n");
        return -1;
    }
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
    
    while(addr < end) {
        *pbuf++ = FMC_Read(addr);
        addr += 4;
    }
    
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();
    return ret;
}

void DataFlashInit(uint32_t u32Base)
{
    uint32_t u32Data;
    
    u32Data = FMC_ReadDataFlashBaseAddr();
    
    if (u32Base != u32Data) {
        SetDataFlashBase(u32Base);
    }
}
#endif

/* 国标19056里规定的字节异或校验和 */
uint8_t GB19056_CalcCheckSum(uint8_t *buf, uint16_t len)
{
    uint8_t ret = 0;
    uint16_t i;
    
    for (i = 0; i < len; i++) {
        ret = ret^buf[i];
    }
    return ret;
}

void hi_hexdump(const void *src, size_t len, size_t width)
{
    unsigned int rows, pos, c, i;
    const char *start, *rowpos, *data;

    data = src;
    start = data;
    pos = 0;
    rows = (len % width) == 0 ? len / width : len / width + 1;
    for (i = 0; i < rows; i++)
    {
        rowpos = data;
        printf("%05x: ", pos);
        do
        {
            c = *data++ & 0xff;
            if ((size_t)(data - start) <= len)
            {
                printf(" %02x", c);
            }
            else
            {
                printf("   ");
            }
        }
        while(((data - rowpos) % width) != 0);

        printf("  |");
        data -= width;
        do
        {
            c = *data++;
            if (c < 0x20)
            {
                c = '.';
            }
            if ((size_t)(data - start) <= len)
            {
                printf("%c", c);
            }
            else
            {
                printf(" ");
            }
        }
        while(((data - rowpos) % width) != 0);
        printf("|\n");
        pos += width;
    }
}

#define	MAX_ROW	(0x8000000)

void hi_hexdump2(const void *src, size_t len, size_t width)
{
    unsigned int c, i, rows;
    const unsigned int *rowpos, *data;

    data = src;
    rows = (len % width) == 0 ? len / width : len / width + 1;

    if (MAX_ROW < rows)
    {
	printf("error:Input param(len) is greater than 2GB!\n");
	return;
    }

    for (i = 0; i < rows; i++)
    {
        rowpos = data;
        printf("%04x: ", i*0x10);
        do
        {
            c = *data++;
            printf(" %08x", c);
        }while(((data - rowpos) % 4) != 0);
        printf("\n");
    }
}

#if 0
/***********************************
// 按键相关接口
************************************/

void BoardKeyInit(void)
{
#if 1
    GPIO_SetMode(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN), GPIO_PMD_INPUT);
    GPIO_SetMode(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN), GPIO_PMD_INPUT);
    GPIO_SetMode(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN), GPIO_PMD_INPUT);
    GPIO_SetMode(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN), GPIO_PMD_INPUT);
#else
    GPIO_SetMode(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN), GPIO_PMD_QUASI);
    GPIO_SetMode(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN), GPIO_PMD_QUASI);
    GPIO_SetMode(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN), GPIO_PMD_QUASI);
    GPIO_SetMode(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN), GPIO_PMD_QUASI);
    //设置输出1以使能数字输入模式
    BOARD_KEY_MENU = 1;
    BOARD_KEY_DOWN = 1;
    BOARD_KEY_UP = 1;
    BOARD_KEY_OK = 1;
#endif
    
    SYS->GPB_MFP &= ~SYS_GPB_MFP_PB13_Msk;
    SYS->GPB_MFP |= SYS_GPB_MFP_PB13_GPIO;
    SYS->ALT_MFP &= ~SYS_ALT_MFP_PB13_Msk;
    SYS->ALT_MFP |= SYS_ALT_MFP_PB13_GPIO;
    
    GPIO_ENABLE_DEBOUNCE(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN));
    GPIO_ENABLE_DEBOUNCE(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN));
    GPIO_ENABLE_DEBOUNCE(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN));
    GPIO_ENABLE_DEBOUNCE(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN));
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, GPIO_DBCLKSEL_512);
    GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN));
    GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN));
    GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN));
    GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN));
    GPIO_EnableInt(GpioToPort(BOARD_KEY_MENU_PIN), GpioToNum(BOARD_KEY_MENU_PIN), BOARD_KEY_MENU_LEVEL ? GPIO_INT_RISING:GPIO_INT_FALLING);
    GPIO_EnableInt(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToNum(BOARD_KEY_DOWN_PIN), BOARD_KEY_DOWN_LEVEL ? GPIO_INT_RISING:GPIO_INT_FALLING);
    GPIO_EnableInt(GpioToPort(BOARD_KEY_UP_PIN), GpioToNum(BOARD_KEY_UP_PIN), BOARD_KEY_UP_LEVEL ? GPIO_INT_RISING:GPIO_INT_FALLING);
    GPIO_EnableInt(GpioToPort(BOARD_KEY_OK_PIN), GpioToNum(BOARD_KEY_OK_PIN), BOARD_KEY_OK_LEVEL ? GPIO_INT_RISING:GPIO_INT_FALLING);
    //NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(GPAB_IRQn);
    NVIC_EnableIRQ(GPCDEF_IRQn);
}

void BoardKeyDeInit(void)
{
	NVIC_DisableIRQ(GPAB_IRQn);
	NVIC_DisableIRQ(GPCDEF_IRQn);
	GPIO_DisableInt(GpioToPort(BOARD_KEY_MENU_PIN), GpioToNum(BOARD_KEY_MENU_PIN));
	GPIO_DisableInt(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToNum(BOARD_KEY_DOWN_PIN));
	GPIO_DisableInt(GpioToPort(BOARD_KEY_UP_PIN), GpioToNum(BOARD_KEY_UP_PIN));
	GPIO_DisableInt(GpioToPort(BOARD_KEY_OK_PIN), GpioToNum(BOARD_KEY_OK_PIN));
	GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN));
	GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN));
	GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN));
	GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN));
	GPIO_SetMode(GpioToPort(BOARD_KEY_MENU_PIN), GpioToBit(BOARD_KEY_MENU_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(BOARD_KEY_DOWN_PIN), GpioToBit(BOARD_KEY_DOWN_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(BOARD_KEY_UP_PIN), GpioToBit(BOARD_KEY_UP_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN), GPIO_PMD_OUTPUT);
	BOARD_KEY_MENU = 0;
	BOARD_KEY_UP = 0;
	BOARD_KEY_DOWN = 0;
	BOARD_KEY_OK = 0;

}

void Default_Handler(void)
{
	printf("Enter default handler!\n");
	while(1);
}

static volatile uint8_t keyVal, keyFlag;
#if 1
/* PB14无法触发GPAB_IRQHandler，只能触发EINT0_IRQHandler */
void EINT0_IRQHandler(void)
{
    uint32_t ret;
    ret = GPIO_GET_INT_FLAG(GpioToPort(ACC_DET_PIN), GpioToBit(ACC_DET_PIN));
    if (ret) {
        GPIO_CLR_INT_FLAG(GpioToPort(ACC_DET_PIN), GpioToBit(ACC_DET_PIN));
    }
}

#endif

void GPAB_IRQHandler(void)
{
    uint32_t ret;
    ret = GPIO_GET_INT_FLAG(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN));
    if (ret) {
        GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_OK_PIN), GpioToBit(BOARD_KEY_OK_PIN));
        keyVal = KEY_OK;
        keyFlag = 1;
    }

    ret = GPIO_GET_INT_FLAG(PA, 0x0ffff);
    if (ret != 0) {
    	printf("\n***Unexpected int occured: PA, 0x%x\n", ret);
    	while(1);
    }
    ret = GPIO_GET_INT_FLAG(PB, 0x0ffff);
	if (ret != 0) {
		printf("\n***Unexpected int occured: PB, 0x%x\n", ret);
		while(1);
	}
}


void GPCDEF_IRQHandler(void)
{
    uint32_t ret;
    ret = GPIO_GET_INT_FLAG(GpioToPort(BOARD_KEY_MENU_PIN), BOARD_KEY_MENU_PIN&0x0FFFF);
    if (ret) {
        GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_MENU_PIN), BOARD_KEY_MENU_PIN&0x0FFFF);
        keyVal = KEY_MENU;
        keyFlag = 1;
    }
    ret = GPIO_GET_INT_FLAG(GpioToPort(BOARD_KEY_UP_PIN), BOARD_KEY_UP_PIN&0x0FFFF);
    if (ret) {
        GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_UP_PIN), BOARD_KEY_UP_PIN&0x0FFFF);
        keyVal = KEY_UP;
        keyFlag = 1;
    }
    ret = GPIO_GET_INT_FLAG(GpioToPort(BOARD_KEY_DOWN_PIN), BOARD_KEY_DOWN_PIN&0x0FFFF);
    if (ret) {
        GPIO_CLR_INT_FLAG(GpioToPort(BOARD_KEY_DOWN_PIN), BOARD_KEY_DOWN_PIN&0x0FFFF);
        keyVal = KEY_DOWN;
        keyFlag = 1;
    }

#ifdef HDD_LOCK_DET_PIN
    ret = GPIO_GET_INT_FLAG(GpioToPort(HDD_LOCK_DET_PIN), HDD_LOCK_DET_PIN&0x0FFFF);
    if (ret) {
    	printf("*1*\n");
    	GPIO_CLR_INT_FLAG(GpioToPort(HDD_LOCK_DET_PIN), HDD_LOCK_DET_PIN&0x0FFFF);
    }
#endif

#if defined(USE_EXT_RTC)  && defined(RTC_PULSE_IN_PIN)
    ret = GPIO_GET_INT_FLAG(GpioToPort(RTC_PULSE_IN_PIN), RTC_PULSE_IN_PIN&0x0FFFF);
	if (ret) {
		GPIO_CLR_INT_FLAG(GpioToPort(RTC_PULSE_IN_PIN), RTC_PULSE_IN_PIN&0x0FFFF);
		ExtRtcIrqCallback();
	}
#endif

	if (test_mode == 0xE2) { // 输出车速脉冲信号
		ret = GPIO_GET_INT_FLAG(GpioToPort(SPD_PULSE_IN_PIN), SPD_PULSE_IN_PIN&0x0FFFF);
		if (ret) {
			GPIO_CLR_INT_FLAG(GpioToPort(SPD_PULSE_IN_PIN), SPD_PULSE_IN_PIN&0x0FFFF);
			TST_SIGNAL = SPD_PULSE_IN;
		}
	}
	ret = GPIO_GET_INT_FLAG(PC, 0x0ffff);
	if (ret != 0) {
		printf("\n***Unexpected int occured: PC, 0x%x***\n", ret);
		GPIO_CLR_INT_FLAG(PC, 0x0ffff);
	}
	ret = GPIO_GET_INT_FLAG(PD, 0x0ffff);
	if (ret != 0) {
		printf("\n***Unexpected int occured: PD, 0x%x***\n", ret);
		GPIO_CLR_INT_FLAG(PD, 0x0ffff);
	}
	ret = GPIO_GET_INT_FLAG(PE, 0x0ffff);
	if (ret != 0) {
		printf("\n***Unexpected int occured: PE, 0x%x\n", ret);
		GPIO_CLR_INT_FLAG(PE, 0x0ffff);
	}
	ret = GPIO_GET_INT_FLAG(PF, 0x0ffff);
	if (ret != 0) {
		printf("\n***Unexpected int occured: PF, 0x%x\n", ret);
		GPIO_CLR_INT_FLAG(PF, 0x0ffff);
	}
}
#endif

void BoardIoInit(void)
{
	/* 这里的IN PIN是指设备的输入PIN，测试板对应为输出 */
	GPIO_SetMode(GpioToPort(IO_IN1_PIN), GpioToBit(IO_IN1_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(IO_IN2_PIN), GpioToBit(IO_IN2_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(IGN_OUT_PIN), GpioToBit(IGN_OUT_PIN), GPIO_PMD_OUTPUT);
	GPIO_SetMode(GpioToPort(IGN_IN_PIN), GpioToBit(IGN_IN_PIN), GPIO_PMD_INPUT);
#if 0
	GPIO_SetMode(GpioToPort(PWR_TRIGGER_PIN), GpioToBit(PWR_TRIGGER_PIN), GPIO_PMD_OUTPUT);
	PWR_TRIGGER = 0;
#endif

	IGN_OUT_SIG = IGN_OUT_SIG_LEVEL;

	GPIO_SetMode(GpioToPort(PD_4), GpioToBit(PD_4), GPIO_PMD_OUTPUT);
	PD4 = 1;
	//void BuzzerInit(void);
	//BuzzerInit();
}

#if 0

void IoOutput(uint8_t id, uint8_t val)
{
	uint16_t t;
	t = val & 0x7F;
	t *= 50;
	if (id <= IO_CHANNEL_IOIN2) {
		if (val & 0x80) {
			LedBlink(id, t, t, 0xffff, 0);
		} else {
			LedBlink(id, t, 0, 0, 0);
		}
	}
}

uint32_t IoInput(uint8_t id)
{
	switch(id) {
	case IO_CHANNEL_IOOUT1:
		return !IO_OUT1_SIG;
	case IO_CHANNEL_IOOUT2:
		return !IO_OUT2_SIG;
	default:
		return 0xFFFFFFFF;
	}
}

void AnalogOutput(uint8_t channel, uint8_t val)
{
	(void)channel;
	(void)val;
}

void SetSpeedPulse(uint16_t freq)
{
	if (freq > 500) {
		freq = 500;
	}
	if (freq != 0) {
		/* SPD_PULSE_SIG所在通道为9 */
		LedBlink(9, 500/freq, 500/freq, 0xffff, 0);
	} else {
		LedBlink(9, 0, 0, 0, 0);
	}

}
#endif

/************ ADC电压采样相关接口 ******************/

static uint16_t s_adc_sample_buf[8][9];
static uint8_t s_adc_sample_timer_id = 0xFF;

void ADC_SampleTimerTask(int param)
{
	static uint8_t cnt = 0;
	uint8_t ch, i;
	uint16_t *p;
	for (ch = 0; ch < 8; ch++) {
#if CONFIG_ADC_CONTINUOUS
		s_adc_sample_buf[ch][cnt] = ADC_GET_CONVERSION_DATA(ADC, ch);
#else
		s_adc_sample_buf[i][cnt] = ADC_GetSampleData(ch);
#endif
		s_adc_sample_buf[ch][8] = 0;
	}
	//printf("%d", s_adc_sample_buf[ADC_CH_IOUT][cnt]);
	cnt++;
	if (cnt >= 8) {
		cnt = 0;
	}
	for (ch = 0; ch < 8; ch++) {
		p = s_adc_sample_buf[ch];
		for (i = 0; i < 8; i++) {
			p[8] += p[i];
		}
		p[8] >>= 3;
	}
	//printf(",%d\n", s_adc_sample_buf[ADC_CH_IOUT][8]);
}

void ADC_AvgSampleStart(void)
{
	if (!IsValidTimerId(s_adc_sample_timer_id)) {
		s_adc_sample_timer_id = SoftTimerAlloc();
		if (IsValidTimerId(s_adc_sample_timer_id)) {
			SoftTimerStart(s_adc_sample_timer_id, 1, TMR_AUTO_MODE, ADC_SampleTimerTask, 0);
		}
	}
}

void ADC_SampleInit(void)
{
    /* Disable the GPA0 - GPA6 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, 0x7F);
    
    /* Configure the GPA0 - GPA4 ADC analog input pins */
    SYS->GPA_MFP &= ~(0x7F) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 | SYS_GPA_MFP_PA1_ADC1 | SYS_GPA_MFP_PA2_ADC2 | SYS_GPA_MFP_PA3_ADC3 |\
    		SYS_GPA_MFP_PA4_ADC4 | SYS_GPA_MFP_PA5_ADC5 | SYS_GPA_MFP_PA6_ADC6| SYS_GPA_MFP_PA7_ADC7;
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA1_Msk | SYS_ALT_MFP_PA2_Msk | SYS_ALT_MFP_PA3_Msk |\
    		SYS_ALT_MFP_PA4_Msk | SYS_ALT_MFP_PA5_Msk | SYS_ALT_MFP_PA6_Msk | SYS_ALT_MFP_PA7_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PA1_ADC1 | SYS_ALT_MFP_PA2_ADC2 | SYS_ALT_MFP_PA3_ADC3 |\
    		SYS_ALT_MFP_PA4_ADC4 | SYS_ALT_MFP_PA5_ADC5 | SYS_ALT_MFP_PA6_ADC6 | SYS_ALT_MFP_PA7_ADC7;
    SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA0_Msk | SYS_ALT_MFP1_PA1_Msk | SYS_ALT_MFP1_PA2_Msk | SYS_ALT_MFP1_PA3_Msk |\
    		SYS_ALT_MFP1_PA4_Msk | SYS_ALT_MFP1_PA5_Msk | SYS_ALT_MFP1_PA6_Msk | SYS_ALT_MFP1_PA7_Msk);
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA0_ADC0 | SYS_ALT_MFP1_PA1_ADC1 | SYS_ALT_MFP1_PA2_ADC2 | SYS_ALT_MFP1_PA3_ADC3 |\
    		SYS_ALT_MFP1_PA4_ADC4 | SYS_ALT_MFP1_PA5_ADC5 | SYS_ALT_MFP1_PA6_ADC6 | SYS_ALT_MFP1_PA7_ADC7;
#ifdef CONFIG_ADC_CONTINUOUS
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, 0xFF);

    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Start A/D conversion */
    ADC_START_CONV(ADC);
#else
	/* Clear the A/D interrupt flag for safe */
	ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
#endif

	ADC_AvgSampleStart();
}

uint16_t ADC_GetSampleData(uint8_t ch)
{
    /* Wait conversion done */
    //while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));
    //ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
#if CONFIG_ADC_CONTINUOUS
    return ADC_GET_CONVERSION_DATA(ADC, ch);
#else
	uint16_t ret;
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 1<<ch);
    ADC_POWER_ON(ADC);
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ADC_START_CONV(ADC);

    /* Wait conversion done */
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
    ret = ADC_GET_CONVERSION_DATA(ADC, ch);
    ADC_POWER_DOWN(ADC);
    return ret;
#endif
}

uint16_t ADC_GetAvgSampleData(uint8_t ch)
{
	uint16_t ret;
	__disable_irq();
	ret = s_adc_sample_buf[ch][8];
	__enable_irq();
	return ret;
}

/* 采集指定通道电压值(单位mv) */
uint32_t ADC_GetChanVolt(uint8_t ch)
{
	if (ch > 8) {
		return 0;
	}
#if 0
#if CONFIG_ADC_CONTINUOUS
    uint32_t val = ADC_GET_CONVERSION_DATA(ADC, ch);
#else
    uint32_t val = ADC_GetSampleData(ch);
#endif
#else
	uint32_t val = ADC_GetAvgSampleData(ch);
#endif
    val = val*ADC_VREF/ADC_SCALE;
    switch(ch) {
    case 0:
    	val = val*ADC_RATIO_CH0;
    	break;
    case 1:
    	val = val*ADC_RATIO_CH1;
    	break;
    case 2:
    	val = val*ADC_RATIO_CH2;
    	break;
    case 3:
    	val = val*ADC_RATIO_CH3;
    	break;
    case 4:
    	val = val*ADC_RATIO_CH4;
    	break;
	case 5:
		val = val*ADC_RATIO_CH5;
    	break;
	case 6:
		val = val*ADC_RATIO_CH6;
    	break;
	case 7:
		val = val*ADC_RATIO_CH7;
    	break;
    default:
    	val = 0;
    	break;
    }

    if (val > 65535) val = 65535;

    return val;
}


/************ 看门狗喂狗接口 *****************/

void WDT_Feed(void)
{
	/* Unlock protected registers */
	SYS_UnlockReg();
	WDT_RESET_COUNTER();
	/* Lock protected registers */
	SYS_LockReg();
}

/************ 蜂鸣器相关接口 *****************/

void BuzzerInit(void)
{
	/* PWMA 4个通道，CH0~CH3分别对应PWM0~3， PWMB 4个通道, CH0~CH3分别对应PWM4~7 */
	SYS_UnlockReg();
	CLK_EnableModuleClock(BUZZER_PWM_MODULE);
	CLK_SetModuleClock(BUZZER_PWM_MODULE, BUZZER_PWM_CLK_SRC, 0);
	SYS_ResetModule(PWM47_RST);
	SYS->GPE_MFP &= ~SYS_GPE_MFP_PE5_Msk;
	SYS->GPE_MFP |= SYS_GPE_MFP_PE5_PWM5;
	SYS->ALT_MFP2 &= ~SYS_ALT_MFP2_PE5_Msk;
	SYS->ALT_MFP2 |= SYS_ALT_MFP2_PE5_PWM5;
	SYS_LockReg();
	PWM_ConfigOutputChannel(PWMB, BUZZER_PWM_CHN, 2000, 30);
	//PWM_EnableOutput(PWMB, 1<<BUZZER_PWM_CHN);
	//PWM_Start(PWMB, 1<<BUZZER_PWM_CHN);
}


struct BuzzerCfg {
	uint16_t freq;
	uint16_t timeout;
};

struct _BuzzerInfo {
	uint8_t duty_cycle;
	uint8_t cfg_index;
	uint8_t cfg_num;
	struct BuzzerCfg cfg[BUZZER_MAX_CFGS];
};

static struct _BuzzerInfo buzzer_info;
static uint8_t buzzer_timer_id = 0xff;

static void BuzzerTimeoutCallback(int param)
{
	buzzer_info.cfg_index++;
	if (buzzer_info.cfg_index < buzzer_info.cfg_num) {
		BuzzerOn(buzzer_info.cfg[buzzer_info.cfg_index].freq, buzzer_info.duty_cycle, buzzer_info.cfg[buzzer_info.cfg_index].timeout);
	} else {
		BuzzerForceOff();
	}
}

void BuzzerOn(uint32_t freq, uint32_t duty_cycle, uint32_t time_ms)
{
	if (freq == 0 || time_ms == 0) {
		PWM_DisableOutput(PWMB, 1<<BUZZER_PWM_CHN);
		PWM_Stop(PWMB, 1<<BUZZER_PWM_CHN);
	} else {
		PWM_ConfigOutputChannel(PWMB, BUZZER_PWM_CHN, freq, duty_cycle);
		PWM_EnableOutput(PWMB, 1<<BUZZER_PWM_CHN);
		PWM_Start(PWMB, 1<<BUZZER_PWM_CHN);
	}
	if (time_ms) {
		if (buzzer_timer_id >= SW_TIMER_NUM) {
			buzzer_timer_id = SoftTimerAlloc();
			if (buzzer_timer_id >= SW_TIMER_NUM) {
				printf("buzzer timer alloc failed!\n");
			}
		}
		SoftTimerStart(buzzer_timer_id, time_ms, TMR_ONCE_MODE, BuzzerTimeoutCallback, buzzer_timer_id);
	} else {
		SoftTimerStop(buzzer_timer_id);
		memset(&buzzer_info, 0, sizeof(buzzer_info));
	}
}

void BuzzerForceOff(void)
{
	if (IsValidTimerId(buzzer_timer_id)) {
		SoftTimerStop(buzzer_timer_id);
		SoftTimerFree(&buzzer_timer_id);
	}
	memset(&buzzer_info, 0, sizeof(buzzer_info));
	PWM_DisableOutput(PWMB, 1<<BUZZER_PWM_CHN);
	PWM_Stop(PWMB, 1<<BUZZER_PWM_CHN);
}

void BuzzerCmdCallback(uint8_t *p, uint8_t len)
{
	uint16_t i;

	buzzer_info.duty_cycle = p[1];
	p += 2;
	len -= 2;
	len >>= 2; // len /= 4;
	for(i=0; i<len; i++) {
		buzzer_info.cfg[i].freq = m_ntohs(p);
		p += 2;
		buzzer_info.cfg[i].timeout = m_ntohs(p);
		p += 2;
	}
	buzzer_info.cfg_index = 0;
	buzzer_info.cfg_num = i;
	BuzzerOn(buzzer_info.cfg[0].freq, buzzer_info.duty_cycle, buzzer_info.cfg[0].timeout);
}

/* 单步配置蜂鸣器鸣响
 * index: 第几步配置
 * freq: 鸣响频率
 * duty_cycle: 占空比
 * time_ms: 鸣响持续时间
 * start: 是否开启输出标志（在最后一组配置的时候开启输出）
 */
void BuzzerStepCfg(uint8_t index, uint32_t freq, uint32_t duty_cycle, uint32_t time_ms, uint8_t start)
{
	if (freq > 20000 || index > BUZZER_MAX_CFGS) {
		printf("Invalid param!\n");
		return;
	}
	if (IsValidTimerId(buzzer_timer_id)) {
		BuzzerForceOff();
	}
	if (index == 0) {
		memset(&buzzer_info, 0, sizeof(buzzer_info));
	}
	buzzer_info.duty_cycle = duty_cycle;
	buzzer_info.cfg[index].freq = freq;
	buzzer_info.cfg[index].timeout = time_ms;
	buzzer_info.cfg_num++;
	if (start) {
		BuzzerOn(buzzer_info.cfg[0].freq, buzzer_info.duty_cycle, buzzer_info.cfg[0].timeout);
	}
}

/* 简单的配置蜂鸣器响指定次(响与不响的时间长度相同)
 * freq: 鸣响频率
 * periodOn: 响的时间
 * periodOff: 不响的时间
 * times: 鸣响次数（最多16次）
 */
void BuzzerSimpleCfg(uint32_t freq, uint32_t periodOn, uint32_t periodOff, uint32_t times)
{
	uint32_t i;
	if (times > 16) {
		times = 16;
	}
	times *= 2;
	for (i=0; i<times; ) {
		/* 配置以freq频率响period/2的时间 */
		BuzzerStepCfg(i++, freq, 50, periodOn, 0);
		/* 配置静音period/2的时间 */
		if (i == times-1) {
			BuzzerStepCfg(i++, 0, 50, periodOff, 1);
		} else {
			BuzzerStepCfg(i++, 0, 50, periodOff, 0);
		}
	}
}

void LedBlink(uint8_t ledId, uint16_t onTime, uint16_t offTime, uint16_t cycles, uint8_t lastStat)
{
	if (ledId > LED_NUM-1) return;
	__disable_irq();
	if (cycles == 0) {
		if (onTime)
			*(ledCfg[ledId].pin) = ledCfg[ledId].on_level;
		else
			*(ledCfg[ledId].pin) = !ledCfg[ledId].on_level;
	}
	ledCfg[ledId].last_stat = lastStat;
	ledCfg[ledId].on_time = onTime;
	ledCfg[ledId].off_time = onTime + offTime;
	ledCfg[ledId].cycles = cycles;
	ledCfg[ledId].counter = 0;
	__enable_irq();
}


#endif

/************** GPIO 扩展API *********************/

uint8_t BitPosToNum(uint32_t bit)
{
    uint8_t cnt = 0;
    while(bit) {
        bit >>= 1;
        cnt++;
    }
    return cnt;
}

GPIO_T *GpioToPort(uint32_t gpio)
{
	return (GPIO_T *)((gpio>>24)*0x40+GPIO_BASE);
}

/************ SPI 相关接口 ******************/

void SPI_Init(SPI_T *port, uint32_t mode, uint32_t clk, uint8_t data_width)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    if (port == SPI0) {
        /* Select HCLK as the clock source of SPI0 */
        CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL1_SPI0_S_HCLK, MODULE_NoMsk);

        /* Enable SPI0 peripheral clock */
        CLK_EnableModuleClock(SPI0_MODULE);

        /* Setup SPI0 multi-function pins */
        SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
        SYS->GPC_MFP |= SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;
        SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
        SYS->ALT_MFP = SYS_ALT_MFP_PC1_SPI0_CLK | SYS_ALT_MFP_PC2_SPI0_MISO0 | SYS_ALT_MFP_PC3_SPI0_MOSI0;
    } else if (port == SPI1) {
        /* Select HCLK as the clock source of SPI0 */
        CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL1_SPI1_S_HCLK, MODULE_NoMsk);

        /* Enable SPI0 peripheral clock */
        CLK_EnableModuleClock(SPI1_MODULE);

        /* Setup SPI0 multi-function pins */
        SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC9_Msk | SYS_GPC_MFP_PC10_Msk | SYS_GPC_MFP_PC11_Msk);
        SYS->GPC_MFP |= SYS_GPC_MFP_PC9_SPI1_CLK | SYS_GPC_MFP_PC10_SPI1_MISO0 | SYS_GPC_MFP_PC11_SPI1_MOSI0;
        SYS->ALT_MFP |= SYS_ALT_MFP_PC9_SPI1_CLK | SYS_ALT_MFP_PC10_SPI1_MISO0 | SYS_ALT_MFP_PC11_SPI1_MOSI0;
    } else if (port == SPI2) {
        /* Select HCLK as the clock source of SPI0 */
        CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL1_SPI2_S_HCLK, MODULE_NoMsk);

        /* Enable SPI0 peripheral clock */
        CLK_EnableModuleClock(SPI2_MODULE);

        /* Setup SPI0 multi-function pins */
        SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk);
        SYS->GPD_MFP |= SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
        SYS->ALT_MFP &= ~(SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk);
        SYS->ALT_MFP |= SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;
    } else {
        /* Select HCLK as the clock source of SPI0 */
        CLK_SetModuleClock(SPI3_MODULE, CLK_CLKSEL1_SPI3_S_HCLK, MODULE_NoMsk);

        /* Enable SPI0 peripheral clock */
        CLK_EnableModuleClock(SPI3_MODULE);

        /* Setup SPI0 multi-function pins */
        SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD9_Msk | SYS_GPD_MFP_PD10_Msk | SYS_GPD_MFP_PD11_Msk);
        SYS->GPD_MFP |= SYS_GPD_MFP_PD9_SPI3_CLK | SYS_GPD_MFP_PD10_SPI3_MISO0 | SYS_GPD_MFP_PD11_SPI3_MOSI0;
        SYS->ALT_MFP |= SYS_ALT_MFP_PD9_SPI3_CLK | SYS_ALT_MFP_PD10_SPI3_MISO0 | SYS_ALT_MFP_PD11_SPI3_MOSI0;
    }
    /* Lock protected registers */
    SYS_LockReg();

    SPI_Open(port, SPI_MASTER, mode, data_width, clk);
}

void SPI_DeInit(SPI_T *port)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    if (port == SPI0) {
        /* Enable SPI0 peripheral clock */
        CLK_DisableModuleClock(SPI0_MODULE);

        /* reset to gpio pins */
        //SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC1_Msk | SYS_GPC_MFP_PC2_Msk | SYS_GPC_MFP_PC3_Msk);
        //SYS->ALT_MFP &= ~(SYS_ALT_MFP_PC1_Msk | SYS_ALT_MFP_PC2_Msk | SYS_ALT_MFP_PC3_Msk);
    } else if (port == SPI1) {
        /* Enable SPI0 peripheral clock */
        CLK_DisableModuleClock(SPI1_MODULE);

        /* reset to gpio pins */
        SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC9_Msk | SYS_GPC_MFP_PC10_Msk | SYS_GPC_MFP_PC11_Msk);
        GPIO_SetMode(GpioToPort(PC_9), GpioToBit(PC_9), GPIO_PMD_OUTPUT);
		GPIO_SetMode(GpioToPort(PC_10), GpioToBit(PC_10), GPIO_PMD_OUTPUT);
		GPIO_SetMode(GpioToPort(PC_11), GpioToBit(PC_11), GPIO_PMD_OUTPUT);
		PC9 = 0;
		PC10 = 0;
		PC11 = 0;
    } else if (port == SPI2) {
        /* Enable SPI0 peripheral clock */
        CLK_DisableModuleClock(SPI2_MODULE);

        /* reset to gpio pins */
        SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk);
        SYS->ALT_MFP &= ~(SYS_GPD_MFP_PD1_Msk | SYS_GPD_MFP_PD2_Msk | SYS_GPD_MFP_PD3_Msk);
        GPIO_SetMode(GpioToPort(PD_1), GpioToBit(PD_1), GPIO_PMD_OUTPUT);
		GPIO_SetMode(GpioToPort(PD_2), GpioToBit(PD_2), GPIO_PMD_OUTPUT);
		GPIO_SetMode(GpioToPort(PD_3), GpioToBit(PD_3), GPIO_PMD_OUTPUT);
		PD1 = 0;
		PD2 = 0;
		PD3 = 0;
    } else {
        /* Enable SPI0 peripheral clock */
        CLK_DisableModuleClock(SPI3_MODULE);

        /* reset to gpio pins */
        //SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD9_Msk | SYS_GPD_MFP_PD10_Msk | SYS_GPD_MFP_PD11_Msk);
    }
    /* Lock protected registers */
    SYS_LockReg();
}

void SPI_WriteBytes(SPI_T *spi, uint8_t *buffer, uint32_t len)
{
    while(len) {
        /* Write to TX register */
        SPI_WRITE_TX0(spi, *buffer++);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(spi);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(spi));
        len--;
    }
}

void SPI_ReadBytes(SPI_T *spi, uint8_t *buffer, uint32_t len)
{
    while(len) {
        /* Write to TX register */
        SPI_WRITE_TX0(spi, 0xFF);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(spi);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(spi));
        *buffer++ = SPI_READ_RX0(spi);
        len--;
    }
}

uint8_t SPI_WriteReadByte(SPI_T *spi, uint8_t byte)
{
    /* Write to TX register */
    SPI_WRITE_TX0(spi, byte);
    /* Trigger SPI data transfer */
    SPI_TRIGGER(spi);
    /* Check SPI0 busy status */
    while(SPI_IS_BUSY(spi));
    return SPI_READ_RX0(spi);
}

void SPI_WriteWords(SPI_T *spi, uint16_t *buffer, uint32_t len)
{
    while(len) {
        /* Write to TX register */
        SPI_WRITE_TX0(spi, *buffer++);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(spi);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(spi));
        len--;
    }
}

void SPI_ReadWords(SPI_T *spi, uint16_t *buffer, uint32_t len)
{
    while(len) {
        /* Write to TX register */
        SPI_WRITE_TX0(spi, 0xFF);
        /* Trigger SPI data transfer */
        SPI_TRIGGER(spi);
        /* Check SPI0 busy status */
        while(SPI_IS_BUSY(spi));
        *buffer++ = SPI_READ_RX0(spi);
        len--;
    }
}

uint16_t SPI_WriteReadWord(SPI_T *spi, uint16_t word)
{
    /* Write to TX register */
    SPI_WRITE_TX0(spi, word);
    /* Trigger SPI data transfer */
    SPI_TRIGGER(spi);
    /* Check SPI0 busy status */
    while(SPI_IS_BUSY(spi));
    return SPI_READ_RX0(spi);
}

#if 0
uint8_t crc8(uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t ret;

    CRC_Open(CRC_8, 0, 0, CRC_CPU_WDATA_8);
    for(i = 0; i < size; i++) {
        CRC_WRITE_DATA(*buf & 0xFF);
        buf++;
    }
    ret = CRC_GetChecksum();
    /* Disable CRC function */
    CRC->CTL &= ~CRC_CTL_CRCCEN_Msk;
    return ret;
}


#ifdef USE_SOFTWARE_CRC32

const uint32_t crc_table[256] = {
0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};

/*计算buffer的crc校验码*/
uint32_t crc32(uint32_t crc, uint8_t *buffer, uint32_t size)
{
	unsigned int i;
	for (i = 0; i < size; i++) {
		crc = crc_table[(crc ^ buffer[i]) & 0xff] ^ (crc >> 8);
	}
	return crc;
}

#else

uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t size)
{
    uint32_t i;
    uint8_t ret;
    CRC_Open(CRC_32, CRC_CHECKSUM_COM, 0xffffffff, CRC_CPU_WDATA_32);
    for(i = 0; i < size; i+=4) {
        CRC_WRITE_DATA(*buf);
        buf++;
    }
    ret = CRC_GetChecksum();
    /* Disable CRC function */
    CRC->CTL &= ~CRC_CTL_CRCCEN_Msk;
    return ret;
}
#endif
#endif

/* 获取val参数中bit位为1的个数 */
uint8_t CountBits(uint32_t val)
{
	uint8_t i, j = 0;
	for (i=0; i<32; i++) {
		if (val & (1<<i)) {
			j++;
		}
	}
	return j;
}



#ifndef __MISC_H
#define __MISC_H
#include <stdint.h>
#include <stddef.h>
#include "NUC230_240.h"
#include "bsp.h"
#include "list.h" 

#define UART_MESG_TIMEOUT       500

#define UART_PROTO_BUF_MAX_SIZE		256

#define BUZZER_MAX_CFGS	32

#define COMMUNICATION_CAN_ID	0x1FF5A000

/* 菜单配置数据存放地址 */
#define SYS_CFG_DATA_ADDR  0x1F000
/* 菜单配置数据标记 */
#define SYS_CFG_MAGIC  0x4A4D4832
/* 密码长度 */
#define SYS_CFG_PASSWD_LEN     8

#define SLEEP_TIME		8000

enum {
	FIRMWARE_TYPE_BOOTLOADER = 0,
	FIRMWARE_TYPE_APP,
	FIRMWARE_TYPE_DATA,
	FIRMWARE_TYPE_FONT,
	FIRMWARE_TYPE_UNKNOW,
};

typedef enum {
	TMR_ONCE_MODE = 0,		/* 一次工作模式 */
	TMR_AUTO_MODE = 1		/* 自动定时工作模式 */
}TMR_MODE_E;

typedef enum {
    COM0 = 0,
    COM1,
    COM2,
} COM_ID_t;

struct ledBlinkCfg {
	volatile uint32_t *pin; /* GPIO引脚地址 */
	uint16_t on_level; /* 点亮的电平 */
	uint16_t last_stat; /* cycle循环结束后LED状态，1点亮，0熄灭，仅对于cycles > 0时有效 */
	uint16_t on_time;  /* 点亮的时间 */
	uint16_t off_time; /* 熄灭的时间(加上点亮的时间) */
	uint16_t cycles; /* 点亮次数 */
	uint16_t counter; /* 计数器 */
};

typedef void (*TimerCallback_t)(int param);

typedef struct {
	uint8_t Inuse;              /* 正在使用的标志,用于动态申请 */
	volatile uint8_t Mode;		/* 计数器模式，1次性 */
	volatile uint8_t Flag;		/* 定时到达标志  */
	volatile uint32_t Count;	/* 计数器 */
	volatile uint32_t PreLoad;	/* 计数器预装值 */
    TimerCallback_t callback;
    int callback_param;
}SOFT_TMR;

typedef struct _ring_buffer{
    uint8_t *buf;
    __IO uint16_t rptr;
    __IO uint16_t wptr;
    __IO uint8_t isfull;
    __IO uint8_t rxchksum;
#ifdef COM_USE_DMA_SEND
    __IO uint16_t dma_pending;
#endif
    union {
        __IO uint16_t send_cnt;
        __IO uint16_t recv_cnt;
    }rw_cnt;
    uint32_t start_time;
    
} RingBuffer_t;

typedef struct {
	UART_T *uart;
	list_t *rxqueue;
	list_t *txqueue;
	uint8_t *rxbuf;
	uint8_t tmprxbuf[6];
	uint16_t rx_cnt;
	uint16_t cmd_len;
	uint8_t chksum;
	uint8_t dma_pending;
	uint8_t cmd_pending;
	uint32_t time_start;
}ComInfo_t;

extern ComInfo_t comInfo;

typedef struct GBTimeStamp {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}GBTimeStamp_t;

typedef struct _MsgHandler{
    uint8_t cmd;
    int8_t stat;
    uint8_t retry;
    RingBuffer_t *cmd_buf;
    RingBuffer_t *resp_buf;
    void (*func)(void);
}MsgObject_t;

typedef struct _ComPair{
    COM_ID_t in_com_id;
    COM_ID_t out_com_id;
}ComPair_t;

typedef struct CanMsgQueue{
    STR_CANMSG_T msg;
    uint8_t inuse;
    struct CanMsgQueue *next;
}CanMsgQueue_t;

typedef struct _DriverCardInfo{
	uint8_t factory_data[32];
    char lic_id[18];
    uint8_t expire_year; // bcd code
    uint8_t expire_month; // bcd code
    uint8_t expire_day; // bcd code
    char cert_id[18];
    uint8_t reserved[56];
    uint8_t chksum;
}DirverCardInfo_t;

/* MCU升级包头信息固定大小为64字节 */
struct FwInfo {
	uint32_t magic;
	char version[16];
	uint32_t load_addr;
	uint16_t fw_type;
	uint16_t header_size;
	uint32_t data_size;
	uint32_t data_crc;
	uint32_t utc_stamp;
	uint32_t reserved[5];
	uint32_t header_crc;
};

/* 系统配置结构体 */
struct sys_cfg{
    uint32_t magic; //菜单配置数据标志
    uint32_t check_sum; //菜单数据校验和，不包括magic字段和该字段本身
    uint8_t backlight_bright; //背光亮度（开或关），该值与菜单项编号值对应
    uint8_t backlight_timeout; //背光超时时间，分档制，每档与菜单项编号值对应
    uint8_t menu_timeout; //菜单超时时间，分档制，每档与菜单项编号值对应，超时后返回主界面
    uint8_t volume; //报警/提醒音量，分档制，每档与菜单项编号值对应
    uint32_t alarm_cfg; //报警开关配置，按位配置，见上面报警开关位定义
    uint8_t passwd[SYS_CFG_PASSWD_LEN];
};

extern struct sys_cfg g_sSysCfg;

void mdelay(uint32_t ms);

void udelay(uint32_t us);

static __inline void mdelay2(uint32_t ms)
{
	while(ms--) {
		udelay(999);
	}
}

void DisableInterrupt(void);

void EnableInterrupt(void);

uint8_t BitPosToNum(uint32_t bit);
GPIO_T *GpioToPort(uint32_t gpio);

void TimersInit(void);
void SoftTimerStop(uint8_t id);
int SoftTimerCheckTimeout(uint8_t id);
void SoftTimerReload(uint8_t id);
void SoftTimerStart(uint8_t id, uint32_t period, TMR_MODE_E mode, TimerCallback_t func, int param);
void SoftTimerFree(uint8_t *id);
uint8_t SoftTimerAlloc(void);
uint32_t GetRunTime(void);
uint32_t CompareRunTime(uint32_t _LastTime);

void TimestampToRtcTime(uint32_t time, S_RTC_TIME_DATA_T *rtctime);

int FlashWritePage(uint32_t addr, void *buffer);

int FlashRead(uint32_t addr, uint32_t size, void *buffer);

void DataFlashInit(uint32_t u32Base);

void SysCfgInit(void);

void SysSaveConfig(void *cfg);

uint8_t crc8(uint8_t *buf, uint32_t size);

uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t size);

uint8_t GB19056_CalcCheckSum(uint8_t *buf, uint16_t len);

void RTC_Init(void);
void RTC_DeInit(void);

#ifdef USE_EXT_RTC
int ExtRtcInit(void);
void ExtRtcDeInit(void);
int ExtRtcSetDateAndTime(S_RTC_TIME_DATA_T *sPt);
int ExtRtcGetDateAndTime(S_RTC_TIME_DATA_T *sPt);
#endif

void BoardKeyInit(void);
void BoardKeyDeInit(void);

int GetKeyVal(void);

int GetLongPressedKey(void);

void BoardComInit(void);
void BoardComSuspend(void);
void BoardIoInit(void);

void UartProcess(void);

uint16_t FormatCmdRespBuffer(uint16_t hdr_type, uint8_t cmd, uint8_t *buf, uint8_t *data, uint16_t data_len);

static __inline uint32_t m_ntohl(uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

static __inline uint16_t m_ntohs(uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static __inline void m_htonl(uint32_t dat, uint8_t *buf)
{
    buf[0] = dat >> 24;
    buf[1] = (dat >> 16) & 0x00FF;
    buf[2] = (dat >> 8) & 0x00FF;
    buf[3] = dat & 0x00FF;
}

static __inline void m_htons(uint16_t dat, uint8_t *buf)
{
    buf[0] = dat >> 8;
    buf[1] = dat & 0x00FF;
}

#define CAN_RECV_QUEUE_LEN  16
#define CAN_SEND_QUEUE_LEN  16

extern CanMsgQueue_t *pCanRecvQHead[2];
extern CanMsgQueue_t *pCanSendQHead[2];
void CAN_Init(uint8_t port, uint32_t baud);
void CAN_DeInit(void);
void CAN_NormalModeTx(int port);;
CanMsgQueue_t *CanAllocMsg(int port, CanMsgQueue_t **qHead);
void CanMsgQueueSortedInsert(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg);
void CanMsgRemove(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg);
uint8_t CAN_SetRecvId(uint8_t port, uint32_t can_id);
void CAN_TriggerQueueSend(uint8_t port);
void CAN_ClearAllRecvId(uint8_t port);
void CanForwardUartMsg(uint8_t port, uint8_t *msg, uint16_t len);

void SPI_Init(SPI_T *port, uint32_t mode, uint32_t clk, uint8_t data_width);
void SPI_DeInit(SPI_T *port);
void SPI_WriteBytes(SPI_T *spi, uint8_t *buffer, uint32_t len);
void SPI_ReadBytes(SPI_T *spi, uint8_t *buffer, uint32_t len);
uint8_t SPI_WriteReadByte(SPI_T *spi, uint8_t byte);
void SPI_WriteWords(SPI_T *spi, uint16_t *buffer, uint32_t len);
void SPI_ReadWords(SPI_T *spi, uint16_t *buffer, uint32_t len);
uint16_t SPI_WriteReadWord(SPI_T *spi, uint16_t word);


void ADC_SampleInit(void);
uint16_t ADC_GetSampleData(uint8_t ch);
uint32_t ADC_GetChanVolt(uint8_t ch);

int8_t CheckRequestResponse(void);
int SendCmdRequest(uint8_t cmd, uint8_t *data, uint16_t len, uint8_t retry, uint8_t **resp);
uint32_t BcdBytesToBinary(uint8_t *bcd, uint8_t len);
void BcdTimeToGBTimeStamp(uint8_t *bcd, GBTimeStamp_t *tim);
void GBTimeStampToBcdTime(GBTimeStamp_t *tim, uint8_t *bcd);
void GetRTCTime(GBTimeStamp_t *tim);
extern MsgObject_t msg_obj;

void WDT_Feed(void);

void BuzzerOn(uint32_t freq, uint32_t duty_cycle, uint32_t time_ms);
void BuzzerStepCfg(uint8_t index, uint32_t freq, uint32_t duty_cycle, uint32_t time_ms, uint8_t start);
void BuzzerSimpleCfg(uint32_t freq, uint32_t periodOn, uint32_t periodOff, uint32_t times);
void BuzzerForceOff(void);
void hi_hexdump(const void *src, size_t len, size_t width);

void StartPowerMonitor(void);
void PowerDownFunction(void);
void LedBlink(uint8_t ledId, uint16_t onTime, uint16_t offTime, uint16_t cycles, uint8_t lastStat);

void SystemResume(void);
void SystemSuspend(void);
void FlushUartFifo(void);
void WaitUartFifoEmpty(void);

uint8_t CountBits(uint32_t val);

void UartCmdDispatch(void);
void UartRespDispatch(void);
void FlushTxRxQueue(void);
void IoOutput(uint8_t id, uint8_t val);
uint32_t IoInput(uint8_t id);
void SetSpeedPulse(uint16_t freq);
void AnalogOutput(uint8_t channel, uint8_t val);

void SendStartTestNotify(void);
void SendEndTestNotify(uint8_t val);
void StartCanSendTask(void);
void StartSendVoltageInfoTask(void);
void StopSendVoltageInfoTask(void);

void mcb_init(uint32_t mem_start, uint32_t mem_end);
void     *malloc(unsigned int size);
void     free(void *mem);


void SC2SendRadarData(void);
void SendWakeupNotify(void);

#endif


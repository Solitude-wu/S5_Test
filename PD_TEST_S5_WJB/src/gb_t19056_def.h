#ifndef __GB_T19056_DEF_H
#define __GB_T19056_DEF_H

#include "stdint.h"

#define GB_CMD_FRAME_HEAD_1 0xAA
#define GB_CMD_FRAME_HEAD_2 0x75
#define GB_CMD_FRAME_HEAD  0x75AA //命令头

#define GB_RESP_FRAME_HEAD_1 0x55
#define GB_RESP_FRAME_HEAD_2 0x7A
#define GB_RESP_FRAME_HEAD  0x7A55 //响应头

#define GB_RESP_GET_DATA_FRAME_ERR		0xFA //获取命令错误
#define GB_RESP_SET_PARAM_FRAME_ERR     0xFB //设置命令错误

#define GB_CMD_GET_STD_VERSION			0x00 //获取执行标准版本
#define GB_CMD_GET_DRIVER_ID_INFO		0x01 //获取驾驶人信息
#define GB_CMD_GET_DEV_REAL_TIME		0x02 //获取记录仪实时时间
#define GB_CMD_GET_TOTOAL_MILEAGE		0x03 //获取累计行驶里程
#define GB_CMD_GET_PULSE_COEFFICIENT	0x04 //获取脉冲系数
#define GB_CMD_GET_CAR_INFO			    0x05 //获取车辆信息
#define GB_CMD_GET_DEV_STATUS			0x06 //获取记录仪状态信号配置信息
#define GB_CMD_GET_DEV_SERIALNUM		0x07 //获取记录仪唯一编号
#define GB_CMD_GET_SPEED_RECORD		    0x08 //获取指定的行驶速度记录
#define GB_CMD_GET_LOCATION_RECORD		0x09 //获取指定的位置信息记录
#define GB_CMD_GET_ACCIDENT_SUSPECT_RECORD 0x10 //获取指定的事故疑点记录
#define GB_CMD_GET_TIMEOUT_RECORD		0x11 //获取指定的超时驾驶记录
#define GB_CMD_GET_DRIVER_LOGIN_INFO	0x12 //获取指定的驾驶人身份记录
#define GB_CMD_GET_EXTERNAL_POWER_REC	0x13 //获取指定的记录仪外部供电记录
#define GB_CMD_GET_PARAM_CHANGE_RECORD	0x14 //获取指定的记录仪参数修改记录
#define GB_CMD_GET_SPEED_STATUS_INFO	0x15 //获取指定的速度状态日志

/* 扩展命令列表 */
#define GB_EXT_CMD_GET_RECORD_CNT       0x60 //获取指定的记录条数
#define GB_EXT_CMD_GET_NTH_RECORD       0x61 //获取指定的第N条记录
#define GB_EXT_CMD_GET_SW_VERSION       0x62 //获取软件版本号
#define GB_EXT_CMD_GET_NET_CFG          0x63 //获取网络配置
#define GB_EXT_CMD_GET_RT_SPEED         0x64 //获取实时速度信息
#define GB_EXT_CMD_GET_PERI_STAT        0x65 //获取外设状态信息
#define GB_EXT_CMD_GET_GPS_STAT         0x66 //获取GPS状态信息
#define GB_EXT_CMD_GET_WIFI_STAT        0x67 //获取WIFI状态信息
#define GB_EXT_CMD_GET_4G_STAT          0x68 //获取4G状态信息
#define GB_EXT_CMD_GET_PROMPT_TONE_CFG  0x69 //获取提示音配置
#define GB_EXT_CMD_GET_LAST_PARK_TIME  	0x6A //获取最近一次停车时间信息

#define GB_EXT_CMD_SET_PROMPT_TONE_CFG  0xA0 //设置提示音配置

/* 内部扩展命令 */
#define GB_EXT_CMD_INTERNAL             0xF0

#define INTERNAL_CMD_GET_PORTS		0x00 //获取端口个数，CPU->MCU
#define INTERNAL_CMD_SET_PORT		0x01 //设置端口参数，CPU->MCU
#define INTERNAL_CMD_DATA_XFER		0x02 //数据传输，CPU<->MCU
#define INTERNAL_CMD_RESET_MCU		0x03 //复位MCU，CPU->MCU
#define INTERNAL_CMD_GET_ADC        0x04 //获取ADC值，CPU->MCU
#define INTERNAL_CMD_SET_LOG        0x05 //设置MCU串口打印开关
#define INTERNAL_CMD_SEND_ALARM	    0x06 //报警消息，CPU->MCU
#define INTERNAL_CMD_HW_CHG_NOTIFY	0x07 //硬件变更通知， MCU->CPU
#define INTERNAL_CMD_GET_MCU_VER	0x08 //获取MCU软件版本，CPU->MCU
#define INTERNAL_CMD_MCU_UPGRADE	0x09 //MCU升级命令，CPU<->MCU
#define INTERNAL_CMD_BUZZER_CTRL	0x10 //蜂鸣器控制命令，CPU->MCU
#define INTERNAL_CMD_TIME_SYNC		0x11 //时间同步命令，当MCU时间与CPU时间差超过1s则通知CPU进行联网同步，MCU->CPU
#define INTERNAL_CMD_FORMAT_DISK	0x12 //磁盘格式化命令
#define INTERNAL_CMD_CAR_SPEED		0x13 //单位时间(1s)内MCU采集的车速脉冲个数上报命令
#define INTERNAL_CMD_GET_PERI_STAT	0x14 //获取MCU外设状态命令（RTC、SPI Flash）用于产测

#define REC_TYPE_OVERSPEED		0x00   //超速记录
#define REC_TYPE_TIMEOUT		0x01   //超时记录
#define REC_TYPE_GENERAL_MSG	0x10   //普通消息
#define REC_TYPE_EVENT_MSG		0x11   //事件消息
#define REC_TYPE_QUESTION_MSG	0x12   //提问消息
#define REC_TYPE_ORDER_MSG		0x13   //点播消息
#define REC_TYPE_SCHEDULE_MSG	0x14   //调度消息
#define REC_TYPE_MISSED_CALL	0x20   //未接来电
#define REC_TYPE_ANSWERED_CALL	0x21   //已接来电
#define REC_TYPE_DIALED_CALL	0x22   //已拨电话
#define REC_TYPE_PHONE_BOOK		0x30   //电话本记录

/* 报警消息 */
#define ALARM_MSG_DRIVER_LOGIN      0 //驾驶人未登录提醒
#define ALARM_MSG_OVERSPEED         1 //超速提醒
#define ALARM_MSG_TIMEOUT_DRIVING	2 //超时驾驶提醒
#define ALARM_MSG_SPEED_ABNORMAL    3 //速度异常提醒
#define ALARM_MSG_IMMEDIATE_NOTIFY	4 //即时消息
#define ALARM_MSG_UDISK_DATA_XFER	5 //U盘数据导入提醒
#define ALARM_MSG_GENERIC			6 //通用提示消息
#define ALARM_MSG_UPGRADING			7 //升级中，勿关闭核心板供电

#define ALARM_MSG_ICCARD_INSERT     0x10 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_ICCARD_REMOVE     0x11 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_MCU_UPGRADE		0x12 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_BAT_LOW			0x13 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_PWR_LOST			0x14 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_SYS_SLEEP			0x15 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_CFG_RESTORE		0x16 //MCU内部使用，不可用于消息交互
#define ALARM_MSG_FORMAT_DISK		0x17 //MCU内部使用，不可用于消息交互

/* 硬件变更通知 */
#define HW_CHANGE_HDD		0
#define HW_CHANGE_ICCARD	1
#define HW_CHANGE_BAT_LOW	2
#define HW_CHANGE_PWR_LOST	3
#define HW_CHANGE_HOST_SD	4

#define HW_CHANGE_DETACH	0
#define HW_CHANGE_ATTACH	1


/* 国标设置命令 */
#define GB_CMD_SET_CAR_INFO			    0x82 //设置车辆信息
#define GB_CMD_SET_EQUIPT_TIME			0x83 //设置安装日期
#define GB_CMD_SET_STATUS_VALUE		    0x84 //设置状态值
#define GB_CMD_SET_DEV_TIME			    0xC2 //设置设备时间
#define GB_CMD_SET_PULSE_COEFFICIENT	0xC3 //设置脉冲系数
#define GB_CMD_SET_INTIAL_MILEAGE		0xC4 //设置初始里程

/* 国标检定命令 */
#define GB_CMD_ENTER_TEST_MODE				0xE0 //进入检定模式
#define GB_CMD_ENTER_MILEAGE_ERROR_TEST 	0xE1 //进入里程误差测量模式
#define GB_CMD_ENTER_PULSE_COEF_ERROR_TEST	0xE2 //进入脉冲系数误差测量模式
#define GB_CMD_ENTER_REAL_TIME_ERROR_TEST	0xE3 //进入实时时间误差测量模式
#define GB_CMD_ENTER_NORMAL_MODE			0xE4 //退出检定模式


#define GB_CMD_INDEX        2
#define GB_CMD_HEADER_SIZE  6
#define GB_CMD_MAX_DATA_LEN    1000
#define GB_CMD_MAX_FRAME_LEN   (1000+7)

#define CUSTOM_CMD_INVALID  0xFF
#define CUSTOM_CMD_PRIVATE  0xFE

#define CMD_RESP_TIMEOUT -1
#define CMD_REQ_INIT 0
#define CMD_SENDING 1
#define CMD_SEND_OK 2
#define CMD_RESP_OK 3
#define CMD_CHKSUM_ERR  -2
#define CMD_FORMAT_ERR  -3
#define CMD_FORCE_STOP  -4

#define UPGRADE_ERR_FLASH_NOT_ENOUGH	1
#define UPGRADE_ERR_FLASH_WRITE_ERR		2
#define UPGRADE_ERR_CHECKSUM_ERR		3
#define UPGRADE_ERR_INVALID_PROG		4
#define UPGRADE_ERR_RECV_TIMEOUT		0x10
#define UPGRADE_ERR_PACKET_SIZE_ERR		0x11
#define UPGRADE_ERR_PACKET_ID_ERR		0x12

typedef struct {
	uint8_t header1;
	uint8_t header2;
	uint8_t cmd;
	uint8_t len_h;
	uint8_t len_l;
	uint8_t res;
	uint8_t *payload;
	uint8_t checksum;
}GB19056_PROTO_t;

#endif



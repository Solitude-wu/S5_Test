#ifndef __GB_T19056_DEF_H
#define __GB_T19056_DEF_H

#include "stdint.h"

#define GB_CMD_FRAME_HEAD_1 0xAA
#define GB_CMD_FRAME_HEAD_2 0x75
#define GB_CMD_FRAME_HEAD  0x75AA //����ͷ

#define GB_RESP_FRAME_HEAD_1 0x55
#define GB_RESP_FRAME_HEAD_2 0x7A
#define GB_RESP_FRAME_HEAD  0x7A55 //��Ӧͷ

#define GB_RESP_GET_DATA_FRAME_ERR		0xFA //��ȡ�������
#define GB_RESP_SET_PARAM_FRAME_ERR     0xFB //�����������

#define GB_CMD_GET_STD_VERSION			0x00 //��ȡִ�б�׼�汾
#define GB_CMD_GET_DRIVER_ID_INFO		0x01 //��ȡ��ʻ����Ϣ
#define GB_CMD_GET_DEV_REAL_TIME		0x02 //��ȡ��¼��ʵʱʱ��
#define GB_CMD_GET_TOTOAL_MILEAGE		0x03 //��ȡ�ۼ���ʻ���
#define GB_CMD_GET_PULSE_COEFFICIENT	0x04 //��ȡ����ϵ��
#define GB_CMD_GET_CAR_INFO			    0x05 //��ȡ������Ϣ
#define GB_CMD_GET_DEV_STATUS			0x06 //��ȡ��¼��״̬�ź�������Ϣ
#define GB_CMD_GET_DEV_SERIALNUM		0x07 //��ȡ��¼��Ψһ���
#define GB_CMD_GET_SPEED_RECORD		    0x08 //��ȡָ������ʻ�ٶȼ�¼
#define GB_CMD_GET_LOCATION_RECORD		0x09 //��ȡָ����λ����Ϣ��¼
#define GB_CMD_GET_ACCIDENT_SUSPECT_RECORD 0x10 //��ȡָ�����¹��ɵ��¼
#define GB_CMD_GET_TIMEOUT_RECORD		0x11 //��ȡָ���ĳ�ʱ��ʻ��¼
#define GB_CMD_GET_DRIVER_LOGIN_INFO	0x12 //��ȡָ���ļ�ʻ����ݼ�¼
#define GB_CMD_GET_EXTERNAL_POWER_REC	0x13 //��ȡָ���ļ�¼���ⲿ�����¼
#define GB_CMD_GET_PARAM_CHANGE_RECORD	0x14 //��ȡָ���ļ�¼�ǲ����޸ļ�¼
#define GB_CMD_GET_SPEED_STATUS_INFO	0x15 //��ȡָ�����ٶ�״̬��־

/* ��չ�����б� */
#define GB_EXT_CMD_GET_RECORD_CNT       0x60 //��ȡָ���ļ�¼����
#define GB_EXT_CMD_GET_NTH_RECORD       0x61 //��ȡָ���ĵ�N����¼
#define GB_EXT_CMD_GET_SW_VERSION       0x62 //��ȡ����汾��
#define GB_EXT_CMD_GET_NET_CFG          0x63 //��ȡ��������
#define GB_EXT_CMD_GET_RT_SPEED         0x64 //��ȡʵʱ�ٶ���Ϣ
#define GB_EXT_CMD_GET_PERI_STAT        0x65 //��ȡ����״̬��Ϣ
#define GB_EXT_CMD_GET_GPS_STAT         0x66 //��ȡGPS״̬��Ϣ
#define GB_EXT_CMD_GET_WIFI_STAT        0x67 //��ȡWIFI״̬��Ϣ
#define GB_EXT_CMD_GET_4G_STAT          0x68 //��ȡ4G״̬��Ϣ
#define GB_EXT_CMD_GET_PROMPT_TONE_CFG  0x69 //��ȡ��ʾ������
#define GB_EXT_CMD_GET_LAST_PARK_TIME  	0x6A //��ȡ���һ��ͣ��ʱ����Ϣ

#define GB_EXT_CMD_SET_PROMPT_TONE_CFG  0xA0 //������ʾ������

/* �ڲ���չ���� */
#define GB_EXT_CMD_INTERNAL             0xF0

#define INTERNAL_CMD_GET_PORTS		0x00 //��ȡ�˿ڸ�����CPU->MCU
#define INTERNAL_CMD_SET_PORT		0x01 //���ö˿ڲ�����CPU->MCU
#define INTERNAL_CMD_DATA_XFER		0x02 //���ݴ��䣬CPU<->MCU
#define INTERNAL_CMD_RESET_MCU		0x03 //��λMCU��CPU->MCU
#define INTERNAL_CMD_GET_ADC        0x04 //��ȡADCֵ��CPU->MCU
#define INTERNAL_CMD_SET_LOG        0x05 //����MCU���ڴ�ӡ����
#define INTERNAL_CMD_SEND_ALARM	    0x06 //������Ϣ��CPU->MCU
#define INTERNAL_CMD_HW_CHG_NOTIFY	0x07 //Ӳ�����֪ͨ�� MCU->CPU
#define INTERNAL_CMD_GET_MCU_VER	0x08 //��ȡMCU����汾��CPU->MCU
#define INTERNAL_CMD_MCU_UPGRADE	0x09 //MCU�������CPU<->MCU
#define INTERNAL_CMD_BUZZER_CTRL	0x10 //�������������CPU->MCU
#define INTERNAL_CMD_TIME_SYNC		0x11 //ʱ��ͬ�������MCUʱ����CPUʱ����1s��֪ͨCPU��������ͬ����MCU->CPU
#define INTERNAL_CMD_FORMAT_DISK	0x12 //���̸�ʽ������
#define INTERNAL_CMD_CAR_SPEED		0x13 //��λʱ��(1s)��MCU�ɼ��ĳ�����������ϱ�����
#define INTERNAL_CMD_GET_PERI_STAT	0x14 //��ȡMCU����״̬���RTC��SPI Flash�����ڲ���

#define REC_TYPE_OVERSPEED		0x00   //���ټ�¼
#define REC_TYPE_TIMEOUT		0x01   //��ʱ��¼
#define REC_TYPE_GENERAL_MSG	0x10   //��ͨ��Ϣ
#define REC_TYPE_EVENT_MSG		0x11   //�¼���Ϣ
#define REC_TYPE_QUESTION_MSG	0x12   //������Ϣ
#define REC_TYPE_ORDER_MSG		0x13   //�㲥��Ϣ
#define REC_TYPE_SCHEDULE_MSG	0x14   //������Ϣ
#define REC_TYPE_MISSED_CALL	0x20   //δ������
#define REC_TYPE_ANSWERED_CALL	0x21   //�ѽ�����
#define REC_TYPE_DIALED_CALL	0x22   //�Ѳ��绰
#define REC_TYPE_PHONE_BOOK		0x30   //�绰����¼

/* ������Ϣ */
#define ALARM_MSG_DRIVER_LOGIN      0 //��ʻ��δ��¼����
#define ALARM_MSG_OVERSPEED         1 //��������
#define ALARM_MSG_TIMEOUT_DRIVING	2 //��ʱ��ʻ����
#define ALARM_MSG_SPEED_ABNORMAL    3 //�ٶ��쳣����
#define ALARM_MSG_IMMEDIATE_NOTIFY	4 //��ʱ��Ϣ
#define ALARM_MSG_UDISK_DATA_XFER	5 //U�����ݵ�������
#define ALARM_MSG_GENERIC			6 //ͨ����ʾ��Ϣ
#define ALARM_MSG_UPGRADING			7 //�����У���رպ��İ幩��

#define ALARM_MSG_ICCARD_INSERT     0x10 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_ICCARD_REMOVE     0x11 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_MCU_UPGRADE		0x12 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_BAT_LOW			0x13 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_PWR_LOST			0x14 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_SYS_SLEEP			0x15 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_CFG_RESTORE		0x16 //MCU�ڲ�ʹ�ã�����������Ϣ����
#define ALARM_MSG_FORMAT_DISK		0x17 //MCU�ڲ�ʹ�ã�����������Ϣ����

/* Ӳ�����֪ͨ */
#define HW_CHANGE_HDD		0
#define HW_CHANGE_ICCARD	1
#define HW_CHANGE_BAT_LOW	2
#define HW_CHANGE_PWR_LOST	3
#define HW_CHANGE_HOST_SD	4

#define HW_CHANGE_DETACH	0
#define HW_CHANGE_ATTACH	1


/* ������������ */
#define GB_CMD_SET_CAR_INFO			    0x82 //���ó�����Ϣ
#define GB_CMD_SET_EQUIPT_TIME			0x83 //���ð�װ����
#define GB_CMD_SET_STATUS_VALUE		    0x84 //����״ֵ̬
#define GB_CMD_SET_DEV_TIME			    0xC2 //�����豸ʱ��
#define GB_CMD_SET_PULSE_COEFFICIENT	0xC3 //��������ϵ��
#define GB_CMD_SET_INTIAL_MILEAGE		0xC4 //���ó�ʼ���

/* ����춨���� */
#define GB_CMD_ENTER_TEST_MODE				0xE0 //����춨ģʽ
#define GB_CMD_ENTER_MILEAGE_ERROR_TEST 	0xE1 //�������������ģʽ
#define GB_CMD_ENTER_PULSE_COEF_ERROR_TEST	0xE2 //��������ϵ��������ģʽ
#define GB_CMD_ENTER_REAL_TIME_ERROR_TEST	0xE3 //����ʵʱʱ��������ģʽ
#define GB_CMD_ENTER_NORMAL_MODE			0xE4 //�˳��춨ģʽ


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



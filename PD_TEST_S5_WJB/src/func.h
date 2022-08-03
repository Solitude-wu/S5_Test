/*
 * func.h
 *
 *  Created on: 2020Äê10ÔÂ19ÈÕ
 *      Author: WIN
 */

#ifndef FUNC_H_
#define FUNC_H_

#include "common.h"

#define FRAME_CMD_INDEX			2
#define FRAME_DATA_INDEX		7

#define CMD_TYPE_CAR_SIG		0x01
#define CMD_TYPE_ADAS_INFO		0x02
#define CMD_TYPE_DID_RW			0x03
#define CMD_TYPE_UDS			0x04
#define CMD_TYPE_ROUTINE		0x05
#define CMD_TYPE_ECU_RESET		0x06
#define CMD_TYPE_GET_VERSION	0x07
#define CMD_TYPE_FILE_XFER		0x08
#define CMD_TYPE_RADAR_DATA		0x09
#define CMD_TYPE_DIAGNOSE		0x0A

void mcb_init(uint32_t mem_start, uint32_t mem_end);
void *malloc(unsigned int size);
void free(void *mem);

uint8_t XorCheckSum(uint8_t *input, uint16_t len);

void uart1_xfer_init(void);
void uart_msg_handler(void);
void car_signal_report(int param);
void uart1_trigger_send(void);
void uart1_tx_enqueue_msg(list_t *pmsg);

void can_init(void);
void feed_radar_data(void *data);

status_t NVM_Init(void);
status_t erase_upgrade_flash(uint32_t addr, uint32_t size);
status_t write_flash(uint32_t addr, uint32_t size, uint8_t *buf);

void reset_mcu_after(uint32_t ms);

void timeout_monitor(int param);
void reload_timeout_counter(int tid);
int register_timeout_monitor(uint32_t timeout_ms, void (*timeout_callback)(int tid, int is_timeout));
void unregister_timeout_monitor(int tid);
void change_timeout_value(int tid, uint32_t timeout_ms);

void cpu_timeout_callback(int tid, int is_timeout);
void comm_uart_timeout_callback(int tid, int is_timeout);
void car_can_timeout_callback(int tid, int is_timeout);
void radar_can_timeout_callback(int tid, int is_timeout);
void system_status_monitor(int param);
void sys_voltage_sample_task(int param);

#endif /* FUNC_H_ */

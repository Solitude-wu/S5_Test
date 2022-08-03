#include <stdio.h>
#include "bsp.h"
#include "common.h"
#include "string.h"
#include "gb_t19056_def.h"

#define UART_RX_TIMEOUT         200

/* 注：NUC240只有UART0和UART1支持DMA */
#define UART_TX_DMA_CH0 0
#define UART_TX_DMA_CH1 1

#define UART_CMD_QUEUE_LEN		6
#define UART_RESP_QUEUE_LEN		6


#define CMD_START_TEST	0x01
#define CMD_GET_SW_VER	0x02
#define CMD_GET_HW_VER	0x03
#define CMD_GET_SN		0x04
#define CMD_GET_PERI_ST	0x05
#define CMD_SLEEP		0x10
#define CMD_END_TEST	0x80
#define CMD_UI_READ		0x81
#define CMD_VOLT_TEST	0x82

#define PROTO_CMD_HEAD1			0x51
#define PROTO_CMD_HEAD2			0xA5
#define PROTO_FRAME_HEAD_INDEX	0
#define PROTO_CMD_INDEX			2
#define PROTO_DATA_LEN_INDEX	3
#define PROTO_HEADER_LEN		5
#define PROTO_DATA_START		4


ComInfo_t comInfo;

uint8_t GB19056_ChkDataFrame(uint8_t *buf)
{
    uint8_t chksum = 0;
    uint16_t len;

#if 0
    len = ((uint16_t)buf[3]<<8)|buf[4];
    chksum = GB19056_CalcCheckSum(buf, len+6);
#else
    uint16_t i;
    len = (((uint16_t)buf[3]<<8)|buf[4]) + 6;
    for (i = 0; i < len; i++) {
        chksum = chksum^buf[i];
    }
#endif
    return chksum == buf[len];
}

uint16_t FormatCmdRespBuffer(uint16_t hdr_type, uint8_t cmd, uint8_t *buf, uint8_t *data, uint16_t data_len)
{
    GB19056_PROTO_t *hdr;
    uint8_t chksum;

    hdr = (GB19056_PROTO_t *)buf;
    hdr->header1 = hdr_type & 0x00FF;
    hdr->header2 = hdr_type >> 8;
    hdr->cmd = cmd;
    hdr->len_h = data_len >> 8;
    hdr->len_l = data_len &0x0ff;
    hdr->res = 0;
    /* 调用该函数时可以预先把data数据放到payload里，这里判断如果data地址与payload地址一致则不拷贝 */
    if (data && data_len)
        memcpy(&buf[GB_CMD_HEADER_SIZE], data, data_len);
    chksum = GB19056_CalcCheckSum(buf, data_len+6);
    buf[data_len+6] = chksum;
    return data_len+7;
}


static void SC2ComInit(uint32_t baud)
{
	/* Enable SC1 module clock */
	CLK_EnableModuleClock(SC2_MODULE);
	/* Select SC1 module clock source */
	CLK_SetModuleClock(SC2_MODULE, CLK_CLKSEL3_SC2_S_HXT, CLK_CLKDIV1_SC2(1));

	/* Set PA multi-function pins for SC UART mode */
	SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk | SYS_GPA_MFP_PA13_Msk);
	SYS->GPA_MFP |= SYS_GPA_MFP_PA12_UART5_RXD | SYS_GPA_MFP_PA13_UART5_TXD;
	SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA6_Msk | SYS_ALT_MFP_PA7_Msk);
	SYS->ALT_MFP |= SYS_ALT_MFP_PA12_UART5_RXD | SYS_ALT_MFP_PA13_UART5_TXD;
	SYS->ALT_MFP1 &= ~(SYS_ALT_MFP1_PA6_Msk | SYS_ALT_MFP1_PA7_Msk);
	SYS->ALT_MFP1 |= SYS_ALT_MFP1_PA12_UART5_RXD | SYS_ALT_MFP1_PA13_UART5_TXD;

	SCUART_Open(SC2, baud);
	
	SCUART_SetLineConfig(SC2, baud, SCUART_CHAR_LEN_8, SCUART_PARITY_EVEN, SCUART_STOP_BIT_1);
	// Enable receive interrupt
	//SCUART_ENABLE_INT(SC2, SC_IER_RDA_IE_Msk);
	//NVIC_EnableIRQ(SC012_IRQn);
	
}

void SC2SendRadarData(void)
{
	uint8_t txbuf[] = {0x55, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x04, 0xfe, 0xff};
	SCUART_Write(SC2, txbuf, sizeof(txbuf));
}

void BoardComInit(void)
{
#if 1
   /* Enable UART module clock */
   CLK_EnableModuleClock(UART1_MODULE);

   /* Select UART module clock source */
   CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));
   //CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

   /* Set GPB multi-function pins for UART0 RXD and TXD */
   /* Set GPB multi-function pins for UART1 RXD, TXD, nRTS and nCTS */

   SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk |
                     SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk );

   SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD |
                    SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD );
   
   SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD14_Msk | SYS_GPD_MFP_PD15_Msk);

	SYS->GPD_MFP |= (SYS_GPD_MFP_PD14_UART2_RXD | SYS_GPD_MFP_PD15_UART2_TXD);

   SYS_ResetModule(UART1_RST);
   UART_Open(UART1, 115200);
   UART_SetLine_Config(UART1, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

   UART_EnableInt(UART1, UART_IER_RDA_IEN_Msk);

   NVIC_EnableIRQ(UART1_IRQn);
   
   NVIC_SetPriority(UART1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

   comInfo.uart = UART1;
   comInfo.rxqueue = NULL;
   comInfo.txqueue = NULL;
   comInfo.rxbuf = 0;
#endif
	SC2ComInit(19200);
#if 1
	CLK_EnableModuleClock(UART0_MODULE);

   /* Select UART module clock source */
   CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_CLKDIV_UART(1));

   SYS_ResetModule(UART0_RST);
   UART_Open(UART0, 115200);
   UART_SetLine_Config(UART0, 115200, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);
#endif
}

void ComSendBuffer(UART_T* pCom, uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    while (u32WriteBytes--) {
        while(pCom->FSR & UART_FSR_TX_FULL_Msk) {} /* Wait for fifo is not full */
        pCom->DATA = *pu8TxBuf++;
    }
}

static uint8_t CheckSum(uint8_t *buf, uint16_t len)
{
	uint8_t val = 0;
	while(len--) {
		val ^= *buf++;
	}
	return val;
}

void Uart1IrqHandler(UART_T *pCom)
{
    uint8_t ch;
	list_t *plist_node;
	ComInfo_t *pComInfo = &comInfo;

    while(!(pCom->FSR & UART_FSR_RX_EMPTY_Msk)) {
        ch = pCom->RBR;
		pComInfo->chksum ^= ch;
        switch(pComInfo->rx_cnt) {
        case 0: /* header */
        	if (ch == 0x51) {
				pComInfo->tmprxbuf[pComInfo->rx_cnt++] = ch;
        	} else {
				pComInfo->chksum = 0;
				pComInfo->rx_cnt = 0;
			}
        	break;
        case 1: /* protocal ver */
			if (ch == 0xA5) {
				pComInfo->tmprxbuf[pComInfo->rx_cnt++] = ch;
			} else {
        		pComInfo->chksum = 0;
				pComInfo->rx_cnt = 0;
        	}
        	break;
		case 2: /* cmd byte */
		case 3: /* length high byte */
		case 4: /* length low byte */
			pComInfo->tmprxbuf[pComInfo->rx_cnt++] = ch;
			if (pComInfo->rx_cnt == 5) {
				pComInfo->cmd_len = (uint16_t)pComInfo->tmprxbuf[pComInfo->rx_cnt - 2] << 8 | pComInfo->tmprxbuf[pComInfo->rx_cnt - 1];
				if (pComInfo->cmd_len > UART_PROTO_BUF_MAX_SIZE) {
					pComInfo->rx_cnt = 0;
					pComInfo->chksum = 0;
				} else {
					plist_node = malloc(pComInfo->cmd_len + sizeof(list_t));
					if (!plist_node) {
						pComInfo->rx_cnt = 0;
						pComInfo->chksum = 0;
						pComInfo->rxbuf = 0;
						pComInfo->cmd_len = 0;
					} else {
						pComInfo->rxbuf = LIST_TO_BUFF(plist_node);
						plist_node->next = 0;
						memcpy(pComInfo->rxbuf, pComInfo->tmprxbuf, pComInfo->rx_cnt);
					}
				}
			}
        	break;
        default:
			if (pComInfo->rxbuf) {
				pComInfo->rxbuf[pComInfo->rx_cnt++] = ch;
				if (pComInfo->rx_cnt >= pComInfo->cmd_len) {
					if (pComInfo->chksum == 0) {
						plist_node = BUFF_TO_LIST(pComInfo->rxbuf);
						list_insert_tail(&pComInfo->rxqueue, plist_node);
					} else {
						printf("chksum err, expect: 0x%02x\n", ch^pComInfo->chksum);
						free(plist_node);
					}
					pComInfo->rx_cnt = 0;
					pComInfo->chksum = 0;
					pComInfo->rxbuf = 0;
					pComInfo->cmd_len = 0;
				}
			}
        	break;
        }
    }
}

void UART02_IRQHandler(void)
{
    if((UART0->IER & UART0->ISR) & 0x3)    /* UART data received interrupt or tx empty int(if enabled) */
	{
		//Uart0IrqHandler(UART0);
	}
}

void UART1_IRQHandler(void)
{
    if((UART1->IER & UART1->ISR) & 0x3)    /* UART data received interrupt or tx empty int(if enabled) */
    {
    	Uart1IrqHandler(UART1);
    }
}

void FlushUartFifo(void)
{
	//UART0->FCR |= UART_FCR_TFR_Msk | UART_FCR_RFR_Msk;
	UART1->FCR |= UART_FCR_TFR_Msk | UART_FCR_RFR_Msk;
	//UART2->FCR |= UART_FCR_TFR_Msk | UART_FCR_RFR_Msk;
}

void WaitUartFifoEmpty(void)
{
	//while(!(UART0->FSR & UART_FSR_TX_EMPTY_Msk));
	while(!(UART1->FSR & UART_FSR_TX_EMPTY_Msk));
	//while(!(UART2->FSR & UART_FSR_TX_EMPTY_Msk));
}

void FlushTxRxQueue(void)
{
	while (comInfo.txqueue) {
		list_remove(&comInfo.txqueue, comInfo.txqueue);
	}
	while (comInfo.rxqueue) {
		list_remove(&comInfo.rxqueue, comInfo.rxqueue);
	}
}



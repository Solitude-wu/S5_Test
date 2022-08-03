#include "NUC230_240.h"
#include "stdio.h"
#include "string.h"
#include "common.h"
#include "bsp.h"
#include "gb_t19056_def.h"

#define INVALID_CAN_ID	0xFFFFFFFF

extern volatile uint8_t print_onoff;

CAN_T  *tCAN[2] = {CAN0, CAN1};
uint8_t can_timer_id[2] = {0xff, 0xff};

static uint32_t canid_filters[2][CAN_RECV_QUEUE_LEN];

static void CAN_XferMsgToUart(int port);

void CAN_Init(uint8_t port, uint32_t baud)
{
	if (port > 1)
		return;

    printf("CAN_Init(%d, %d)\n", port, baud);
    CAN_Close(tCAN[port]);
    CAN_ClearAllRecvId(port);

    if (port == 0) {
    	if (baud != 0) {
			/* Enable CAN module clock */
			CLK_EnableModuleClock(CAN0_MODULE);
			/* Set PD multi-function pins for CANTX0, CANRX0 */
			SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
			SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;
			NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 3);
			NVIC_EnableIRQ(CAN0_IRQn);
			CAN_Open(tCAN[port],  baud, CAN_NORMAL_MODE);
			CAN_EnableInt(tCAN[port], CAN_CON_IE_Msk|CAN_CON_SIE_Msk);     /* Enable CAN interrupt and corresponding NVIC of CAN */
    	} else {
    		CLK_DisableModuleClock(CAN0_MODULE);
    		NVIC_DisableIRQ(CAN0_IRQn);
    	}
    } else {
    	if (baud != 0) {
			/* Enable CAN module clock */
			CLK_EnableModuleClock(CAN1_MODULE);
			/* Set PA multi-function pins for CANTX1, CANRX1 */
			SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD14_Msk | SYS_GPD_MFP_PD15_Msk);
			SYS->GPD_MFP |= SYS_GPD_MFP_PD14_CAN1_RXD | SYS_GPD_MFP_PD15_CAN1_TXD;
			SYS->ALT_MFP2 |= SYS_ALT_MFP2_PD14_CAN1_RXD | SYS_ALT_MFP2_PD15_CAN1_TXD;
			NVIC_SetPriority(CAN1_IRQn, (1 << __NVIC_PRIO_BITS) - 2);
			NVIC_EnableIRQ(CAN1_IRQn);
			CAN_Open(tCAN[port],  baud, CAN_NORMAL_MODE);
			CAN_EnableInt(tCAN[port], CAN_CON_IE_Msk|CAN_CON_SIE_Msk);
    	} else {
			CLK_DisableModuleClock(CAN1_MODULE);
			NVIC_DisableIRQ(CAN1_IRQn);
		}
    }
}

void CAN_DeInit(void)
{
//	if (can_timer_id[0] < SW_TIMER_NUM) {
//		SoftTimerFree(&can_timer_id[0]);
//	}
//	if (can_timer_id[1] < SW_TIMER_NUM) {
//		SoftTimerFree(&can_timer_id[1]);
//	}
	/* Disable CAN module clock */
	CLK_DisableModuleClock(CAN0_MODULE | CAN1_MODULE);
	/* Set PD multi-function pins for CANTX0, CANRX0 */
	SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk | SYS_GPD_MFP_PD14_Msk | SYS_GPD_MFP_PD15_Msk);
	SYS->ALT_MFP &= ~(SYS_ALT_MFP_PA10_Msk | SYS_ALT_MFP_PA11_Msk);
	SYS->ALT_MFP2 &= ~(SYS_ALT_MFP2_PD14_CAN1_RXD | SYS_ALT_MFP2_PD15_CAN1_TXD);
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);

	CAN_Close(tCAN[0]);
	CAN_ClearAllRecvId(0);

	CAN_Close(tCAN[1]);
	CAN_ClearAllRecvId(1);
}

uint8_t FindFilterId(uint8_t port, uint32_t id)
{
	uint8_t i;
	for (i = 0; i < CAN_RECV_QUEUE_LEN; i++) {
		if (canid_filters[port][i] == id)
			break;
	}
	return i;
}


/* Declare a CAN message structure */
CanMsgQueue_t CanRecvMsgQ[2][CAN_RECV_QUEUE_LEN] = {0};
CanMsgQueue_t *pCanRecvQHead[2] = {NULL};
CanMsgQueue_t CanSendMsgQ[2][CAN_SEND_QUEUE_LEN] = {0};
CanMsgQueue_t *pCanSendQHead[2] = {NULL};

CanMsgQueue_t *CanAllocMsg(int port, CanMsgQueue_t **qHead)
{
    CanMsgQueue_t *ret = NULL;
    uint8_t i;
    DisableInterrupt();
    if (qHead == &pCanRecvQHead[port]) {
        if (*qHead == NULL) {
            *qHead = CanRecvMsgQ[port];
            (*qHead)->inuse = 1;
            (*qHead)->next = NULL;
            ret = *qHead;
        } else {
            for (i=0; i<CAN_RECV_QUEUE_LEN; i++) {
                if(!CanRecvMsgQ[port][i].inuse)
                    break;
            }

            if (i < CAN_RECV_QUEUE_LEN) {
                CanRecvMsgQ[port][i].inuse = 1;
                CanRecvMsgQ[port][i].next = NULL;
                ret = &CanRecvMsgQ[port][i];
            } else {
                ret = NULL;
            }
        }
    } else if (qHead == &pCanSendQHead[port]) {
        if (*qHead == NULL) {
            *qHead = CanSendMsgQ[port];
            (*qHead)->inuse = 1;
            (*qHead)->next = NULL;
            ret = *qHead;
        } else {
            for (i=0; i<CAN_SEND_QUEUE_LEN; i++) {
                if(!CanSendMsgQ[port][i].inuse)
                    break;
            }
            if (i < CAN_SEND_QUEUE_LEN) {
                CanSendMsgQ[port][i].inuse = 1;
                CanSendMsgQ[port][i].next = NULL;
                ret = &CanSendMsgQ[port][i];
            } else {
                ret = NULL;
            }
        }
    }
    EnableInterrupt();
    return ret;
}

void CanFreeMsg(CanMsgQueue_t *pMsg)
{
    if (pMsg) {
    	DisableInterrupt();
        pMsg->inuse = 0;
        pMsg->next = NULL;
        EnableInterrupt();
    }
}

/* 以CAN ID从小到大的顺序插入消息到队列 */
void CanMsgQueueSortedInsert(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg)
{
    CanMsgQueue_t *p = *qHead, *q;
    if (!qHead || !pMsg) {
    	return;
    }
    if(*qHead == NULL) {
        *qHead = pMsg;
        return;
    } else if (*qHead == pMsg){
        return;
    } else {
    	DisableInterrupt();
        q = p;
        while(p) {
            if (p->msg.Id <= pMsg->msg.Id) {
                q = p;
                p = p->next;
            } else {
                if (p == *qHead) {
                    *qHead = pMsg;
                    pMsg->next = p;
                } else {
                    q->next = pMsg;
                    pMsg->next = p;
                }
                EnableInterrupt();
                return;
            }
        }
        q->next = pMsg;
        EnableInterrupt();
    }
}

void CanMsgRemove(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg)
{
    CanMsgQueue_t *p = *qHead, *q;
    if(*qHead == NULL || pMsg == NULL) {
        return;
    } else {
        if (*qHead == pMsg) {
        	DisableInterrupt();
            *qHead = pMsg->next;
            pMsg->inuse = 0; //remove后自动释放掉，避免再调一次free
            EnableInterrupt();
        } else {
        	DisableInterrupt();
            q = p;
            while(p != pMsg && p != NULL) {q = p; p = p->next;}
            if (p) {
                q->next = pMsg->next;
                pMsg->inuse = 0;
            }
            EnableInterrupt();
        }
    }
}

void CanFreeMsgQueue(int port, CanMsgQueue_t **qHead)
{
    uint8_t i;
    DisableInterrupt();
    for(i=0;i<CAN_SEND_QUEUE_LEN;i++)
    {
        CanSendMsgQ[port][i].inuse = 0;
        CanSendMsgQ[port][i].next = NULL;
    }
    *qHead = NULL;
    EnableInterrupt();
}

uint8_t CAN_GetFreeRecvMsgBufCnt(int port)
{
    uint8_t i,ret = 0;
    for(i=0; i<CAN_RECV_QUEUE_LEN; i++) {
        if (!CanRecvMsgQ[port][i].inuse) {
            ret++;
        }
    }
    return ret;
}


static void CAN_XferMsgToUart(int port)
{
    uint16_t i=0;
    CanMsgQueue_t *pMsg, *pTmp;
	list_t *ptxlist_node;
	static uint16_t frame_len = 0;
	static uint8_t *pbuf;
	static uint8_t partial_flag = 0;
	uint8_t new_frame = 0;

    if (pCanRecvQHead[port] == NULL) {
        return;
    }
	pMsg = pCanRecvQHead[port];
	while(pMsg) {
        if(pMsg->msg.Data[0] & 0x80) {
			if (partial_flag == 0 && (pMsg->msg.Data[0] & 0x7f) == 0) {
				partial_flag = 1;
				frame_len = ((uint16_t)pMsg->msg.Data[4] << 8) | pMsg->msg.Data[5];
				if (frame_len == 0) {
					partial_flag = 0;
					goto lable1;
				}
				ptxlist_node = malloc(sizeof(list_t) + frame_len);
				if (!ptxlist_node) {
					printf("uart tx queue is full!\n");
					new_frame = 0;
					partial_flag = 0;
					goto lable1;
				}
				pbuf = LIST_TO_BUFF(ptxlist_node);
			}
#if 0
			else if (partial_flag == 1 && (pMsg->msg.Data[0] & 0x7f) == 0) {
				/* last partial serial packets is incomplete and a new partial pack reveived */
				
			}
#endif
		} else {
			if (partial_flag == 1) {
				new_frame = 1;
			} else if((pMsg->msg.Data[0] & 0x7f) == 0){
				frame_len = ((uint16_t)pMsg->msg.Data[4] << 8) | pMsg->msg.Data[5];
				if (frame_len == 0) {
					partial_flag = 0;
					goto lable1;
				}
				ptxlist_node = malloc(sizeof(list_t) + i);
				if (!ptxlist_node) {
					printf("uart tx queue is full!\n");
					new_frame = 0;
					partial_flag = 0;
					goto lable1;
				}
				pbuf = LIST_TO_BUFF(ptxlist_node);
				new_frame = 1;
			}
			partial_flag = 0;
		}
		if (partial_flag || new_frame) {
			i = (uint16_t)(pMsg->msg.Data[0]&0x7F)*7;
			if (i > UART_PROTO_BUF_MAX_SIZE) {
				/* abandon long frame */
				printf("Msg length too long!");
				partial_flag = 0;
			} else {
				if (i + 7 > frame_len) {
					memcpy(&pbuf[i], &pMsg->msg.Data[1], frame_len - i);
				} else {
					memcpy(&pbuf[i], &pMsg->msg.Data[1], 7);
				}
			}
		}
lable1:
        pTmp = pMsg->next;
        CanMsgRemove(&pCanRecvQHead[port], pMsg);
        pMsg = pTmp;
        if (new_frame == 1) {
			ptxlist_node = BUFF_TO_LIST(pbuf);
			list_insert_tail(&comInfo.txqueue, ptxlist_node);
			new_frame = 0;
			partial_flag = 0;
		}
    }
}

static void ForwardMsgToUart(STR_CANMSG_T *msg)
{
    uint16_t i=0;
	list_t *ptxlist_node;
	static uint16_t frame_len = 0;
	static uint8_t *pbuf;
	static uint8_t partial_flag = 0;
	uint8_t new_frame = 0;

	if(msg->Data[0] & 0x80) {
		if (partial_flag == 0 && (msg->Data[0] & 0x7f) == 0) {
			partial_flag = 1;
			frame_len = ((uint16_t)msg->Data[4] << 8) | msg->Data[5];
			if (frame_len == 0) {
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			ptxlist_node = malloc(sizeof(list_t) + frame_len);
			if (!ptxlist_node) {
				printf("uart tx queue is full!\n");
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			pbuf = LIST_TO_BUFF(ptxlist_node);
		}
#if 0
		else if (partial_flag == 1 && (pMsg->msg.Data[0] & 0x7f) == 0) {
			/* last partial serial packets is incomplete and a new partial pack reveived */
			
		}
#endif
	} else {
		if (partial_flag == 1) {
			new_frame = 1;
		} else if((msg->Data[0] & 0x7f) == 0){
			frame_len = ((uint16_t)msg->Data[4] << 8) | msg->Data[5];
			if (frame_len == 0) {
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			ptxlist_node = malloc(sizeof(list_t) + frame_len);
			if (!ptxlist_node) {
				printf("uart tx queue is full!\n");
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			pbuf = LIST_TO_BUFF(ptxlist_node);
			new_frame = 1;
		}
		partial_flag = 0;
	}
	if (partial_flag || new_frame) {
		i = (uint16_t)(msg->Data[0]&0x7F)*7;
		if (i > UART_PROTO_BUF_MAX_SIZE || i >= frame_len) {
			/* abandon long frame */
			printf("Msg length too long!");
			partial_flag = 0;
			new_frame = 0;
		} else if (pbuf) {
			if (i + 7 > frame_len) {
				memcpy(&pbuf[i], &msg->Data[1], frame_len - i);
			} else {
				memcpy(&pbuf[i], &msg->Data[1], 7);
			}
		}
	}
	if (new_frame == 1 && pbuf) {
		ptxlist_node = BUFF_TO_LIST(pbuf);
		list_insert_tail(&comInfo.txqueue, ptxlist_node);
		new_frame = 0;
		partial_flag = 0;
		pbuf = NULL;
	}
}


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN interrupt event                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_MsgInterrupt(CAN_T *tCan, uint32_t u32IIDR)
{
    uint8_t i;
#if 0
    CanMsgQueue_t *pMsg;
#else
	STR_CANMSG_T msg;
#endif
    u32IIDR -= 1;
    int port = (tCan == CAN0) ? 0:1;

    if (tCan->STATUS & CAN_STATUS_RXOK_Msk) {
        tCan->STATUS &= ~CAN_STATUS_RXOK_Msk;
#if 0
//        if (can_timer_id[port] > SW_TIMER_NUM) {
//        	can_timer_id[port] = SoftTimerAlloc();
//        	SoftTimerStart(can_timer_id[port], 20, TMR_AUTO_MODE, CAN_XferMsgToUart, port);
//        }

        // recv ok
        pMsg = CanAllocMsg(port, &pCanRecvQHead[port]);
        if (pMsg) {
            CAN_Receive(tCan, u32IIDR, &pMsg->msg);
            //printf("can%d recv1: id=%x\n", port, pMsg->msg.Id);
            CanMsgQueueSortedInsert(&pCanRecvQHead[port], pMsg);
//            if (CAN_GetFreeRecvMsgBufCnt(port) < 1) {
//                SoftTimerReload(can_timer_id[port]);
                CAN_XferMsgToUart(port);
//            }
        } else {
            // 如果报文没有来得及发送则覆盖相同报文的id的老报文
			if (u32IIDR < CAN_RECV_QUEUE_LEN) {
				for (i = 0; i < CAN_RECV_QUEUE_LEN; i++) {
					if (CanRecvMsgQ[port][i].msg.Id == (canid_filters[port][u32IIDR] & 0x7FFFFFFF) &&
							CanRecvMsgQ[port][i].msg.IdType == (canid_filters[port][u32IIDR] >> 31)) {
						pMsg = &CanRecvMsgQ[port][i];
						CAN_Receive(tCan, u32IIDR, &pMsg->msg);
						//printf("rep: id=0x%x\n", canid_filters[port][u32IIDR]);
						break;
					}
				}
				if (i >= CAN_RECV_QUEUE_LEN) {
					printf("CAN%d recv buffer run out, drop frame(id=0x%x)!\n", port, canid_filters[port][u32IIDR]);
#if 0
					puts("msg in queue:\n");
					for (i=0;i<CAN_RECV_QUEUE_LEN;i++) {
						printf("id[%d]=0x%x ", i, CanRecvMsgQ[port][i].msg.Id);
					}
					puts("\n");
#endif
				}
			}
			CAN_XferMsgToUart(port);
        }
#else
		CAN_Receive(tCan, u32IIDR, &msg);
		ForwardMsgToUart(&msg);
#endif
    }
    if(tCan->STATUS & CAN_STATUS_TXOK_Msk) {
        tCan->STATUS &= ~CAN_STATUS_TXOK_Msk;
        // send ok
        CanMsgRemove(&pCanSendQHead[port], pCanSendQHead[port]);
        CAN_NormalModeTx(port);
    }
}

//extern void CAN_LeaveInitMode(CAN_T *tCAN);
/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
#if 0
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
            printf("RxOK INT\n") ;

        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            printf("TxOK INT\n") ;
        }
#endif
        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("CAN%d BOFF INT(0x%02x)\n", 0, CAN0->STATUS);
            /* 总线出错关闭后重新进入Init模式，一旦Init被CPU清除，设备
             * 将在恢复正常操作之前等待129个总线空闲（129*11个连续的隐性位）。
             * 在bus-off修复时序的最后，错误管理计数器将被复位。
            */
            CAN0->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            CanFreeMsgQueue(0, &pCanSendQHead[0]);
            return;
        }

        if((CAN0->STATUS & CAN_STATUS_EWARN_Msk) && !(CAN0->STATUS & (CAN_STATUS_TXOK_Msk|CAN_STATUS_RXOK_Msk))) {
            printf("CAN%d EWARN INT(0x%02x)\n", 0, CAN0->STATUS) ;
            /* 发送失败时从队列移除该消息，继续发送下一个 */
            CanMsgRemove(&pCanSendQHead[0], pCanSendQHead[0]);
            /* 如果队列非空则发送，否则关掉CAN错误中断，不关的话会一直触发 */
            if (pCanSendQHead[0]) {
                CAN_NormalModeTx(0);
            } else {
                CAN0->CON &= ~CAN_CON_SIE_Msk;
            }
        }
    } else if(u8IIDRstatus != 0) {
        CAN_MsgInterrupt(CAN0, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN0, ((CAN0->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN0->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN1 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN1->IIDR;

    if(u8IIDRstatus == 0x00008000) {      /* Check Status Interrupt Flag (Error status Int and Status change Int) */
        /**************************/
        /* Status Change interrupt*/
        /**************************/
#if 0
        if(CAN1->STATUS & CAN_STATUS_RXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
            printf("RxOK INT\n") ;

        }

        if(CAN1->STATUS & CAN_STATUS_TXOK_Msk) {
            CAN1->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
            printf("TxOK INT\n") ;
        }
#endif
        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN1->STATUS & CAN_STATUS_BOFF_Msk) {
            printf("CAN%d BOFF INT(0x%02x)\n", 1, CAN1->STATUS);
            /* 总线出错关闭后重新进入Init模式，一旦Init被CPU清除，设备
             * 将在恢复正常操作之前等待129个总线空闲（129*11个连续的隐性位）。
             * 在bus-off修复时序的最后，错误管理计数器将被复位。
            */
            CAN1->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
            CanFreeMsgQueue(1, &pCanSendQHead[1]);
            return;
        }

        if((CAN1->STATUS & CAN_STATUS_EWARN_Msk) && !(CAN1->STATUS & (CAN_STATUS_TXOK_Msk|CAN_STATUS_RXOK_Msk))) {
            printf("CAN%d EWARN INT(0x%02x)\n", 1, CAN1->STATUS) ;
            /* 发送失败时从队列移除该消息，继续发送下一个 */
            CanMsgRemove(&pCanSendQHead[1], pCanSendQHead[1]);
            /* 如果队列非空则发送，否则关掉CAN错误中断，不关的话会一直触发 */
            if (pCanSendQHead[1]) {
                CAN_NormalModeTx(1);
            } else {
                CAN1->CON &= ~CAN_CON_SIE_Msk;
            }
        }
    } else if(u8IIDRstatus != 0) {
        CAN_MsgInterrupt(CAN1, u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(CAN1, ((CAN1->IIDR) - 1)); /* Clear Interrupt Pending */
    } else if(CAN1->WU_STATUS == 1) {
        printf("Wake up\n");

        CAN1->WU_STATUS = 0;    /* Write '0' to clear */
    }

}


/*---------------------------------------------------------------------------------------------------------*/
/* Reset message interface parameters                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ResetIF(CAN_T *tCan, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCan->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCan->IF[u8IF_Num].CMASK    = 0x0;
    tCan->IF[u8IF_Num].MASK1    = 0x0;          // useless in basic mode
    tCan->IF[u8IF_Num].MASK2    = 0x0;          // useless in basic mode
    tCan->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCan->IF[u8IF_Num].ARB2     = 0x0;          // MsgVal, eXt, xmt, ID28~16
    tCan->IF[u8IF_Num].MCON     = 0x0;          // DLC
    tCan->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCan->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCan->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCan->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*----------------------------------------------------------------------------*/
/*  Send Tx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/

void CAN_NormalModeTx(int port)
{
    uint32_t u32MsgNum;
    CAN_T *tCan = (port == 0) ? CAN0:CAN1;

    if(tCan->STATUS & CAN_STATUS_BOFF_Msk) {
        printf("BOFF(0x%02x)\n", tCan->STATUS);
        tCan->CON &= (~(CAN_CON_INIT_Msk | CAN_CON_CCE_Msk));
        return;
    }

    if (pCanSendQHead[port] && !(tCan->IF[1].CREQ & CAN_IF_CREQ_BUSY_Msk)) {
        u32MsgNum = ((uint32_t)pCanSendQHead[port] - (uint32_t)CanSendMsgQ[port])/sizeof(CanMsgQueue_t) + CAN_RECV_QUEUE_LEN;
        CAN_SetTxMsg(tCan, MSG(u32MsgNum), &pCanSendQHead[port]->msg);
        /* 前面如果出现发送失败状态中断会被手动关掉（中断处理函数里），这里重新打开 */
        tCan->CON |= CAN_CON_SIE_Msk;
        CAN_TriggerTxMsg(tCan, MSG(u32MsgNum));
    }
}

/*----------------------------------------------------------------------------*/
/*  Send Rx Msg by Normal Mode Function (With Message RAM)                    */
/*----------------------------------------------------------------------------*/
uint8_t CAN_SetRecvId(uint8_t port, uint32_t can_id)
{
	uint8_t index;

	if (FindFilterId(port, can_id) < CAN_RECV_QUEUE_LEN) {
		/* 当前CAN过滤ID已配置 */
		return 0;
	}
	index = FindFilterId(port, INVALID_CAN_ID);
	if (index >= CAN_RECV_QUEUE_LEN) {
		/* 无可用的过滤ID */
		return 1;
	}

	printf("CAN_SetRecvId(port=%d, index=%d, id=0x%x)\n", port, index, can_id);

    if(CAN_SetRxMsg(tCAN[port], MSG(index), can_id>>31, can_id&0x7FFFFFFF) == FALSE) {
        printf("Set Rx Msg Object failed\n");
        return 1;
    }
    canid_filters[port][index] = can_id;
    return 0;
}

void CAN_ClearAllRecvId(uint8_t port)
{
	uint32_t i;

	for (i=0; i<CAN_RECV_QUEUE_LEN; i++) {
		tCAN[port]->IF[1].ARB2 = 0;
		canid_filters[port][i] = INVALID_CAN_ID;
	}
}

void CAN_TriggerQueueSend(uint8_t port)
{
    CAN_NormalModeTx(port);
}

void CanForwardUartMsg(uint8_t port, uint8_t *msg, uint16_t len)
{
	uint8_t *p = msg;
	CanMsgQueue_t *pCanMsg;
	uint16_t i=0, part_flag, tmp;
	uint8_t pid = 0;
	
	do {
		pCanMsg = CanAllocMsg(port, &pCanSendQHead[port]);
		if (pCanMsg) {
			pCanMsg->msg.Id = COMMUNICATION_CAN_ID;
			pCanMsg->msg.FrameType = CAN_DATA_FRAME;
			pCanMsg->msg.IdType = CAN_EXT_ID;
			pCanMsg->msg.DLC = 8;
			tmp = len - i;
			if (len <= 7) {
				part_flag = 0;
				pCanMsg->msg.DLC = len + 1;
			} else if (tmp > 7) {
				part_flag = 0x80;
			} else {
				part_flag = 0;
				if (tmp <= 7) {
					pCanMsg->msg.DLC = tmp + 1;
				}
			}

			pCanMsg->msg.Data[0] = part_flag | pid;
			memcpy(&pCanMsg->msg.Data[1], p+i, 7);

			CanMsgQueueSortedInsert(&pCanSendQHead[port], pCanMsg);
			i += 7;
			pid += 1;
		} else {
			printf("CAN send queue is full, drop %d packets\n", len-i);
			break;
		}
	} while(i < len);
	if (len) {
		CAN_TriggerQueueSend(port);
	}
}


#include "mqueue.h"
#include "NUC230_240.h"
#include "common.h"
//#define QUEUE_DEBUG

#ifdef QUEUE_DEBUG
#include <stdio.h>
#define queue_debug(fmt, ...) printf("[QUEUE]%s "fmt, __func__, ##__VA_ARGS__)
#else
#define queue_debug(fmt, ...)
#endif

#define osalExitCritical 	EnableInterrupt
#define osalEnterCritical	DisableInterrupt

static Queue_t sQueue[MAX_QUEUES];

static uint8_t queue_used_bitmap = 0;

Queue_t *create_queue(void* pitems, uint16_t size_of_item, uint8_t max_items)
{
	 uint8_t i;
	 osalEnterCritical();
	 for (i=0; i<MAX_QUEUES; i++) {
		 if ((queue_used_bitmap & (1<<i)) == 0) {
			 sQueue[i].head = 0;
			 sQueue[i].tail = 0;
			 sQueue[i].count = 0;
			 sQueue[i].max_items = max_items;
			 sQueue[i].pitems = pitems;
			 sQueue[i].item_size = size_of_item;
			 queue_used_bitmap |= (1<<i);
			 break;
		 }
	 }
	 osalExitCritical();
	 if (i == MAX_QUEUES) {
		 return 0;
	 } else {
		 queue_debug("0x%x\n\r", (uint32_t)&sQueue[i]);
		 return &sQueue[i];
	 }
}

void release_queue(Queue_t *q)
{
	uint8_t i;

	if (!q) {
		return;
	}

	osalEnterCritical();
	for (i=0; i<MAX_QUEUES; i++) {
		if (q == &sQueue[i]) {
			queue_used_bitmap &= ~(1<<i);
			break;
		}
	}
	osalExitCritical();
}

int8_t enqueue_item(Queue_t *q, void *item)
{
	if (!q) {
		queue_debug("Invalid param!\n\r");
		return -2;
	}

	if (q->count < q->max_items) {
		osalEnterCritical();
		q->lock_index = 0xFFFF;
		if (item) {
			memcpy(((char *)q->pitems + q->tail * q->item_size), item, q->item_size);
		}
		q->tail++;
		q->count++;
		if (q->tail == q->max_items) {
			q->tail = 0;
		}
		queue_debug("0x%x, count=%d,tail=%d,max=%d\n\r", (uint32_t)q, q->count, q->tail, q->max_items);
		osalExitCritical();
	} else {
		queue_debug("0x%x fail, count=%d,tail=%d,head=%d,max=%d\n\r", (uint32_t)q, q->count, q->tail, q->head, q->max_items);
		return -1;
	}

	return 0;
}

void *get_queue_tail(Queue_t *q)
{
	void *p = 0;

	if (!q) {
		return NULL;
	}

	if (q->lock_index == 0xFFFF)
		return 0;
	
	osalEnterCritical();
	if (q->count < q->max_items) {
		p = (char *)q->pitems + q->tail * q->item_size;
		q->lock_index = q->tail;
		queue_debug("0x%x, count=%d,tail=%d,max=%d\n\r", (uint32_t)q, q->count, q->tail, q->max_items);
	}
	osalExitCritical();
	return p;
}

void *get_queue_head(Queue_t *q)
{
	void *p = 0;

	if (!q) {
		queue_debug("Invalid param!\n\r");
		queue_debug("count=%d,head=%d,tail=%d\n\r", q->count, q->head, q->tail);
		return 0;
	}

	osalEnterCritical();
	if (q->count != 0) {
		p = (char *)q->pitems + q->head * q->item_size;
	}
	osalExitCritical();
	if (p != NULL) {
		queue_debug("0x%x, count=%d,head=%d,tail=%d\n\r", (uint32_t)q, q->count, q->head, q->tail);
	}
	return p;
}

void dequeue_item(Queue_t *q)
{
	if (!q) {
		return;
	}

	if (q->count == 0) {
		return;
	} else {
		osalEnterCritical();
		queue_debug("0x%x, count=%d,head=%d,tail=%d\n\r", (uint32_t)q, q->count, q->head, q->tail);
		q->head++;
		q->count--;
		if (q->head == q->max_items) {
			q->head = 0;
		}
		osalExitCritical();
	}
}




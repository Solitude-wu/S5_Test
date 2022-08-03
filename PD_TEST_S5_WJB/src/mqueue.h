#ifndef __MQUEUE_H__
#define __MQUEUE_H__

#include <stdint.h>
#include <string.h>

#define MAX_QUEUES	8

typedef struct {
	/* ����ͷ */
	uint16_t head;
	/* ����β */
	uint16_t tail;
	/* ��������ЧԪ�ظ��� */
	uint16_t count;
	/* ������󳤶� */
	uint16_t max_items;
	/* ����Ԫ�ش�С */
	uint16_t item_size;
	/* ������ţ���Ӧ�ó����ȡ�������ݲ�����ʱ��Ҫ������ǰ���ݣ����ⱻ�жϽ��ճ���д */
	uint16_t lock_index;

	uint16_t reserved[2];
	/* ָ�����Ԫ�������ָ�� */
	void* pitems;
}Queue_t;

#ifdef __cplusplus
extern "C" {
#endif

Queue_t *create_queue(void* pitems, uint16_t size_of_item, uint8_t max_items);
void release_queue(Queue_t *q);
int8_t enqueue_item(Queue_t *q, void *item);
void *get_queue_head(Queue_t *q);
void *get_queue_tail(Queue_t *q);
void dequeue_item(Queue_t *q);


#ifdef __cplusplus
}
#endif

#endif

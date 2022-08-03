#ifndef __MQUEUE_H__
#define __MQUEUE_H__

#include <stdint.h>
#include <string.h>

#define MAX_QUEUES	8

typedef struct {
	/* 队列头 */
	uint16_t head;
	/* 队列尾 */
	uint16_t tail;
	/* 队列中有效元素个数 */
	uint16_t count;
	/* 队列最大长度 */
	uint16_t max_items;
	/* 队列元素大小 */
	uint16_t item_size;
	/* 锁定标号，在应用程序读取队列数据并处理时需要锁定当前数据，避免被中断接收程序复写 */
	uint16_t lock_index;

	uint16_t reserved[2];
	/* 指向队列元素数组的指针 */
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

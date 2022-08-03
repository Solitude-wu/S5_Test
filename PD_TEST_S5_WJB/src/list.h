#ifndef __LIST_H_
#define __LIST_H_

#include "stdint.h"

typedef struct list{
	struct list *next;
}list_t;

#define BUFF_TO_LIST(a) ((list_t *)((uint8_t *)a - sizeof(list_t)))
#define LIST_TO_BUFF(a) ((uint8_t *)(a) + sizeof(list_t))

extern void list_insert_tail(list_t **head, list_t *node);
extern int list_remove(list_t **head, list_t *node);

#endif

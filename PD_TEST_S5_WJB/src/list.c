/*
 * list.c
 *
 *  Created on: 2020Äê10ÔÂ17ÈÕ
 *      Author: WIN
 */

#include <stdio.h>
#include "NUC230_240.h"
#include "list.h"
#include "common.h"

void list_insert_tail(list_t **head, list_t *node)
{
	list_t *p;
	if (node && head) {
		DisableInterrupt();
		if (*head == NULL) {
			*head = node;
		} else {
			p = *head;
			while(p->next) {
				p = p->next;
			}
			p->next = node;
		}
		node->next = NULL;
		EnableInterrupt();
	}
}

int list_remove(list_t **head, list_t *node)
{
	list_t *p, *q;
	int ret = -1;

	if (node && head) {
		DisableInterrupt();
		if (*head == node) {
			*head = (*head)->next;
			ret = 0;
		} else {
			p = *head;
			while(p!=node && p!=NULL) {
				q = p;
				p = p->next;
			}
			if (p != NULL) {
				q->next = p->next;
				ret = 0;
			} else {
				printf("node:%p is not in list:%p\n\r", node, *head);
			}
		}
		EnableInterrupt();
	}
	return ret;
}


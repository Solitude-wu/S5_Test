#include "common.h"
#include <stdint.h>

typedef uint32_t   m32_t;
typedef uint16_t   m16_t;
typedef uint8_t    m8_t; 

#ifndef NULL
#define NULL (void*)0
#endif

#define MCB_USED_MAIGC		0xDEADBEEF
  
/* �ڴ������ƿ����ݽṹ,�ڴ������ڹ������� */
struct mcb
{
    struct mcb  *mcb_next;	/* ָ����һ���ڴ���ƿ� */
	struct mcb  *mcb_prev;  /* ָ��ǰһ���ڴ���ƿ� */		
	m32_t        mcb_used;  /* �ڴ��Ƿ�ʹ��, 1=�ڴ�ռ��  0=�ڴ���� */
};


#define MCB_SIZE()     ((sizeof(struct mcb) + 3) & ~3u)
#define MCB_HEAD(mem)  (struct mcb*)((unsigned)(mem) - MCB_SIZE())
#define MIN_MCB_DATA_SIZE  16  /* �����ڴ��ʱ, ���ڴ����С�����ֽ��� */


/* ���ڴ��СΪ 16Kbytes */
static struct mcb  *mcb_head;        //�ڴ���ƿ�����ı�ͷ
static struct mcb  *mcb_tail;        //�ڴ���ƿ������β��ָ��
static uint32_t mcb_start, mcb_end;

#define MEM_VALID(mem) ((unsigned)(mem) >= (mcb_start + MCB_SIZE()) &&  \
	                    (unsigned)(mem) <= (mcb_end - MIN_MCB_DATA_SIZE) && \
	                    ((unsigned)(mem) & 3) == 0)
	
/*
**********************************************
* �������ܣ��ڴ���ƿ������ʼ��
**********************************************
*/
void      mcb_init(uint32_t addr_start, uint32_t addr_end)
{
    mcb_start = (addr_start + 64) & ~3u;
    mcb_end   = addr_end & ~3u;
	mcb_head         = (struct mcb*)mcb_start;
	mcb_tail         = (struct mcb*)mcb_end;
	mcb_head->mcb_next = mcb_tail;
	mcb_head->mcb_prev = mcb_head;
	mcb_head->mcb_used = 0;    
}

/*
**********************************************
* �������ܣ������ڴ���ƿ�
* ��    ����size ��Ҫ������ڴ��С
* �� �� ֵ������ɹ�����MCBָ��,����ʧ�ܷ��ؿ�ָ��
**********************************************
*/
void     *__malloc(unsigned int size)
{
	struct mcb *mcb_ptr, *new_mcb;
	int  free_size;
	
	/* ������ڴ�������(4�ֽ�)���� */
	if (size < 1)
        return NULL;  	
    if (size < MIN_MCB_DATA_SIZE)
		size = MIN_MCB_DATA_SIZE;
	size = (size + 3) & ~3u;
	size += MCB_SIZE();
	for (mcb_ptr = mcb_head; mcb_ptr != mcb_tail; mcb_ptr = mcb_ptr->mcb_next)
	{
	    /* Ѱ�Ҵ��ڵ���size���ڴ�� */
		if ((mcb_ptr->mcb_used == 0)&&
		    (( free_size = (unsigned)mcb_ptr->mcb_next - (unsigned)mcb_ptr) >= size))
		{
		    //�����ڴ���������㹻�Ŀռ��ٷ���һ���µ��ڴ��
			if ( (free_size - size) >= MCB_SIZE())
			{
			    new_mcb = (struct mcb*)((unsigned)mcb_ptr + size);  //�����µ�MCB
				new_mcb->mcb_next  = mcb_ptr->mcb_next;
				new_mcb->mcb_prev  = mcb_ptr;
				new_mcb->mcb_used  = 0;
				if (mcb_ptr->mcb_next != mcb_tail)
				{
				    mcb_ptr->mcb_next->mcb_prev = new_mcb;
				}
				mcb_ptr->mcb_next = new_mcb;
			}
			
			mcb_ptr->mcb_used = MCB_USED_MAIGC;
			return (void*)((unsigned)mcb_ptr + MCB_SIZE());    
		}
	}

	return NULL;
}


void     *malloc(unsigned int size)
{
	void *ret;
	DisableInterrupt();
	ret = __malloc(size);
	EnableInterrupt();
	return ret;
}

/*
**********************************************
* �������ܣ��ͷ��ڴ���ƿ�
* ��    ����mcb ��Ҫ�ͷŵ��ڴ���ƿ�ָ��
* ˵    �������ͷ��ڴ��ͬʱ��ǰ��Ŀ����ڴ�����һƬ
**********************************************
*/
void      __free(void *mem)
{
	struct mcb *start, *end, *free;

	if (MEM_VALID(mem) == 0)
		return;
	free = MCB_HEAD(mem);
	if (free->mcb_used != MCB_USED_MAIGC)
		return;

	/* Ѱ����һ�������ڴ�� */
    if (free->mcb_prev->mcb_used == 0)
	{
	    start =	free->mcb_prev;
	}
	else
	{
	    start = free;
	}

	/* Ѱ����һ�����������ڴ�� */
	if (free->mcb_next != mcb_tail && free->mcb_next->mcb_used == 0)
	{
	    end  = free->mcb_next;
	}
	else
	{
	    end  = free;
	}
    
    /* ���ڴ������һƬ */
	if (start != end)
	{
	    start->mcb_next = end->mcb_next;
	    end->mcb_prev = start;
	}
	start->mcb_used = 0;
}


void     free(void *mem)
{
	DisableInterrupt();
	__free(mem);
	EnableInterrupt();
}

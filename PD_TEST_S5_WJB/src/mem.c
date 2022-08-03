#include "common.h"
#include <stdint.h>

typedef uint32_t   m32_t;
typedef uint16_t   m16_t;
typedef uint8_t    m8_t; 

#ifndef NULL
#define NULL (void*)0
#endif

#define MCB_USED_MAIGC		0xDEADBEEF
  
/* 内存管理控制块数据结构,内存块紧接在管理快后面 */
struct mcb
{
    struct mcb  *mcb_next;	/* 指向下一个内存控制块 */
	struct mcb  *mcb_prev;  /* 指向前一个内存控制块 */		
	m32_t        mcb_used;  /* 内存是否使用, 1=内存占用  0=内存空闲 */
};


#define MCB_SIZE()     ((sizeof(struct mcb) + 3) & ~3u)
#define MCB_HEAD(mem)  (struct mcb*)((unsigned)(mem) - MCB_SIZE())
#define MIN_MCB_DATA_SIZE  16  /* 分裂内存块时, 新内存块最小数据字节数 */


/* 总内存大小为 16Kbytes */
static struct mcb  *mcb_head;        //内存控制块链表的表头
static struct mcb  *mcb_tail;        //内存控制块链表的尾部指针
static uint32_t mcb_start, mcb_end;

#define MEM_VALID(mem) ((unsigned)(mem) >= (mcb_start + MCB_SIZE()) &&  \
	                    (unsigned)(mem) <= (mcb_end - MIN_MCB_DATA_SIZE) && \
	                    ((unsigned)(mem) & 3) == 0)
	
/*
**********************************************
* 函数功能：内存控制块链表初始化
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
* 函数功能：分配内存控制块
* 参    数：size 需要分配的内存大小
* 返 回 值：分配成功返回MCB指针,分配失败返回空指针
**********************************************
*/
void     *__malloc(unsigned int size)
{
	struct mcb *mcb_ptr, *new_mcb;
	int  free_size;
	
	/* 分配的内存块必须字(4字节)对齐 */
	if (size < 1)
        return NULL;  	
    if (size < MIN_MCB_DATA_SIZE)
		size = MIN_MCB_DATA_SIZE;
	size = (size + 3) & ~3u;
	size += MCB_SIZE();
	for (mcb_ptr = mcb_head; mcb_ptr != mcb_tail; mcb_ptr = mcb_ptr->mcb_next)
	{
	    /* 寻找大于等于size的内存块 */
		if ((mcb_ptr->mcb_used == 0)&&
		    (( free_size = (unsigned)mcb_ptr->mcb_next - (unsigned)mcb_ptr) >= size))
		{
		    //空余内存块里面有足够的空间再分配一个新的内存块
			if ( (free_size - size) >= MCB_SIZE())
			{
			    new_mcb = (struct mcb*)((unsigned)mcb_ptr + size);  //分配新的MCB
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
* 函数功能：释放内存控制块
* 参    数：mcb 需要释放的内存控制块指针
* 说    明：在释放内存的同时将前后的空闲内存连成一片
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

	/* 寻找上一个空闲内存块 */
    if (free->mcb_prev->mcb_used == 0)
	{
	    start =	free->mcb_prev;
	}
	else
	{
	    start = free;
	}

	/* 寻找下一个连续空闲内存块 */
	if (free->mcb_next != mcb_tail && free->mcb_next->mcb_used == 0)
	{
	    end  = free->mcb_next;
	}
	else
	{
	    end  = free;
	}
    
    /* 将内存块连成一片 */
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

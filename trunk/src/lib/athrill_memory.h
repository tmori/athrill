#ifndef _ATHRLL_MEMORY_H_
#define _ATHRLL_MEMORY_H_

#include "std_types.h"
#include "assert.h"
#include <string.h>

#define ATHRILL_MEM_ENTRY_SIZE	(10240 * 1024 * 4)
typedef struct {
	uint32 free_start;
	uint32 freesize;
	void *next;
	uint8 data[ATHRILL_MEM_ENTRY_SIZE];
} AthrillMemEntryType;

typedef struct {
	AthrillMemEntryType *headp;
	AthrillMemEntryType *currp;
} AthrillMemHeadType;

extern AthrillMemHeadType athrill_mem_head;

static inline void athrill_mem_init(AthrillMemEntryType *entryp)
{
	entryp->next = NULL;
	entryp->free_start = 0U;
	entryp->freesize = ATHRILL_MEM_ENTRY_SIZE;
	return;
}

static inline void *athrill_mem_alloc(uint32 size)
{
	static int alloc_count = 0;
	ASSERT(size <= ATHRILL_MEM_ENTRY_SIZE);
	if (athrill_mem_head.headp == NULL) {
		athrill_mem_head.headp = malloc(sizeof(AthrillMemEntryType));
		alloc_count++;
		ASSERT(athrill_mem_head.headp != NULL);
		athrill_mem_head.currp = athrill_mem_head.headp;
		athrill_mem_init(athrill_mem_head.headp);
	}
	if (size > athrill_mem_head.currp->freesize) {
		AthrillMemEntryType *new_entryp = malloc(sizeof(AthrillMemEntryType));
		alloc_count++;
		ASSERT(new_entryp != NULL);
		athrill_mem_init(new_entryp);
		new_entryp->next = athrill_mem_head.currp;
		athrill_mem_head.currp = new_entryp;
	}
	void *retp =  &athrill_mem_head.currp->data[athrill_mem_head.currp->free_start];
	athrill_mem_head.currp->free_start += size;
	athrill_mem_head.currp->freesize -= size;
	return retp;
}
static inline void *athrill_mem_calloc(uint32 memsz, uint32 size)
{
	void *p = athrill_mem_alloc(memsz * size);
	memset(p, 0, size);
	return p;
}

#endif /* _ATHRLL_MEMORY_H_ */

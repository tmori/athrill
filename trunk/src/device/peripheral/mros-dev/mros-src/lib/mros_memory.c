#include "mros_memory.h"
#define ASSERT(expr)	\
do {	\
	if (!(expr))	{	\
		printf("ASSERTION FAILED:%s:%s:%d:%s\n", __FILE__, __FUNCTION__, __LINE__, #expr);	\
		exit(1);	\
	}	\
} while (0)


#define MEMORY_ID(index)		((index) + 1U)
#define MEMORY_INDEX(id)		((id) - 1U)

#define MEMORY_OBJ(mid, id)		memory_manager[(mid)].memory_entries[MEMORY_INDEX((id))]

mRosReturnType mros_mem_init(mRosSizeType config_num, mRosMemoryConfigType **config, mRosMemoryManagerType *mgrp)
{
	return MROS_E_OK;
}

mRosReturnType mros_mem_alloc(mRosMemoryManagerType *mgrp, mRosSizeType size, mRosMemoryListEntryType **memory)
{
	mRosMemoryListEntryType *mp;
	char *ptr;
	mp = malloc(sizeof(mRosMemoryListEntryType));
	ASSERT(mp != MROS_NULL);
	ptr = malloc(size);
	ASSERT(ptr != MROS_NULL);
	mp->data.memp = ptr;
	mp->data.memsize = size;
	mp->data.size = size;
	*memory = mp;
	return MROS_E_OK;
}

mRosReturnType mros_mem_free(mRosMemoryManagerType *mgrp, mRosMemoryListEntryType *memory)
{
	if (memory != NULL) {
		if (memory->data.memp != MROS_NULL) {
			free(memory->data.memp);
		}
		free(memory);
	}
	return MROS_E_OK;
}


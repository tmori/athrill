#ifndef _CPU_H_
#define _CPU_H_

#include "std_types.h"
#include "cpu_config.h"
#include "target_cpu.h"
#include "assert.h"

typedef struct {
	/*
	 * core must be top.
	 * do not add member on this region. 
	 */
	TargetCoreType		core;
	uint64				elaps;
} CpuCoreType;

typedef struct {
	OpDecodedCodeType	decoded_code;
#ifndef ARCH_V850ES_FK3
	OpCodeId			code_id;
#endif /* ARCH_V850ES_FK3 */
	int (*op_exec) (TargetCoreType *cpu);
	/*
	 * not supported yet..
	 * use this flag if cache region is rewrite by user program.
	 * then cache reconstruction is needed!!
	 */
	bool				is_dirty;
} CpuOperationCodeType;

typedef struct {
	uint32				code_start_addr;
	uint32				code_size;
	CpuOperationCodeType	*codes;
} CachedOperationCodeType;

#define DEFAULT_CPU_FREQ		100 /* MHz */
typedef struct {
	uint32						cpu_freq; /* Mhz */
	CpuCoreType					*current_core;
	uint32						core_id_num;
	CpuCoreType					cores[CPU_CONFIG_CORE_NUM];
	uint32						cached_code_num;
	CachedOperationCodeType		**cached_code;
} CpuType;

extern CpuType	virtual_cpu;
#define CPU_CONFIG_GET_CORE_ID_NUM()	((int)virtual_cpu.core_id_num)

static inline CachedOperationCodeType *virtual_cpu_get_cached_code(uint32 pc)
{
	uint32 i;
	static CachedOperationCodeType *last = NULL;

	if (last != NULL) {
		if ((pc >= last->code_start_addr) && (pc < (last->code_start_addr + last->code_size))) {
			return last;
		}
	}

	for (i = 0; i < virtual_cpu.cached_code_num; i++) {
		if (pc < virtual_cpu.cached_code[i]->code_start_addr) {
			continue;
		}
		else if (pc >= (virtual_cpu.cached_code[i]->code_start_addr + virtual_cpu.cached_code[i]->code_size)) {
			continue;
		}
		last = virtual_cpu.cached_code[i];
		return virtual_cpu.cached_code[i];
	}
	/*
	 * not reached.
	 */
	return NULL;
}

static inline void virtual_cpu_add_cached_code(CachedOperationCodeType *cached_code)
{
	virtual_cpu.cached_code_num++;
	virtual_cpu.cached_code = realloc(virtual_cpu.cached_code, virtual_cpu.cached_code_num * (sizeof (CachedOperationCodeType*)));
	ASSERT(virtual_cpu.cached_code != NULL);
	virtual_cpu.cached_code[virtual_cpu.cached_code_num - 1] = cached_code;
	return;
}
/*
 * start_addr: 	unit=byte
 * memsz: 		unit=byte
 */
static inline void virtual_cpu_cache_code_add_with_check(uint32 memsz, uint32 start_addr)
{
	CachedOperationCodeType *cached_code = virtual_cpu_get_cached_code(start_addr);
	if (cached_code != NULL) {
		return;
	}

	cached_code = malloc(sizeof(CachedOperationCodeType));
	ASSERT(cached_code != NULL);
	cached_code->codes = calloc(memsz, sizeof(CpuOperationCodeType));
	ASSERT(cached_code->codes != NULL);
	cached_code->code_start_addr = start_addr;
	cached_code->code_size = (memsz);

	virtual_cpu_add_cached_code(cached_code);
	return;
}


extern uint32 cpu_get_current_core_id(void);
extern uint32 cpu_get_ep(const TargetCoreType *core);

extern uint32 cpu_get_current_core_register(uint32 inx);
extern uint32 cpu_get_current_core_pc(void);
extern void cpu_set_core_pc(CoreIdType core_id, uint32 pc);
extern uint32 cpu_get_current_core_sp(void);
extern uint32 cpu_get_current_core_ep(void);
extern uint32 cpu_get_return_addr(const TargetCoreType *core);

extern CoreIdType cpu_get_core_id(const TargetCoreType *core);
extern void intc_cpu_trigger_interrupt(CoreIdType core_id, int intno);

extern bool cpu_may_store_on_stack_overflow(uint32 start_addr, uint32 size);

#endif /* _CPU_H_ */

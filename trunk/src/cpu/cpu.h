#ifndef _CPU_H_
#define _CPU_H_

#include "std_types.h"
#include "cpu_config.h"
#include "target_cpu.h"
#include "assert.h"

typedef struct {
	TargetCoreType		core;
} CpuCoreType;

typedef struct {
	OpDecodedCodeType	decoded_code;
	int (*op_exec) (TargetCoreType *cpu);
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

static inline CachedOperationCodeType *virtual_cpu_get_cached_code(uint32 pc)
{
	uint32 i;
	for (i = 0; i < virtual_cpu.cached_code_num; i++) {
		if (pc < virtual_cpu.cached_code[i]->code_start_addr) {
			continue;
		}
		else if (pc >= (virtual_cpu.cached_code[i]->code_start_addr + virtual_cpu.cached_code[i]->code_size)) {
			continue;
		}
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


extern uint32 cpu_get_current_core_id(void);
extern uint32 cpu_get_pc(const TargetCoreType *core);
extern uint32 cpu_get_sp(const TargetCoreType *core);
extern uint32 cpu_get_ep(const TargetCoreType *core);

extern uint32 cpu_get_current_core_register(uint32 inx);
extern uint32 cpu_get_current_core_pc(void);
extern uint32 cpu_get_current_core_sp(void);
extern uint32 cpu_get_current_core_ep(void);
extern uint32 cpu_get_return_addr(const TargetCoreType *core);

extern CoreIdType cpu_get_core_id(const TargetCoreType *core);
extern void intc_cpu_trigger_interrupt(CoreIdType core_id, int intno);

extern bool cpu_may_store_on_stack_overflow(uint32 start_addr, uint32 size);

#endif /* _CPU_H_ */

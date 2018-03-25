#ifndef _CPU_H_
#define _CPU_H_

#include "std_types.h"
#include "cpu_config.h"
#include "target_cpu.h"

typedef struct {
	TargetCoreType		core;
} CpuCoreType;

typedef struct {
	CpuCoreType			*current_core;
	CpuCoreType			cores[CPU_CONFIG_CORE_NUM];
} CpuType;

extern CpuType	virtual_cpu;

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

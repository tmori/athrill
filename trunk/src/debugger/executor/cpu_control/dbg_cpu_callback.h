#ifndef _DBG_CPU_CALLBACK_H_
#define _DBG_CPU_CALLBACK_H_

#include "std_types.h"
#include "cpu_config.h"
#include "target_cpu.h"
#include "dbg_log.h"

typedef struct {
	uint32 enable_ft;
	uint32 enable_bt;
	uint32 enable_prof;
	uint32 enable_watch;
	/*
	 * enable sync real time and virtual time.
	 */
	uint32 enable_sync_time;
	uint32 show_skip_time;
	uint32 reset_pc;
} DbgCpuCallbackFuncEnableType;

extern void dbg_notify_cpu_clock_supply_start(const TargetCoreType *core);
extern void dbg_notify_cpu_clock_supply_end(const TargetCoreType *core, const DbgCpuCallbackFuncEnableType *enable_dbg);

extern bool cpuemu_is_cui_mode;

static inline bool cpuemu_cui_mode(void)
{
	return cpuemu_is_cui_mode;
}
#endif /* _DBG_CPU_CALLBACK_H_ */

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
} DbgCpuCallbackFuncEnableType;

extern void dbg_notify_cpu_clock_supply_start(const TargetCoreType *core);
extern void dbg_notify_cpu_clock_supply_end(const TargetCoreType *core, const DbgCpuCallbackFuncEnableType *enable_dbg);


#endif /* _DBG_CPU_CALLBACK_H_ */

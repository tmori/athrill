#ifndef _DBG_TARGET_CPU_H_
#define _DBG_TARGET_CPU_H_

#include "std_types.h"

extern void dbg_target_print_cpu(uint32 core_id);
extern void cpu_debug_print_mpu_status(uint32 core_id);

#endif /* _DBG_TARGET_CPU_H_ */

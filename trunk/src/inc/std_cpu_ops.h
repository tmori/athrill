#ifndef _STD_CPU_OPS_H_
#define _STD_CPU_OPS_H_

#include "std_types.h"
#include "std_errno.h"

extern void cpu_init(void);
extern void cpu_reset(CoreIdType core_id);
extern void cpu_illegal_opcode_trap(CoreIdType core_id);
extern Std_ReturnType cpu_supply_clock(CoreIdType core_id);

#endif /* _STD_CPU_OPS_H_ */

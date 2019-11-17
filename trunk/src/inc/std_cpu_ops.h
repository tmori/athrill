#ifndef _STD_CPU_OPS_H_
#define _STD_CPU_OPS_H_

#include "std_types.h"
#include "std_errno.h"
#include "cpu_config_ops.h"

extern void cpu_init(void);
extern void cpu_reset(CoreIdType core_id);
extern void cpu_illegal_opcode_trap(CoreIdType core_id);
extern void cpu_set_current_core(CoreIdType core_id);
extern Std_ReturnType cpu_supply_clock(CoreIdType core_id);
extern bool cpu_is_halt(CoreIdType core_id);
extern bool cpu_is_halt_all(void);
extern void cpu_mpu_construct_containers(CoreIdType core_id);
extern bool cpu_illegal_access(CoreIdType core_id);

#endif /* _STD_CPU_OPS_H_ */

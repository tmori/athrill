#ifndef _MPU_MALLOC_H_
#define _MPU_MALLOC_H_

#include "std_types.h"
#include "std_errno.h"
#include "mpu_types.h"
#include "mpu_types.h"

extern void  mpu_malloc_add_region(MpuAddressRegionType *region);
extern uint32 mpu_malloc_get_memory(uint32 size);
extern void  mpu_malloc_rel_memory(uint32 addr);
extern uint32 mpu_malloc_ref_size(uint32 addr);


#endif /* _MPU_MALLOC_H_ */
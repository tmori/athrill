#ifndef _MPU_OPS_H_
#define _MPU_OPS_H_

#include "std_types.h"

extern Std_ReturnType mpu_get_data8(CoreIdType core_id, uint32 addr, uint8 *data);
extern Std_ReturnType mpu_get_data16(CoreIdType core_id, uint32 addr, uint16 *data);
extern Std_ReturnType mpu_get_data32(CoreIdType core_id, uint32 addr, uint32 *data);

extern Std_ReturnType mpu_put_data8(CoreIdType core_id, uint32 addr, uint8 data);
extern Std_ReturnType mpu_put_data16(CoreIdType core_id, uint32 addr, uint16 data);
extern Std_ReturnType mpu_put_data32(CoreIdType core_id, uint32 addr, uint32 data);

extern Std_ReturnType mpu_get_pointer(CoreIdType core_id, uint32 addr, uint8 **data);


/*
 * the following enum values must be equal MemoryAddressImplType(cpuemu_ops.h).
 */
typedef enum {
    MpuAddressGetType_ROM = 0,
    MpuAddressGetType_RAM,
    MpuAddressGetType_MMAP,
    MpuAddressGetType_MALLOC,
} MpuAddressGetType;
#define MPU_MALLOC_REGION_UNIT_SIZE         1024    /* KB */
#define MPU_MALLOC_REGION_UNIT_GROUP_NUM    10      /* 10MB */

extern uint8 *mpu_address_set_rom_ram(MpuAddressGetType getType, uint32 addr, uint32 size, void *mmap_addr);
extern uint8 *mpu_address_get_rom(uint32 addr, uint32 size);
extern void mpu_address_set_malloc_region(uint32 addr, uint32 size);

#endif /* _MPU_OPS_H_ */

#ifndef _MPU_H_
#define _MPU_H_

#include "std_types.h"
#include "std_errno.h"
#include "mpu_types.h"
#include "mpu_config.h"

typedef struct {
	MpuAddressRegionType	map[MPU_CONFIG_REGION_NUM];
	uint32					dynamic_map_num;
	MpuAddressRegionType	*dynamic_map;
} MpuAddressMapType;

extern MpuAddressMapType	mpu_address_map;

extern void mpu_add_dynamic_map(MpuAddressRegionType *p);
extern MpuAddressRegionType *mpu_search_dynamic_map(uint32 start, uint32 size);

#endif /* _MPU_H_ */

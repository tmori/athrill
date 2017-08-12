#ifndef _MPU_H_
#define _MPU_H_

#include "std_types.h"
#include "std_errno.h"
#include "mpu_types.h"
#include "mpu_config.h"

typedef struct {
	MpuAddressRegionType	map[MPU_CONFIG_REGION_NUM];
} MpuAddressMapType;

extern MpuAddressMapType	mpu_address_map;

#endif /* _MPU_H_ */

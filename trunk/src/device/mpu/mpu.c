#include "mpu.h"
#include "cpu_config.h"
#include "mpu_ops.h"
#include <stdio.h>

static Std_ReturnType memory_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data);
static Std_ReturnType memory_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data);
static Std_ReturnType memory_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data);
static Std_ReturnType memory_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data);
static Std_ReturnType memory_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data);
static Std_ReturnType memory_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data);
static Std_ReturnType memory_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data);

MpuAddressRegionOperationType	default_memory_operation = {
		.get_data8 		= 	memory_get_data8,
		.get_data16		=	memory_get_data16,
		.get_data32		=	memory_get_data32,

		.put_data8 		= 	memory_put_data8,
		.put_data16		=	memory_put_data16,
		.put_data32		=	memory_put_data32,

		.get_pointer	= memory_get_pointer
};

static inline bool has_permission(MpuAddressRegionType *region, CoreIdType core_id)
{
	if (	(region->permission != MPU_ADDRESS_REGION_PERM_ALL) &&
			((region->permission & (1U << core_id)) == FALSE) ) {
		return FALSE;
	}
	return TRUE;
}

static inline MpuAddressRegionType *search_region(CoreIdType core_id, uint32 addr, uint32 search_size)
{
	uint32 i;

	//printf("addr=0x%x\n", addr);

	for (i = 0U; i < MPU_CONFIG_REGION_NUM; i++) {
		uint32 start = mpu_address_map.map[i].start;
		uint32 end = mpu_address_map.map[i].start  + mpu_address_map.map[i].size;
		uint32 paddr_str = (addr & mpu_address_map.map[i].mask);
		uint32 paddr_end = paddr_str + search_size;

#if 0
		printf("%u:start=0x%x end=0x%x mask=0x%x\n", i, start, end, mpu_address_map.map[i].mask);
		printf("%u:pstart=0x%x pend=0x%x\n", i, paddr_str, paddr_end);
#endif
		if (	((start <= paddr_str) && (paddr_str < end)) &&
				((start <  paddr_end) && (paddr_end <= end))
			) {
			//printf("1:passed1\n");
			if (has_permission( &mpu_address_map.map[i], core_id) == FALSE) {
				printf("search_region:permission error:addr=0x%x\n", addr);
				return NULL;
			}
			//printf("2:passed1:%u:0x%p\n", i,  &mpu_address_map.map[i]);
			return &mpu_address_map.map[i];
		}
	}
	printf("search_region:not found error:addr=0x%x\n", addr);
	return NULL;
}

Std_ReturnType mpu_get_data8(CoreIdType core_id, uint32 addr, uint8 *data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 1U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->get_data8 == NULL) {
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->get_data8(region, core_id, paddr, data);
}
Std_ReturnType mpu_get_data16(CoreIdType core_id, uint32 addr, uint16 *data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 2U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->get_data16 == NULL) {
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->get_data16(region, core_id, paddr, data);
}

Std_ReturnType mpu_get_data32(CoreIdType core_id, uint32 addr, uint32 *data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 4U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->get_data32 == NULL) {
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->get_data32(region, core_id, paddr, data);
}


Std_ReturnType mpu_put_data8(CoreIdType core_id, uint32 addr, uint8 data)
{
	Std_ReturnType err;
	MpuAddressRegionType *region = search_region(core_id, addr, 1U);
	if (region == NULL) {
		printf("mpu_put_data8:error1:addr=0x%x data=%u\n", addr, data);
		return STD_E_SEGV;
	}
	if (region->ops->put_data8 == NULL) {
		printf("mpu_put_data8:error2:addr=0x%x data=%u\n", addr, data);
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	err = region->ops->put_data8(region, core_id, paddr, data);
	if (err != STD_E_OK) {
		printf("mpu_put_data8:error3:addr=0x%x data=%u\n", addr, data);
	}
	return err;
}

Std_ReturnType mpu_put_data16(CoreIdType core_id, uint32 addr, uint16 data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 2U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->put_data16 == NULL) {
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->put_data16(region, core_id, paddr, data);
}

Std_ReturnType mpu_put_data32(CoreIdType core_id, uint32 addr, uint32 data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 4U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->put_data32 == NULL) {
		return STD_E_SEGV;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->put_data32(region, core_id, paddr, data);
}


Std_ReturnType mpu_get_pointer(CoreIdType core_id, uint32 addr, uint8 **data)
{
	MpuAddressRegionType *region = search_region(core_id, addr, 1U);
	if (region == NULL) {
		return STD_E_SEGV;
	}
	if (region->ops->get_pointer == NULL) {
		return STD_E_INVALID;
	}
	uint32 paddr = (addr & region->mask);
	return region->ops->get_pointer(region, core_id, paddr, data);
}





static Std_ReturnType memory_get_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint8*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType memory_get_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint16*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType memory_get_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 *data)
{
	uint32 off = (addr - region->start);
	*data = *((uint32*)(&region->data[off]));
	return STD_E_OK;
}
static Std_ReturnType memory_put_data8(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 data)
{
	uint32 off = (addr - region->start);
	*((uint8*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType memory_put_data16(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint16 data)
{
	uint32 off = (addr - region->start);
	*((uint16*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType memory_put_data32(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint32 data)
{
	uint32 off = (addr - region->start);
	*((uint32*)(&region->data[off])) = data;
	return STD_E_OK;
}
static Std_ReturnType memory_get_pointer(MpuAddressRegionType *region, CoreIdType core_id, uint32 addr, uint8 **data)
{
	uint32 off = (addr - region->start);
	*data = ((uint8*)(&region->data[off]));
	return STD_E_OK;
}

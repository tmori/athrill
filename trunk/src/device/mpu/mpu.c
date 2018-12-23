#include "cpu_config.h"
#include "mpu.h"
#include "mpu_ops.h"
#include <stdio.h>
#include <stdlib.h>
#include "assert.h"
#include "mpu_malloc.h"

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

#if 0
static inline bool has_permission(MpuAddressRegionType *region, CoreIdType core_id)
{
	if (	(region->permission != MPU_ADDRESS_REGION_PERM_ALL) &&
			((region->permission & (1U << core_id)) == FALSE) ) {
		return FALSE;
	}
	return TRUE;
}
#endif

static inline MpuAddressRegionType *search_region(CoreIdType core_id, uint32 addr, uint32 search_size)
{
	uint32 i;

	//printf("addr=0x%x\n", addr);

	for (i = 0U; i < mpu_address_map.dynamic_map_num; i++) {
		uint32 start = mpu_address_map.dynamic_map[i].start;
		uint32 end = mpu_address_map.dynamic_map[i].start  + mpu_address_map.dynamic_map[i].size;
		uint32 paddr_str = (addr & mpu_address_map.dynamic_map[i].mask);
		uint32 paddr_end = paddr_str + search_size;

		if (	((start <= paddr_str) && (paddr_str < end)) &&
				((start <  paddr_end) && (paddr_end <= end))
			) {

#if 0
			//printf("1:passed1\n");
			if (has_permission( &mpu_address_map.dynamic_map[i], core_id) == FALSE) {
				printf("search_region:permission error:addr=0x%x\n", addr);
				return NULL;
			}
#endif
			if (mpu_address_map.dynamic_map[i].data == NULL) {
				//MALLOC not malloced region
				return NULL;
			}
			//printf("2:passed1:%u:0x%p\n", i,  &mpu_address_map.map[i]);
			return &mpu_address_map.dynamic_map[i];
		}
	}


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
#if 0
			//printf("1:passed1\n");
			if (has_permission( &mpu_address_map.map[i], core_id) == FALSE) {
				printf("search_region:permission error:addr=0x%x\n", addr);
				return NULL;
			}
#endif
			//printf("2:passed1:%u:0x%p\n", i,  &mpu_address_map.map[i]);
			return &mpu_address_map.map[i];
		}
	}
	printf("search_region:not found error:addr=0x%x\n", addr);
	return NULL;
}

static MpuAddressRegionType *mpu_address_search_region(uint32 addr, uint32 size)
{
	uint32 i;
	MpuAddressRegionType *region = NULL;

	for (i = 0U; i < mpu_address_map.dynamic_map_num; i++) {
		uint32 start = mpu_address_map.dynamic_map[i].start;
		uint32 end = mpu_address_map.dynamic_map[i].start  + mpu_address_map.dynamic_map[i].size;
		uint32 paddr_str = (addr & mpu_address_map.dynamic_map[i].mask);
		uint32 paddr_end = paddr_str + size;

		if (	((start <= paddr_str) && (paddr_str < end)) &&
				((start <  paddr_end) && (paddr_end <= end))
			) {
			region = &mpu_address_map.dynamic_map[i];
			break;
		}
	}

	return region;
}

uint8 *mpu_address_get_rom(uint32 addr, uint32 size)
{
	MpuAddressRegionType *region = NULL;

	region = mpu_address_search_region(addr, size);
	if ((region != NULL) && (region->type == READONLY_MEMORY)) {
		return region->data;
	}
	return NULL;
}

uint8 *mpu_address_set_rom_ram(MpuAddressGetType getType, uint32 addr, uint32 size, void *mmap_addr)
{
	MpuAddressRegionType *region = NULL;

	region = mpu_address_search_region(addr, size);

	if (region == NULL) {
		mpu_address_map.dynamic_map_num++;
		mpu_address_map.dynamic_map = realloc(mpu_address_map.dynamic_map, (sizeof(MpuAddressRegionType)) * mpu_address_map.dynamic_map_num);
		ASSERT(mpu_address_map.dynamic_map != NULL);
		mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].start = addr;
		mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].is_malloc = FALSE;

		mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].permission	= MPU_ADDRESS_REGION_PERM_ALL;
		mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].mask		= MPU_ADDRESS_REGION_MASK_ALL;
		mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].ops			= &default_memory_operation;

		if (getType == MpuAddressGetType_ROM) {
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].type = READONLY_MEMORY;
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].size = size;
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data = malloc(size);
		}
		else if (getType == MpuAddressGetType_RAM) {
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].type = GLOBAL_MEMORY;
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].size = size;
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data = calloc(1U, size);
		}
		else {
#ifdef OS_LINUX
			mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].size = size;
			if (getType == MpuAddressGetType_MMAP) {
				/* MMAP */
				mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].type = GLOBAL_MEMORY;
				mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data = mmap_addr;
			}
			else if (getType == MpuAddressGetType_MALLOC) {
				/* MALLOC */
				mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].type = GLOBAL_MEMORY;
				mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].is_malloc = TRUE;
				mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data = NULL;
			}
#endif /* OS_LINUX */
		}
		if (getType == MpuAddressGetType_MALLOC) {
			ASSERT(mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data == NULL);
		}
		else {
			ASSERT(mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data != NULL);
		}
		return mpu_address_map.dynamic_map[mpu_address_map.dynamic_map_num -1].data;
	}
	else {
		return region->data;
	}
}
void mpu_address_set_malloc_region(uint32 addr, uint32 size)
{
	MpuAddressRegionType *region = NULL;

	region = mpu_address_search_region(addr, size);

	mpu_malloc_add_region(region);
	return;
}

MpuAddressRegionEnumType mpu_address_region_type_get(uint32 addr, bool *is_malloc)
{
	uint32 i;

	for (i = 0U; i < mpu_address_map.dynamic_map_num; i++) {
		uint32 start = mpu_address_map.dynamic_map[i].start;
		uint32 end = mpu_address_map.dynamic_map[i].start  + mpu_address_map.dynamic_map[i].size;
		uint32 paddr_str = (addr & mpu_address_map.dynamic_map[i].mask);

		if ((start <= paddr_str) && (paddr_str < end)) {
			if (is_malloc != NULL) {
				*is_malloc = mpu_address_map.dynamic_map[i].is_malloc;
			}
			return mpu_address_map.dynamic_map[i].type;
		}
	}

	for (i = 0U; i < MPU_CONFIG_REGION_NUM; i++) {
		uint32 start = mpu_address_map.map[i].start;
		uint32 end = mpu_address_map.map[i].start  + mpu_address_map.map[i].size;
		uint32 paddr_str = (addr & mpu_address_map.map[i].mask);

		if ((start <= paddr_str) && (paddr_str < end)) {
			if (is_malloc != NULL) {
				*is_malloc = FALSE;
			}
			return mpu_address_map.map[i].type;
		}
	}
	printf("search_region:not found error:addr=0x%x\n", addr);
	return REGION_UNKNOWN;
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
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_READ, addr, 1U)) {
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
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_READ, addr, 2U)) {
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
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_READ, addr, 4U)) {
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
	if (region->type == READONLY_MEMORY) {
		printf("mpu_put_data8:error: can not write data on ROM :addr=0x%x data=%u\n", addr, data);
		return STD_E_SEGV;
	}
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_WRITE, addr, 1U)) {
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
	if (region->type == READONLY_MEMORY) {
		printf("mpu_put_data16:error: can not write data on ROM :addr=0x%x data=%u\n", addr, data);
		return STD_E_SEGV;
	}
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_WRITE, addr, 2U)) {
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
	if (region->type == READONLY_MEMORY) {
		printf("mpu_put_data32:error: can not write data on ROM :addr=0x%x data=%u\n", addr, data);
		return STD_E_SEGV;
	}
	if (!CPU_HAS_PERMISSION(core_id, region->type, CpuMemoryAccess_WRITE, addr, 4U)) {
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

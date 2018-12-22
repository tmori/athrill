#include "mpu_malloc.h"
#include "mpu_ops.h"
#include "assert.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

typedef enum {
    MallocSize_32 = 0,
    MallocSize_64,
    MallocSize_128,
    MallocSize_256,
    MallocSize_512,
    MallocSize_1024,
    MallocSize_2048,
    MallocSize_4096,
    MallocSize_8192,
    MallocSize_16384,
    MallocSize_Num,
} MallocSIzeType;

typedef struct {
    uint32	memsize;
    uint32	bitmaxnum;
    uint32	bitmapsize;
} MallocDataInfoType;

#define SIZE_TO_BITMAXNUM(size) ( ( MPU_MALLOC_REGION_UNIT_SIZE * 1024 ) / (size) )
#define SIZE_TO_BITMAPSIZE(size) ( (SIZE_TO_BITMAXNUM(32) + 63) / 64 )
static const MallocDataInfoType malloc_data_info_table[MallocSize_Num] = {
    { 32,    SIZE_TO_BITMAXNUM(32),    SIZE_TO_BITMAPSIZE(32) },
    { 64,    SIZE_TO_BITMAXNUM(64),    SIZE_TO_BITMAPSIZE(64) },
    { 128,   SIZE_TO_BITMAXNUM(128),   SIZE_TO_BITMAPSIZE(128) },
    { 256,   SIZE_TO_BITMAXNUM(256),   SIZE_TO_BITMAPSIZE(256) },
    { 512,   SIZE_TO_BITMAXNUM(512),   SIZE_TO_BITMAPSIZE(512) },
    { 1024,  SIZE_TO_BITMAXNUM(1024),  SIZE_TO_BITMAPSIZE(1024) },
    { 2048,  SIZE_TO_BITMAXNUM(2048),  SIZE_TO_BITMAPSIZE(2048) },
    { 4096,  SIZE_TO_BITMAXNUM(4096),  SIZE_TO_BITMAPSIZE(4096) },
    { 8192,  SIZE_TO_BITMAXNUM(8192),  SIZE_TO_BITMAPSIZE(8192) },
    { 16384, SIZE_TO_BITMAXNUM(16384), SIZE_TO_BITMAPSIZE(16384) },
};

#define BIT_IS_FREE8(bits8, index)  ( ( (1 << (index)) & bits8 ) == 0U )
#define BIT_SET(bits8, index) ( ( 1 << (index) ) | (bits8) )
#define BIT_CLR(bits8, index) ( (~( 1 << (index) )) & (bits8) )

static int set_bit(char *bitmap, uint32 bitmaxnum, uint32 bitmapsize)
{
    uint32 bit64_off = 0;
    uint64 *p;
    uint64 *e = (uint64*)((uint64)bitmap + (uint64)bitmapsize);
    uint32 ret;

    for (p = (uint64*)bitmap; p < e; p++, bit64_off++) {
        if ((!(*p)) == 0ULL) {
            continue;
        }
        break;
    }
    if (p == e) {
        return -1;
    }

    int bit8_off;
    int bit8_mod;
    uint8* bitmap8 = (uint8*)p;

    for (bit8_off = 0; bit8_off < 8; bit8_off++) {
        for (bit8_mod = 0; bit8_mod < 8; bit8_mod++) {
            if (BIT_IS_FREE8(bit8_mod, bitmap8[bit8_off])) {
                break;
            }
        }
    }

    ret = ( (bit64_off * 64) + (bit8_off * 8) + bit8_mod );
    if (ret >= bitmaxnum) {
        return -1;
    }
    bitmap8[bit8_off] = BIT_SET(bitmap8[bit8_off], bit8_mod);
    return ret;
}

typedef struct {
    MpuAddressRegionType *region;
    char*	bitmap;
    uint32	bitfreenum;
} MallocRegionUnitType;

typedef struct {
    uint32 unit_num;
    MallocRegionUnitType unit[MallocSize_Num];
} MallocRegionGroupType;

typedef struct {
    uint32 group_num;
    MallocRegionGroupType **groups;
} MallocRegionType;

static MallocRegionType malloc_region = {
    0,
    NULL,
};


uint32 mpu_malloc_get_memory(uint32 size)
{
    int i;
    int j;
    int candidate_index = -1;
    MallocRegionUnitType *unit = NULL;
    for (i = 0; i < MallocSize_Num; i++) {
        if (malloc_data_info_table[i].memsize >= size) {
            candidate_index = i;
            break;
        }
    }
    if (candidate_index < 0) {
        return 0;
    }

    for (i = candidate_index; i < MallocSize_Num; i++) {
        for (j = 0; j < malloc_region.group_num; j++) {
            if (malloc_region.groups[j]->unit[i].bitfreenum > 0) {
                unit = &malloc_region.groups[j]->unit[i];
                break;
            }
        }
    }
    if (unit == NULL) {
        return 0;
    }
    int index = set_bit(unit->bitmap, malloc_data_info_table[i].bitmaxnum, malloc_data_info_table[i].bitmapsize);
    if (index < 0) {
        return 0;
    }
    unit->bitfreenum--;
    return ( unit->region->start + (index * malloc_data_info_table[i].memsize));
}

void mpu_malloc_rel_memory(uint32 addr)
{
    int i;
    int j;
    MallocRegionUnitType *unit = NULL;
    for (i = 0; i < MallocSize_Num; i++) {
        for (j = 0; j < malloc_region.group_num; j++) {
            unit = &malloc_region.groups[j]->unit[i];
            if ((unit->region->start <= addr) &&
                                        (addr < (unit->region->start + unit->region->size))) {
                break;
            }
        }
    }
    if (unit == NULL) {
        return;
    }

    uint32 bits = ( (addr - unit->region->start) / malloc_data_info_table[i].memsize );
    uint32 bit64_off = bits / 64;
    uint32 bit64_mod = bits % 64;
    uint32 bit8_off  = bit64_mod / 8;
    uint32 bit8_mod  = bit64_mod % 8;

    uint64* bitmap64 = &(((uint64*)unit->bitmap)[bit64_off]);
    uint8* bimtap8 = &(((uint8*)bitmap64)[bit8_off]);

    *bimtap8 = BIT_CLR(*bimtap8, bit8_mod);
    unit->bitfreenum++;
    return;
}

static void group_init(MallocRegionGroupType* group)
{
    group->unit_num = 0;
    memset(group->unit, 0, sizeof(MallocRegionUnitType) * MallocSize_Num);
    int i;
    for (i = 0; i < MallocSize_Num; i++) {
        group->unit[i].bitfreenum = malloc_data_info_table[i].bitmaxnum;
    }
    return;
}

static void group_add_region(MallocRegionGroupType* group, MpuAddressRegionType *region)
{
    int index = group->unit_num;

    group->unit[index].region = region;
    group->unit[index].bitmap = malloc(malloc_data_info_table[index].bitmapsize);
    ASSERT(group->unit[index].bitmap != NULL);

    group->unit_num++;
    return;
}

static bool group_has_free_unit(MallocRegionGroupType *group)
{
    int i;
    for (i = 0; i < group->unit_num; i++) {
        if (group->unit[i].region == NULL) {
            return TRUE;
        }
    }
    return FALSE;
}

static MallocRegionGroupType* group_alloc(void)
{
    int index = malloc_region.group_num;

    malloc_region.group_num++;
    malloc_region.groups = (MallocRegionGroupType**)realloc(malloc_region.groups, 
                                malloc_region.group_num * sizeof(MallocRegionGroupType*));
    ASSERT(malloc_region.groups != NULL);

    malloc_region.groups[index] = (MallocRegionGroupType*)malloc(sizeof(MallocRegionGroupType));
    ASSERT(malloc_region.groups[index] != NULL);

    group_init(malloc_region.groups[index]);
    return malloc_region.groups[index];
}

static MallocRegionGroupType* group_get(void)
{
    int index;
    if (malloc_region.groups == NULL) {
        return group_alloc();
    }
    index = malloc_region.group_num - 1;
    MallocRegionGroupType *group = malloc_region.groups[index];
    if (group_has_free_unit(group) == TRUE) {
        return group;
    }
    return group_alloc();
}

void  mpu_malloc_add_region(MpuAddressRegionType *region)
{
    MallocRegionGroupType* group = group_get();
    group_add_region(group, region);
    return;
}
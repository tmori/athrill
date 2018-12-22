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
    uint32	memsize; /* byte */
    uint32	bitmaxnum;
    uint32	bitmapsize; /* byte */
} MallocDataInfoType;

#define SIZE_TO_BITMAXNUM(size) ( ( MPU_MALLOC_REGION_UNIT_SIZE * 1024 ) / (size) )
#define SIZE_TO_BITMAPSIZE(size) ( ( (SIZE_TO_BITMAXNUM(size) + 63) / 64 ) * 8 )
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

#define BIT_IS_FREE8(bits8, index)  ( ( ((uint8)1 << (index)) & bits8 ) == 0U )
#define BIT_SET(bits8, index) ( ( (uint8)1 << (index) ) | (bits8) )
#define BIT_CLR(bits8, index) ( (~( (uint8)1 << (index) )) & (bits8) )

static int set_bit(char *bitmap, uint32 bitmaxnum, uint32 bitmapsize)
{
    uint32 bit64_off;
    uint32 bit64_num = (bitmapsize / 8);
    uint32 ret;
    uint64* p;

    //printf("bitmaxnum=%d bitmapsize=%d\n", bitmaxnum, bitmapsize);

    p = (uint64*)bitmap;
    //printf("p=0x%x\n", p);
    for (bit64_off = 0; bit64_off < bit64_num; bit64_off++) {
        if ((~(p[bit64_off])) == 0ULL) {
            //printf("p[%d]=0x%llx : 0x%llx\n", bit64_off, ~p[bit64_off], 0ULL);
            continue;
        }
        //printf("FOUND:p[%d]=0x%llx : 0x%llx\n", bit64_off, ~p[bit64_off], 0ULL);
        break;
    }
    if (bit64_off == bit64_num) {
        //printf("set_bit:not found1\n");
        return -1;
    }

    int bit8_off;
    int bit8_mod;
    uint8* bitmap8 = (uint8*)(&p[bit64_off]);

    for (bit8_off = 0; bit8_off < 8; bit8_off++) {
        if ((~(bitmap8[bit8_off])) == 0U) {
            continue;
        }
        for (bit8_mod = 0; bit8_mod < 8; bit8_mod++) {
            if (BIT_IS_FREE8(bitmap8[bit8_off], bit8_mod)) {
                goto done;
            }
        }
    }
done:
    //printf("bi64_off=%d bit8_off=%d bit8_mod=%d\n", bit64_off, bit8_off, bit8_mod);

    ret = ( (bit64_off * 64) + (bit8_off * 8) + bit8_mod );
    if (ret >= bitmaxnum) {
        //printf("set_bit:not found2:ret=%d\n", ret);
        return -1;
    }
    //printf("&bitmap8[bit8_off]=0x%x value=0x%x\n", &bitmap8[bit8_off], BIT_SET(bitmap8[bit8_off], bit8_mod));
    bitmap8[bit8_off] = BIT_SET(bitmap8[bit8_off], bit8_mod);
    //printf("bitmap8[bit8_off]=0x%x bitmap[0]=0x%x\n", bitmap8[bit8_off], bitmap[0]);
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
    //printf("candidate=%d\n", candidate_index);
    if (candidate_index < 0) {
        return 0;
    }

    for (i = candidate_index; i < MallocSize_Num; i++) {
        for (j = 0; j < malloc_region.group_num; j++) {
            if (malloc_region.groups[j]->unit[i].bitfreenum > 0) {
                unit = &malloc_region.groups[j]->unit[i];
                goto done;
            }
        }
    }
done:
    //printf("i=%d j=%d unit=0x%x\n", i, j, unit);
    if (unit == NULL) {
        return 0;
    }
    int index = set_bit(unit->bitmap, malloc_data_info_table[i].bitmaxnum, malloc_data_info_table[i].bitmapsize);
    //printf("index=%d\n", index);
    if (index < 0) {
        return 0;
    }
    if (unit->region->data == NULL) {
        unit->region->data = malloc(MPU_MALLOC_REGION_UNIT_SIZE * 1024);
        ASSERT(unit->region->data != NULL);
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
                goto done;
            }
        }
    }
done:
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
    memset(group->unit[index].bitmap, 0, malloc_data_info_table[index].bitmapsize);
    ASSERT(group->unit[index].bitmap != NULL);

    group->unit_num++;
    return;
}

static bool group_has_free_unit(MallocRegionGroupType *group)
{
    int i;
    if (group->unit_num < MallocSize_Num) {
        return TRUE;
    }
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

#if 0
void mpu_malloc_test_get(void)
{
    uint32 addr[20];
    
    addr[0] = mpu_malloc_get_memory(31);
    addr[1] = mpu_malloc_get_memory(32);
    addr[2] = mpu_malloc_get_memory(33);
    addr[3] = mpu_malloc_get_memory(34);
    printf("31:0x%x\n", addr[0]);
    printf("32:0x%x\n", addr[1]);
    printf("33:0x%x\n", addr[2]);
    printf("34:0x%x\n", addr[3]);

    mpu_malloc_rel_memory(addr[0]);
    addr[4] = mpu_malloc_get_memory(10);
    printf("10:0x%x\n", addr[4]);

    return;
}
#endif
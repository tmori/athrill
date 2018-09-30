#ifdef OS_LINUX

#include "athrill_device.h"
#include "mpu_ops.h"
#include "symbol_ops.h"
#include <string.h>
#include <sys/file.h>
#include "assert.h"
#include "std_device_ops.h"


static uint32 athrill_device_func_call_addr = 0x0;
static uint32 athrill_device_raise_interrupt_addr = 0x0;

typedef struct {
	bool isLocked;
	AthrillDeviceMmapInfoType info;
} AthrillDeviceMmapInfoTableEntryType;

typedef struct {
	uint32 count;
	AthrillDeviceMmapInfoTableEntryType *entry;
} AthrillDeviceMmapInfoTableType;

static AthrillDeviceMmapInfoTableType athrill_mmap_table;

void device_init_athrill_device(void)
{
    uint32 addr;
    uint32 size;
    int err;

    err = symbol_get_gl("athrill_device_func_call", 
        strlen("athrill_device_func_call"), &addr, &size);
    if (err >= 0) {
		printf("athrill_device_func_call=0x%x\n", addr);
	    athrill_device_func_call_addr = addr;
    }
    err = symbol_get_gl("athrill_device_raise_interrupt", 
        strlen("athrill_device_raise_interrupt"), &addr, &size);
    if (err >= 0) {
		printf("athrill_device_raise_interrupt=0x%x\n", addr);
	    athrill_device_raise_interrupt_addr = addr;
    }

    return;
}

void athrill_device_set_mmap_info(AthrillDeviceMmapInfoType *info)
{
	int inx = athrill_mmap_table.count;
	athrill_mmap_table.count++;
	athrill_mmap_table.entry = realloc(athrill_mmap_table.entry,
			sizeof(AthrillDeviceMmapInfoTableEntryType) * athrill_mmap_table.count);
	ASSERT(athrill_mmap_table.entry != NULL);

	athrill_mmap_table.entry[inx].isLocked = FALSE;
	athrill_mmap_table.entry[inx].info = *info;
	return;
}

static inline AthrillDeviceMmapInfoTableEntryType *getMmapInfo(void *addr)
{
	int i;
	for (i = 0; i < athrill_mmap_table.count; i++) {
		if (addr == athrill_mmap_table.entry[i].info.addr) {
			return &athrill_mmap_table.entry[i];
		}
	}
	return NULL;
}

static void do_athrill_device_func_call(void)
{
    Std_ReturnType err;
    uint32 data;
	char cmd[256];

    if (athrill_device_func_call_addr == 0x0) {
        return;
    }

    err = mpu_get_data32(0U, athrill_device_func_call_addr, &data);
    if (err != 0) {
        return;
    }
    if (data == 0U) {
        return;
    }

    AthrillDeviceMmapInfoTableEntryType *mmapInfo = getMmapInfo(CAST_UINT32_TO_ADDR(data));
    if (mmapInfo == NULL) {
    	snprintf(cmd, sizeof(cmd), "athrill_extfunc.sh %u", data);
    	if (system(cmd) < 0) {
    		printf("can not execute athrill_extfunc.sh\n");
    	}
    }
    else {
    	int err;
    	if (mmapInfo->isLocked == FALSE) {
    		err = flock(mmapInfo->info.fd, LOCK_EX);
    		mmapInfo->isLocked = TRUE;
    	}
    	else {
    		err = flock(mmapInfo->info.fd, LOCK_UN);
    		mmapInfo->isLocked = FALSE;
    	}
		ASSERT(err == 0);
    }

    (void)mpu_put_data32(0U, athrill_device_func_call_addr, 0U);
	return;
}
static void do_athrill_device_external_raise_interrupt(void)
{
    Std_ReturnType err;
    uint32 data;

    if (athrill_device_raise_interrupt_addr == 0x0) {
        return;
    }

    err = mpu_get_data32(0U, athrill_device_raise_interrupt_addr, &data);
    if (err != 0) {
        return;
    }
    if (data == 0U) {
        return;
    }
    (void)mpu_put_data32(0U, athrill_device_raise_interrupt_addr, 0U);
	(void)intc_raise_intr(data);
	return;
}
#ifdef CONFIG_STAT_PERF
ProfStatType cpuemu_dev_adev1_prof;
ProfStatType cpuemu_dev_adev2_prof;

#define CPUEMU_DEV_ADEV1_PROF_START()	profstat_start(&cpuemu_dev_adev1_prof)
#define CPUEMU_DEV_ADEV1_PROF_END()		profstat_end(&cpuemu_dev_adev1_prof)
#define CPUEMU_DEV_ADEV2_PROF_START()	profstat_start(&cpuemu_dev_adev2_prof)
#define CPUEMU_DEV_ADEV2_PROF_END()		profstat_end(&cpuemu_dev_adev2_prof)
#else
#define CPUEMU_DEV_ADEV1_PROF_START()
#define CPUEMU_DEV_ADEV1_PROF_END()	
#define CPUEMU_DEV_ADEV2_PROF_START()
#define CPUEMU_DEV_ADEV2_PROF_END()	
#endif /* CONFIG_STAT_PERF */
void device_supply_clock_athrill_device(void)
{
    CPUEMU_DEV_ADEV1_PROF_START();
	do_athrill_device_func_call();
    CPUEMU_DEV_ADEV1_PROF_END();

    CPUEMU_DEV_ADEV2_PROF_START();
	do_athrill_device_external_raise_interrupt();
    CPUEMU_DEV_ADEV2_PROF_END();
    return;
}

#endif /* OS_LINUX */

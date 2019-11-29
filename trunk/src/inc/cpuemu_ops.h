#ifndef _CPUEMU_OPS_H_
#define _CPUEMU_OPS_H_

#include "std_types.h"
#include "cpu_config.h"
#include "target_cpu.h"
#include "cpu_config_ops.h"
#ifdef OS_LINUX
#include <sys/time.h>
#endif /* OS_LINUX */

typedef struct {
	uint64	total_clocks;
	uint64	intr_clocks;
	int     core_id_num;
	uint64  cpu_clocks[CPU_CONFIG_CORE_NUM];
#ifdef OS_LINUX
	struct timeval elaps_tv;
#endif /* OS_LINUX */
} CpuEmuElapsType;
extern void cpuemu_init(void *(*cpu_run)(void *), void *opt);
extern void cpuemu_get_elaps(CpuEmuElapsType *elaps);
#ifdef OS_LINUX
extern void cpuemu_start_elaps(void);
extern void cpuemu_end_elaps(void);
#endif /* OS_LINUX */
extern uint32 cpuemu_get_retaddr(CoreIdType core_id);
extern Std_ReturnType cpuemu_get_addr_pointer(uint32 addr, uint8 **data);
extern void cpuemu_get_register(CoreIdType core_id, TargetCoreType *cpu);

extern void *cpuemu_thread_run(void* arg);
extern uint64 cpuemu_get_cpu_end_clock(void);
extern void cpuemu_set_cpu_end_clock(uint64 clock);


extern Std_ReturnType cpuemu_set_comm_fifocfg(const char* fifocfg);
extern const char* cpuemu_get_comm_rx_fifo(void);
extern const char* cpuemu_get_comm_tx_fifo(void);

extern Std_ReturnType cpuemu_symbol_set(void);

extern void cpuemu_raise_intr(uint32 intno);


/*
 * the following enum values must be equal MpuAddressGetType(mpu_ops.h).
 */
typedef enum {
	MemoryAddressImplType_ROM = 0,
	MemoryAddressImplType_RAM,
	MemoryAddressImplType_MMAP,
	MemoryAddressImplType_MALLOC,
} MemoryAddressImplType;

typedef struct {
	MemoryAddressImplType type;
	uint32	start;
	uint32	size;
	void *mmap_addr;
} MemoryAddressType;

typedef struct {
	uint32				rom_num;
	MemoryAddressType	*rom;
	uint32				ram_num;
	MemoryAddressType	*ram;
} MemoryAddressMapType;
extern Std_ReturnType cpuemu_load_memmap(const char *path, MemoryAddressMapType *map);

extern Std_ReturnType cpuemu_load_devcfg(const char *path);
extern Std_ReturnType cpuemu_get_devcfg_value(const char* key, uint32 *value);
extern Std_ReturnType cpuemu_get_devcfg_value_hex(const char* key, uint32 *value);
extern Std_ReturnType cpuemu_get_devcfg_string(const char* key, char **value);

#endif /* _CPUEMU_OPS_H_ */

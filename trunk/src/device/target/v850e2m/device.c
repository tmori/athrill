#include "device.h"
#include "cpuemu_ops.h"
#include "device_ex_serial_ops.h"
#include "concrete_executor/target/dbg_target_serial.h"
#include <stdio.h>
#include "std_device_ops.h"

#ifdef CONFIG_STAT_PERF
ProfStatType cpuemu_dev_timer_prof;
ProfStatType cpuemu_dev_serial_prof;
ProfStatType cpuemu_dev_intr_prof;

#define CPUEMU_DEV_TIMER_PROF_START()	profstat_start(&cpuemu_dev_timer_prof)
#define CPUEMU_DEV_TIMER_PROF_END()		profstat_end(&cpuemu_dev_timer_prof)
#define CPUEMU_DEV_SERIAL_PROF_START()	profstat_start(&cpuemu_dev_serial_prof)
#define CPUEMU_DEV_SERIAL_PROF_END()		profstat_end(&cpuemu_dev_serial_prof)
#define CPUEMU_DEV_INTR_PROF_START()	profstat_start(&cpuemu_dev_intr_prof)
#define CPUEMU_DEV_INTR_PROF_END()		profstat_end(&cpuemu_dev_intr_prof)
#else
#define CPUEMU_DEV_TIMER_PROF_START()
#define CPUEMU_DEV_TIMER_PROF_END()
#define CPUEMU_DEV_SERIAL_PROF_START()
#define CPUEMU_DEV_SERIAL_PROF_END()
#define CPUEMU_DEV_INTR_PROF_START()
#define CPUEMU_DEV_INTR_PROF_END()
#endif /* CONFIG_STAT_PERF */

static DeviceExSerialOpType device_ex_serial_op = {
		.putchar = dbg_serial_putchar,
		.getchar = dbg_serial_getchar,
		.flush = NULL,
};
static DeviceExSerialOpType device_ex_serial_file_op = {
		.putchar = dbg_serial_putchar_file,
		.getchar = dbg_serial_getchar_file,
		.flush = dbg_serial_flush_file,
};

static void device_init_clock(MpuAddressRegionType *region)
{
	/*
	 * OSTC
	 */
	(void)device_io_write8(region, 0xFFFFF6C2, 0x01);

	/*
	 * ロック・レジスタ（ LOCKR）
	 */
	(void)device_io_write8(region, 0xFFFFF824, 0x00);

	return;
}


void device_init(CpuType *cpu, DeviceClockType *dev_clock)
{
	char *path;
	dev_clock->clock = 0;
	dev_clock->intclock = 0;
	dev_clock->min_intr_interval = DEVICE_CLOCK_MAX_INTERVAL;
	dev_clock->can_skip_clock = FALSE;

	device_init_clock(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);
	device_init_intc(cpu, &mpu_address_map.map[MPU_ADDRESS_REGION_INX_INTC]);
	device_init_timer(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);

	device_init_serial(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_SERIAL]);
	device_ex_serial_register_ops(0U, &device_ex_serial_op);
	if (cpuemu_get_devcfg_string("SERIAL_FILE_PATH", &path) == STD_E_OK) {
		device_ex_serial_register_ops(1U, &device_ex_serial_file_op);
	}

	return;
}

void device_supply_clock(DeviceClockType *dev_clock)
{
	dev_clock->min_intr_interval = DEVICE_CLOCK_MAX_INTERVAL;
	dev_clock->can_skip_clock = TRUE;

	CPUEMU_DEV_TIMER_PROF_START();
	device_supply_clock_timer(dev_clock);
	CPUEMU_DEV_TIMER_PROF_END();

	CPUEMU_DEV_SERIAL_PROF_START();
	device_supply_clock_serial(dev_clock);
	CPUEMU_DEV_SERIAL_PROF_END();

	CPUEMU_DEV_INTR_PROF_START();
	device_supply_clock_intc(dev_clock);
	CPUEMU_DEV_INTR_PROF_END();
	return;
}


int device_io_write8(MpuAddressRegionType *region,  uint32 addr, uint8 data)
{
	return region->ops->put_data8(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}
int device_io_write16(MpuAddressRegionType *region, uint32 addr, uint16 data)
{
	return region->ops->put_data16(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}

int device_io_write32(MpuAddressRegionType *region, uint32 addr, uint32 data)
{
	return region->ops->put_data32(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}

int device_io_read8(MpuAddressRegionType *region, uint32 addr, uint8 *data)
{
	return region->ops->get_data8(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}

int device_io_read16(MpuAddressRegionType *region, uint32 addr, uint16 *data)
{
	return region->ops->get_data16(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}

int device_io_read32(MpuAddressRegionType *region, uint32 addr, uint32 *data)
{
	return region->ops->get_data32(region, CPU_CONFIG_CORE_ID_0, (addr & region->mask), data);
}

void device_raise_int(uint16 intno)
{
	intc_raise_intr(intno);
}



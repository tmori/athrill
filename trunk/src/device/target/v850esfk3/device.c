#include "device.h"
#include "device_ex_serial_ops.h"
#include "dbg_can.h"
#include "can.h"
#include "concrete_executor/target/dbg_target_serial.h"
#include <stdio.h>

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

static DeviceExSerialOpType device_ex_serial_op = {
		.putchar = dbg_serial_putchar,
		.getchar = dbg_serial_getchar,
};

void device_init(CpuType *cpu, DeviceClockType *dev_clock)
{
	dev_clock->clock = 0;
	dev_clock->intclock = 0;

	device_init_clock(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);
	device_init_intc(cpu, &mpu_address_map.map[MPU_ADDRESS_REGION_INX_INTC]);
	device_init_timer(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);
	device_init_timer_m(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);

	device_init_serial(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_SERIAL]);
	device_ex_serial_register_ops(0U, &device_ex_serial_op);

	device_can_register_ops( NULL, &dbg_can_ops);
	device_init_can( &mpu_address_map.map[MPU_ADDRESS_REGION_INX_CAN]);

	device_init_adc(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);
	//device_init_wdg(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);
	device_init_comm(&mpu_address_map.map[MPU_ADDRESS_REGION_INX_PH0]);

	return;
}

void device_supply_clock(DeviceClockType *dev_clock)
{

	device_supply_clock_timer(dev_clock);
#ifndef DISABLE_DEVICE_TIMER_M
	device_supply_clock_timer_m(dev_clock);
#endif /* DISABLE_DEVICE_TIMER_M */
	device_supply_clock_serial(dev_clock);
#ifndef DISABLE_DEVICE_CAN
	device_supply_clock_can(dev_clock);
#endif /* DISABLE_DEVICE_CAN */
	//device_supply_clock_adc(dev_clock);
	//device_supply_clock_wdg(dev_clock);
#ifndef DISABLE_DEVICE_COMM
	device_supply_clock_comm(dev_clock);
#endif /* DISABLE_DEVICE_COMM */
	device_supply_clock_intc(dev_clock);
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



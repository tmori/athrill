#ifndef _DEVICE_H_
#define _DEVICE_H_

#include "std_types.h"
#include "cpu.h"
#include "mpu_config.h"
#include "mpu.h"
#include "std_device_ops.h"

/*
 * デバイス初期化関数
 */
extern void device_init_intc(CpuType *cpu, MpuAddressRegionType *region);
extern void device_init_timer(MpuAddressRegionType *region);
extern void device_init_timer_m(MpuAddressRegionType *region);
extern void device_init_serial(MpuAddressRegionType *region);
extern void device_init_can(MpuAddressRegionType *region);
extern void device_init_adc(MpuAddressRegionType *region);
extern void device_init_wdg(MpuAddressRegionType *region);
extern void device_init_comm(MpuAddressRegionType *region);

/*
 * デバイスクロック供給
 */
extern void device_supply_clock_intc(DeviceClockType *dev_clock);
extern void device_supply_clock_timer(DeviceClockType *dev_clock);
extern void device_supply_clock_timer_m(DeviceClockType *dev_clock);
extern void device_supply_clock_serial(DeviceClockType *dev_clock);
extern void device_supply_clock_can(DeviceClockType *dev_clock);
extern void device_supply_clock_adc(DeviceClockType *dev_clock);
extern void device_supply_clock_wdg(DeviceClockType *dev_clock);
extern void device_supply_clock_comm(DeviceClockType *dev_clock);

/*
 * 割込み関数
 */
extern void device_raise_int(uint16 intno);

/*
 * メモリアクセス関数
 */
extern int device_io_write8(MpuAddressRegionType *region, uint32 addr, uint8 data);
extern int device_io_write16(MpuAddressRegionType *region, uint32 addr, uint16 data);
extern int device_io_write32(MpuAddressRegionType *region, uint32 addr, uint32 data);

extern int device_io_read8(MpuAddressRegionType *region, uint32 addr, uint8 *data);
extern int device_io_read16(MpuAddressRegionType *region, uint32 addr, uint16 *data);
extern int device_io_read32(MpuAddressRegionType *region, uint32 addr, uint32 *data);

/*
 * 割込みコントローラAPI
 */
extern void intc_clr_nmi(TargetCoreType *cpu);
extern void intc_clr_currlvl_ispr(TargetCoreType *cpu);

#endif /* _DEVICE_H_ */

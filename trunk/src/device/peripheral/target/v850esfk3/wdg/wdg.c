#include "device/inc/wdg.h"
#include "intc_ops.h"
#include <stdio.h>

typedef struct {
	uint16 state;
	uint32 cnt;
} WdgDeviceType;

static WdgDeviceType WdgDevice;
static MpuAddressRegionType *wdg_region;

void device_init_wdg(DeviceClockType *device, MpuAddressRegionType *region)
{
	wdg_region = region;

	device->dev.wdg = &WdgDevice;
	WdgDevice.state = 0;
	WdgDevice.cnt = 0;
	return;
}

void device_supply_clock_wdg(DeviceClockType *dev_clock)
{
	uint16 reg16;
	/*
	 * カウントインクリメント
	 */
	WdgDevice.cnt++;

	/*
	 * タイムアウト確認
	 */
	if (WdgDevice.cnt >= DVICE_WDG_TIMEOUT) {
		intc_raise_nmi(NULL, INTC_NMINO_NMI);
	}

	/*
	 * パルス有無確認
	 */
	(void)device_io_read16(dev_clock, 0xFFFFF404, &reg16);
	reg16 = reg16 & DVICE_WDG_PORT_BIT;
	if (WdgDevice.state != reg16) {
		WdgDevice.cnt = 0;
		WdgDevice.state = reg16;
	}

	return;
}

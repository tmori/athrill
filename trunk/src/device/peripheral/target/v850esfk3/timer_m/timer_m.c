#include "inc/timer_m.h"
#include <stdio.h>

typedef enum {
	TIMER_MODE_STOP,
	TIMER_MODE_RUN
} TimerMModeType;

typedef struct {
	uint16 cnt;
	uint16 precnt;
	TimerMModeType mode;
	uint16 compare;
	uint16 intno;
	uint16 fd;
} TimerMDeviceType;

static TimerMDeviceType TimerMDevice;
static MpuAddressRegionType *timer_m_region;

void device_init_timer_m(MpuAddressRegionType *region)
{
	timer_m_region = region;

	TimerMDevice.cnt = 0;
	TimerMDevice.precnt = 0;
	TimerMDevice.mode = TIMER_MODE_STOP;
	TimerMDevice.compare = 0;
	TimerMDevice.intno = 30;
	TimerMDevice.fd = 4;

	return;
}

static int device_timer_do_update(void)
{
	TimerMDeviceType *timer = &TimerMDevice;
	uint8 data8;

	/*
	 * コンペア値の取得
	 */
	(void)device_io_read16(timer_m_region, MPU_TMM_ADDR_TM0CMP0, &timer->compare);

	/*
	 * 動作判定
	 */
	(void)device_io_read8(timer_m_region, MPU_TMM_ADDR_TM0CTL0, &data8);
	if (timer->mode == TIMER_MODE_STOP) {
		if ((data8 & (1 << MPU_TMM_ADDR_TM0CTL0_TM0CE)) == (1 << MPU_TMM_ADDR_TM0CTL0_TM0CE)) {
			timer->cnt = 0;
			timer->precnt = 0;
			timer->mode = TIMER_MODE_RUN;
			return FALSE;
		}
		else {
			return FALSE;
		}
	}
	else { /* RUN */
		if ((data8 & (1 << MPU_TMM_ADDR_TM0CTL0_TM0CE)) != (1 << MPU_TMM_ADDR_TM0CTL0_TM0CE)) {
			timer->mode = TIMER_MODE_STOP;
			return FALSE;
		}
	}

	if (timer->precnt == 0) {
		timer->precnt++;
		return FALSE;
	}

	if (timer->cnt != timer->compare) {
		timer->cnt++;
		return FALSE;
	}

	timer->cnt = 0;
	return TRUE;
}


void device_supply_clock_timer_m(DeviceClockType *dev_clock)
{
	if ((dev_clock->clock % TimerMDevice.fd) != 0) {
			return;
	}

	if (device_timer_do_update() == TRUE) {
		device_raise_int(TimerMDevice.intno);
	}
	return;
}

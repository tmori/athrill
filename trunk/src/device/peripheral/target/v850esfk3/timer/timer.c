#include "inc/timer.h"
#include "cpuemu_ops.h"
#include <stdio.h>

typedef enum {
	TIMER_MODE_STOP,
	TIMER_MODE_RUN
} TimerModeType;

typedef struct {
	uint16 cnt;
	TimerModeType mode;
	uint16 compare0;
	uint16 compare1;
	uint16 compare0_intno;
	uint16 compare1_intno;
	uint16 overflow_intno;
	bool raise_int_compare0;
	bool raise_int_compare1;
	bool raise_int_overflow;
	uint16 fd;
} TimerDeviceType;

static TimerDeviceType TimerDevice[TAAnChannelNum];
static MpuAddressRegionType *timer_region;

void device_init_timer(MpuAddressRegionType *region)
{
	int i = 0;
	uint16 base;
	uint32 value = 32;

	(void)cpuemu_get_devcfg_value("DEVICE_CONFIG_TIMER_FD", &value);
	//printf("timer value=%d\n", value);

	timer_region = region;

	for (i = 0; i < TAAnChannelNum; i++) {
		TimerDevice[i].cnt = 0;
		TimerDevice[i].mode = TIMER_MODE_STOP;
		TimerDevice[i].compare0 = 0;
		TimerDevice[i].compare1 = 0;
		//TimerDevice[i].fd = 32;
		TimerDevice[i].fd = value;
	}

	for (i = 0; i < 5; i++) {
		base = 15 + (i * 3);
		TimerDevice[i].overflow_intno = base + 0;
		TimerDevice[i].compare0_intno = base + 1;
		TimerDevice[i].compare1_intno = base + 2;
	}
	for (i = 5; i < TAAnChannelNum; i++) {
		base = 96 + ((i - 5) * 3) + 1;
		TimerDevice[i].overflow_intno = base + 0;
		TimerDevice[i].compare0_intno = base + 1;
		TimerDevice[i].compare1_intno = base + 2;
	}

	return;
}

static void device_timer_do_mode(DeviceClockType *device, int ch)
{
	TimerModeType org;
	TimerDeviceType *timer = &(TimerDevice[ch]);
	uint8 data;
	uint16 data16;

	org = timer->mode;
	(void)device_io_read8(timer_region, TAAnCTL0(ch), &data);
	if ((data & (1 << 7)) == (1 << 7)) {
		timer->cnt = 0;
		timer->mode = TIMER_MODE_RUN;
	}
	else {
		timer->mode = TIMER_MODE_STOP;
		timer->cnt = 0;
		(void)device_io_write16(timer_region, TAAnCNT(ch), 0);
	}

	/*
	 * コンペア値の取得
	 */
	(void)device_io_read16(timer_region, TAAnCCR0(ch), &timer->compare0);
	(void)device_io_read16(timer_region, TAAnCCR1(ch), &timer->compare1);

#if 0
	if (ch == 6) {
		printf("TAAnCTL0=0x%x\n", data);
		printf("TAAnCCR0=%d\n", timer->compare0);
		fflush(stdout);
	}
#endif

	/*
	 * カウンタ値の取得
	 */
	(void)device_io_read16(timer_region, TAAnCNT(ch), &data16);
	timer->cnt = data16;
	if (org != timer->mode) {
		//printf("%d:timer(%d) mode:%d => %d: counter=%d/%d\n", device->clock, ch, org, timer->mode, timer->cnt, timer->compare0);
		fflush(stdout);
	}
	/*
	 * TODO クロック設定は省略する
	 */
	return;
}
static void device_timer_do_update(DeviceClockType *device, int ch)
{
	TimerDeviceType *timer = &(TimerDevice[ch]);

	if (timer->mode == TIMER_MODE_STOP) {
		return;
	}

	if (timer->cnt == timer->compare1) {
		timer->raise_int_compare1 = TRUE;
	}
	else {
	}

	if (timer->cnt == timer->compare0) {
		timer->cnt = 0;
		timer->raise_int_compare0 = TRUE;
	}
	else {
		//printf("device_timer_do_update(%d):compare0=%d cnt=%d\n", ch, timer->compare0, timer->cnt);
		timer->cnt++;
	}

	(void)device_io_write16(timer_region, TAAnCNT(ch), timer->cnt);

	return;
}
static void device_timer_do_interrupt(DeviceClockType *device, int ch)
{
	TimerDeviceType *timer = &(TimerDevice[ch]);

	if (ch != 2) {
		return;
	}

	if (timer->raise_int_compare0 == TRUE) {
		//printf("%d:device_timer_do_interrupt(%d):compare0=%d intno=%d cnt=%d\n", device->clock, ch, timer->compare0, timer->compare0_intno, timer->cnt);
		//fflush(stdout);
		device_raise_int(timer->compare0_intno);
		timer->raise_int_compare0 = FALSE;
	}
	if (timer->raise_int_compare1 == TRUE) {
		//printf("device_timer_do_interrupt(%d):compare1 cnt=%d\n", ch, timer->cnt);
		//fflush(stdout);
		//device_raise_int(device, timer->compare1_intno);
		timer->raise_int_compare1 = FALSE;
	}

	return;
}

#define INLINE_device_supply_clock_timer(dev_clock, ch)	\
do {	\
	if ((dev_clock->clock % TimerDevice[ch].fd) == 0) {	\
		device_timer_do_mode(dev_clock, ch);	\
		device_timer_do_update(dev_clock, ch);	\
		device_timer_do_interrupt(dev_clock, ch);	\
	}	\
} while(0)

void device_supply_clock_timer(DeviceClockType *dev_clock)
{
#if 1
	INLINE_device_supply_clock_timer(dev_clock, 2);
	INLINE_device_supply_clock_timer(dev_clock, 3);
#else
	int ch;


	for (ch = 0; ch < TAAnChannelNum; ch++) {
		if ((dev_clock->clock % TimerDevice[ch].fd) != 0) {
			continue;
		}
		//モード設定
		device_timer_do_mode(dev_clock, ch);

		//カウンタ更新
		device_timer_do_update(dev_clock, ch);

		//割込み生成
		device_timer_do_interrupt(dev_clock, ch);
	}
#endif
	return;
}

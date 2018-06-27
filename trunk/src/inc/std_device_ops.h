#ifndef _STD_DEVICE_OPS_H_
#define _STD_DEVICE_OPS_H_

#include "cpu_config_ops.h"
#ifdef OS_LINUX
#include <sys/time.h>

static inline void cpuemu_timeval_sub(struct timeval *tv1, struct timeval *tv2, struct timeval *tv_result)
{
	tv_result->tv_sec = tv1->tv_sec - tv2->tv_sec;
	if (tv1->tv_usec >= tv2->tv_usec) {
		tv_result->tv_usec = tv1->tv_usec - tv2->tv_usec;
	}
	else {
		tv_result->tv_usec = (1000000 + tv1->tv_usec) - tv2->tv_usec;
	}
	return;
}
static inline void cpuemu_timeval_add(struct timeval *tv1, struct timeval *tv2, struct timeval *tv_result)
{
	tv_result->tv_sec = tv1->tv_sec + tv2->tv_sec;
	tv_result->tv_usec = tv1->tv_usec + tv2->tv_usec;
	if (tv_result->tv_usec >= 1000000) {
		tv_result->tv_usec -= 1000000;
		tv_result->tv_sec++;
	}
	return;
}
#endif /* OS_LINUX */

/*
 * スキップ可能な最大クロック数
 */
#define DEVICE_CLOCK_MAX_INTERVAL	-1
typedef struct {
	uint64 clock;
	uint64 intclock;//割込み処理で消費している時間
#ifdef OS_LINUX
	struct timeval elaps_tv;
	struct timeval start_tv;
#endif /* OS_LINUX */


	/**************************************
	 * CPUがHALT状態になった場合，タイマ割り込みの
	 * 発生する時間まで時間を飛ばす機能
	 *
	 * 本機能はまだ検討中のものであるため，注意して
	 * 使用する．
	 *
	 **************************************/
	/*
	 * 本機能のON/OFFをする
	 */
	bool	enable_skip;
	/*
	 * デバイスが時間飛ばし可能かどうかを判断した結果を格納する
	 */
	bool	can_skip_clock;
	/*
	 * 全デバイス中で次の割り込みが発生するまでの時間の最小値
	 */
	uint64 	min_intr_interval;
} DeviceClockType;

extern void device_init(CpuType *cpu, DeviceClockType *dev_clock);
extern void device_supply_clock(DeviceClockType *dev_clock);
/*
 * デバイスクロック参照
 */
extern void device_get_clock(DeviceClockType *dev_clock);

/*
 * 割込みコントローラ制御
 */
extern int intc_raise_intr(uint32 intno);

#endif /* _STD_DEVICE_OPS_H_ */

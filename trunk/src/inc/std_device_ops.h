#ifndef _STD_DEVICE_OPS_H_
#define _STD_DEVICE_OPS_H_

#include "cpu_config_ops.h"
#ifdef OS_LINUX
#include <sys/time.h>
#include <time.h>

static inline void cpuemu_timeval_sub(struct timeval *tv1, struct timeval *tv2, struct timeval *tv_result)
{
	tv_result->tv_sec = tv1->tv_sec - tv2->tv_sec;
	if (tv1->tv_usec >= tv2->tv_usec) {
		tv_result->tv_usec = tv1->tv_usec - tv2->tv_usec;
	}
	else {
		tv_result->tv_usec = (1000000 + tv1->tv_usec) - tv2->tv_usec;
		tv_result->tv_sec--;
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
static inline void cpuemu_timespec_sub(struct timespec *tv1, struct timespec *tv2, struct timespec *tv_result)
{
	tv_result->tv_sec = tv1->tv_sec - tv2->tv_sec;
	if (tv1->tv_nsec >= tv2->tv_nsec) {
		tv_result->tv_nsec = tv1->tv_nsec - tv2->tv_nsec;
	}
	else {
		tv_result->tv_nsec = (1000000000 + tv1->tv_nsec) - tv2->tv_nsec;
		tv_result->tv_sec--;
	}
	return;
}

#define PROF_STAT_LOOP_MAX	1
typedef struct {
	uint64 max; /* nsec */
	uint64 total; /* nsec */
	uint64 count;
	struct timespec start_ts;
	struct timespec end_ts;
	struct timespec elaps_ts;
} ProfStatType;

static inline void profstat_start(ProfStatType *prof)
{
	clock_gettime(CLOCK_REALTIME, &prof->start_ts);
	return;
}
static inline void profstat_end(ProfStatType *prof)
{
	if (prof->start_ts.tv_sec == 0) {
		return;
	}
	clock_gettime(CLOCK_REALTIME, &prof->end_ts);
	cpuemu_timespec_sub(&prof->end_ts, &prof->start_ts, &prof->elaps_ts);
	/* set max */
	uint64 elaps = ((uint64)(prof->elaps_ts.tv_sec) * ((uint64)1000000000)) + (uint64)(prof->elaps_ts.tv_nsec);
	//uint64 elaps =  (uint64)(prof->elaps.tv_usec);
	//printf("%llu start=%lu end=%lu loop=%u\n", elaps, prof->start_time.tv_sec, prof->end_time.tv_sec, prof->loop);
	if (prof->max < elaps) {
		prof->max = elaps;
	}
	/* set average */
	prof->count++;
	prof->total += elaps;
	return;
}
#ifdef CONFIG_STAT_PERF
#define DEBUG_STAT_NUM	5
extern ProfStatType cpuemu_cpu_total_prof;
extern ProfStatType cpuemu_dev_total_prof;
extern ProfStatType cpuemu_dbg_total_prof[DEBUG_STAT_NUM];
extern ProfStatType cpuemu_dev_timer_prof;
extern ProfStatType cpuemu_dev_serial_prof;
extern ProfStatType cpuemu_dev_intr_prof;
extern ProfStatType cpuemu_dev_adev1_prof;
extern ProfStatType cpuemu_dev_adev2_prof;
extern ProfStatType cpuemu_tool1_prof;
extern ProfStatType cpuemu_tool2_prof;

#define PROFSTAT_START(arg)	profstat_start(arg)
#define PROFSTAT_END(arg)	profstat_end(arg)
#else
#define PROFSTAT_START(arg)
#define PROFSTAT_END(arg)
#endif
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

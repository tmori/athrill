/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2007-2016 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: test_dlynse.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/*
 *		sil_dly_nseに関するテスト
 */

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include "syssvc/test_svc.h"
#include "syssvc/histogram.h"
#include "kernel_cfg.h"
#include "test_dlynse.h"

/*
 *  SIL_DLY_TIM1とSIL_DLY_TIM2を参照するために，カーネル用のヘッダファ
 *  イルをインクルードする．
 */
#include "target_kernel_impl.h"

#define	NO_LOOP		1000000UL

HRTCNT	empty_time;

static void
test_empty(void)
{
	HRTCNT	stime, etime;
	volatile ulong_t	i;

	stime = fch_hrt();
	for (i = 0; i < NO_LOOP; i++) {
	}
	etime = fch_hrt();
	empty_time = etime - stime;
#ifdef TCYC_HRTCNT
	if (etime < stime) {
		empty_time += TCYC_HRTCNT;
	}
#endif /* TCYC_HRTCNT */
	syslog(LOG_NOTICE, "empty loop: %u", empty_time);
	(void) syslog_fls_log();
}

static void
test_dly_nse(ulong_t dlytim)
{
	HRTCNT	stime, etime, diff;
	ulong_t	delay_time;
	volatile ulong_t	i;

	stime = fch_hrt();
	for (i = 0; i < NO_LOOP; i++) {
		sil_dly_nse(dlytim);
	}
	etime = fch_hrt();
	diff = etime - stime;
#ifdef TCYC_HRTCNT
	if (etime < stime) {
		diff += TCYC_HRTCNT;
	}
#endif /* TCYC_HRTCNT */
	delay_time = (diff - empty_time) / 1000U;
	syslog(LOG_NOTICE, "sil_dly_nse(%lu): %lu %s", dlytim, delay_time,
									delay_time >= dlytim ? "OK" : "NG");
	(void) syslog_fls_log();
}

void
main_task(intptr_t exinf)
{
	test_start(__FILE__);

	test_empty();

	syslog(LOG_NOTICE, "-- for fitting parameters --");
	test_dly_nse(0);
	test_dly_nse(SIL_DLY_TIM1);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 1);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 2);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 3);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 4);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 5);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 6);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 7);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 8);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 9);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 10);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 20);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 50);

	syslog(LOG_NOTICE, "-- for checking boundary conditions --");
	test_dly_nse(SIL_DLY_TIM1 + 1);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 1 + 1);
	test_dly_nse(SIL_DLY_TIM1 + SIL_DLY_TIM2 * 2 + 1);

	check_finish(0);
}

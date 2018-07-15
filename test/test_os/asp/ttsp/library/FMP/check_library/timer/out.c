/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
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
 *  $Id: out.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "ttsp_test_lib.h"
#include "out.h"

#define LOOP_TIME 10

/* sil_dly_nseによる遅延処理設定(10ミリ秒) */
#define SIL_DELAY_TIME 10000000

void main_task(intptr_t exinf){
	int i;
	ulong_t timeout = 0;
	SYSTIM system1, system2;
	ID prcid, tskid;
	get_pid(&prcid);
	get_tid(&tskid);

	/* 時間が進んでいることを確認 */
	get_tim(&system1);
	sil_dly_nse(SIL_DELAY_TIME);
	get_tim(&system2);
	check_assert(system1 < system2);
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
	syslog_1(LOG_NOTICE, "[PE%d]timer_interrupt : OK", prcid);

	/* 時間が止まることを確認 */
	if (TOPPERS_MASTER_PRCID == prcid){
		ttsp_target_stop_tick();
	}
	ttsp_barrier_sync(1, TNUM_PRCID);
	get_tim(&system1);
	sil_dly_nse(SIL_DELAY_TIME);
	get_tim(&system2);
	check_assert(system1 == system2);
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
	syslog_1(LOG_NOTICE, "[PE%d]ttsp_target_stop_tick() : OK", prcid);

	/* 特定のプロセッサの時間を1ティックずつ進められることを確認
	   (時間が進むまでリターンしない)*/
	for(i = 0; i < LOOP_TIME; i++){
		get_tim(&system1);
		ttsp_barrier_sync(i + 2, TNUM_PRCID);
		if (TOPPERS_MASTER_PRCID == prcid) {
#ifdef TOPPERS_SYSTIM_GLOBAL
			ttsp_target_gain_tick_pe(TOPPERS_SYSTIM_PRCID, true);
#else /* TOPPERS_SYSTIM_LOCAL */
			ttsp_target_gain_tick_pe(1, true);
#if TNUM_PRCID >= 2
			ttsp_target_gain_tick_pe(2, true);
#endif /* TNUM_PRCID >= 2 */
#if TNUM_PRCID >= 3
			ttsp_target_gain_tick_pe(3, true);
#endif /* TNUM_PRCID >= 3 */
#if TNUM_PRCID >= 4
			ttsp_target_gain_tick_pe(4, true);
#endif /* TNUM_PRCID >= 4 */
#endif /* TOPPERS_SYSTIM_LOCAL */
		}
		ttsp_barrier_sync(++i + 2, TNUM_PRCID);
		get_tim(&system2);
		check_assert((system1 + 1) == system2);
	}
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
#ifdef TOPPERS_SYSTIM_GLOBAL
	syslog_2(LOG_NOTICE, "[PE%d]ttsp_target_gain_tick_pe(%d, true) : OK", prcid, TOPPERS_SYSTIM_PRCID);
#else /* TOPPERS_SYSTIM_LOCAL */
	syslog_2(LOG_NOTICE, "[PE%d]ttsp_target_gain_tick_pe(%d, true) : OK", prcid, prcid);
#endif /* TOPPERS_SYSTIM_LOCAL */

	/* 特定のプロセッサの時間を1ティックずつ進められることを確認
	   (すぐにリターンする)*/
	for(i = 0; i < LOOP_TIME; i++){
		get_tim(&system1);
		ttsp_barrier_sync(i + LOOP_TIME + 4, TNUM_PRCID);
		if (TOPPERS_MASTER_PRCID == prcid) {
#ifdef TOPPERS_SYSTIM_GLOBAL
			ttsp_target_gain_tick_pe(TOPPERS_SYSTIM_PRCID, false);
#else /* TOPPERS_SYSTIM_LOCAL */
			ttsp_target_gain_tick_pe(1, false);
#if TNUM_PRCID >= 2
			ttsp_target_gain_tick_pe(2, false);
#endif /* TNUM_PRCID >= 2 */
#if TNUM_PRCID >= 3
			ttsp_target_gain_tick_pe(3, false);
#endif /* TNUM_PRCID >= 3 */
#if TNUM_PRCID >= 4
			ttsp_target_gain_tick_pe(4, false);
#endif /* TNUM_PRCID >= 4 */
#endif /* TOPPERS_SYSTIM_LOCAL */
		}
		do {
			get_tim(&system2);
			timeout++;
			if (timeout > TTSP_LOOP_COUNT) {
				syslog_1(LOG_ERROR, "## PE %d : main_task caused a timeout", prcid);
				syslog_1(LOG_ERROR, "## by \"ttsp_target_gain_tick_pe(%d, false)\".", prcid);
				ttsp_set_cp_state(false);
				ext_ker();
			}
			sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
		} while (system1 == system2);
		ttsp_barrier_sync(++i + LOOP_TIME + 4, TNUM_PRCID);
	}
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
#ifdef TOPPERS_SYSTIM_GLOBAL
	syslog_2(LOG_NOTICE, "[PE%d]ttsp_target_gain_tick_pe(%d, false) : OK", prcid, TOPPERS_SYSTIM_PRCID);
#else /* TOPPERS_SYSTIM_LOCAL */
	syslog_2(LOG_NOTICE, "[PE%d]ttsp_target_gain_tick_pe(%d, false) : OK", prcid, prcid);
#endif /* TOPPERS_SYSTIM_LOCAL */

	/* 全体のプロセッサの時間を1ティックずつ進められることを確認 */
#ifdef TOPPERS_SYSTIM_LOCAL
	for(i = 0; i < LOOP_TIME; i++){
		get_tim(&system1);
		ttsp_barrier_sync(i + (LOOP_TIME * 2) + 4, TNUM_PRCID);
		if (TOPPERS_MASTER_PRCID == prcid) {
			ttsp_target_gain_tick();
		}
		ttsp_barrier_sync(++i + (LOOP_TIME * 2) + 4, TNUM_PRCID);
		get_tim(&system2);
		check_assert((system1 + 1) == system2);
	}
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
	syslog_2(LOG_NOTICE, "[PE%d]ttsp_target_gain_tick() : OK", prcid, prcid);
#endif /* TOPPERS_SYSTIM_LOCAL */

	/* 時間が動き出すことを確認 */
	if (TOPPERS_MASTER_PRCID == prcid) {
		ttsp_target_start_tick();
	}
	ttsp_barrier_sync((LOOP_TIME * 3) + 4, TNUM_PRCID);
	get_tim(&system1);
	sil_dly_nse(SIL_DELAY_TIME);
	get_tim(&system2);
	check_assert(system1 < system2);
	syslog_2(LOG_NOTICE, "[PE%d]system1 : %d", prcid, system1);
	syslog_2(LOG_NOTICE, "[PE%d]system2 : %d", prcid, system2);
	syslog_1(LOG_NOTICE, "[PE%d]ttsp_target_start_tick() : OK", prcid);

	ttsp_barrier_sync((LOOP_TIME * 3) + 5, TNUM_PRCID);

	if (TOPPERS_MASTER_PRCID == prcid) {
		ttsp_mp_check_finish(prcid, 1);
	}
}

void ttsp_test_lib_init(intptr_t exinf){
	ttsp_initialize_test_lib();
}

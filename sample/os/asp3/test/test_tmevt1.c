/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_tmevt1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		タイムイベント管理モジュールのテスト(1)
 *
 * 【テストの目的】
 *
 *	・システム時刻が単調増加すること
 *	・アラームハンドラの実行開始までの時間が単調減少すること
 *	・adj_timによりシステム時刻を進めた場合の動作の確認
 *	・adj_timによりシステム時刻を遅らせた場合の動作の確認
 *
 * 【テスト項目】
 *
 *  
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	ALM1:  アラームハンドラ
 *	ALM2:  アラームハンドラ
 *	ALM3:  アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-1が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(wait_alarm1(&alarm1_flag1))
 *	== ALM1-1（1回目）==
 *	2:	DO(alarm1_flag1 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	3:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-2が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(wait_alarm1(&alarm1_flag2))
 *	== ALM1-2（2回目）==
 *	4:	DO(alarm1_flag2 = true)
 *	5:	sta_alm(ALM1, TEST_TIME_CP) ... ALM1-3が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(skip_check = true)
 *		RETURN
 *	== TASK1（続き）==
 *	6:	DO(wait_alarm1(&alarm1_flag3))
 *	== ALM1-3（3回目）==
 *	7:	DO(alarm1_flag3 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	8:	sta_alm(ALM1, 3 * TEST_TIME_CP) ... ALM1-4が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(refer_alarm1())
 *  9:	dly_tsk(TEST_TIME_CP) ... どのような時間でも良い
 *		DO(prev_lefttim -= TEST_TIME_CP)
 *		DO(refer_alarm1())
 *  10:	adj_tim(TEST_TIME_CP)
 *		DO(wait_alarm1(&alarm1_flag4))
 *	== ALM1-4（4回目）==
 *	11:	DO(alarm1_flag4 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	12:	sta_alm(ALM1, 4 * TEST_TIME_CP) ... ALM1-5が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(refer_alarm1())
 *  13:	dly_tsk(TEST_TIME_CP) ... どのような時間でも良い
 *		DO(prev_lefttim -= TEST_TIME_CP)
 *		DO(refer_alarm1())
 *  14:	adj_tim(-TEST_TIME_CP)
 *		DO(prev_lefttim += TEST_TIME_CP)
 *	15:	DO(wait_alarm1(&alarm1_flag5))
 *	== ALM1-5（5回目）==
 *	16:	DO(alarm1_flag5 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	17:	sta_alm(ALM1, 4 * TEST_TIME_CP) ... ALM1-6が実行開始するまで
 *		sta_alm(ALM2, 3 * TEST_TIME_CP) ... stp_almで停止するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *		DO(refer_alarm1())
 *  18:	dly_tsk(TEST_TIME_CP) ... どのような時間でも良い
 *		DO(prev_lefttim -= TEST_TIME_CP)
 *		DO(refer_alarm1())
 *	19:	stp_alm(ALM2)
 *		ref_alm(ALM2, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *		DO(refer_alarm1())
 *	20:	DO(wait_alarm1(&alarm1_flag6))
 *	== ALM1-6（6回目）==
 *	21:	DO(alarm1_flag6 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	22:	sta_alm(ALM1, 3 * TEST_TIME_CP) ... ALM1-7が実行開始するまで
 *		sta_alm(ALM2, 2 * TEST_TIME_CP) ... stp_almで停止するまで
 *		sta_alm(ALM3, TEST_TIME_CP) ... ALM3-1が実行開始するまで
 *		DO(prev_lefttim = UINT32_MAX)
 *	23:	DO(wait_alarm1(&alarm1_flag7))
 *	== ALM3-1（1回目）==
 *	24:	stp_alm(ALM2)
 *		RETURN
 *	== ALM1-7（7回目）==
 *	25:	DO(alarm1_flag7 = true)
 *		RETURN
 *	== TASK1（続き）==
 *	26:	ref_alm(ALM2, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	27:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_tmevt1.h"

volatile bool_t		alarm1_flag1 = false;
volatile bool_t		alarm1_flag2 = false;
volatile bool_t		alarm1_flag3 = false;
volatile bool_t		alarm1_flag4 = false;
volatile bool_t		alarm1_flag5 = false;
volatile bool_t		alarm1_flag6 = false;
volatile bool_t		alarm1_flag7 = false;

RELTIM		prev_lefttim;			/* lefttimの増加チェック用の変数 */
bool_t		skip_check = false;		/* 増加チェックを1回スキップする */

SYSTIM		prev_systim;			/* systimの増加チェック用の変数 */

void
refer_alarm1(void)
{
	T_RALM	ralm;
	bool_t	skip_check_bak;
	ER		ercd;

	skip_check_bak = skip_check;
	skip_check = false;

	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	if ((ralm.almstat & TALM_STA) != 0U) {
//		syslog(LOG_NOTICE, "lefttim = %tu", ralm.lefttim);
		if (ralm.lefttim > prev_lefttim && !skip_check_bak) {
			syslog(LOG_ERROR, "lefttim increases (%tu -> %tu)",
										prev_lefttim, ralm.lefttim);
		}
		prev_lefttim = ralm.lefttim;
	}
	else {
//		syslog(LOG_NOTICE, "ALM1 is stopped.");
		prev_lefttim = 0U;
	}
}

void
refer_systim(void)
{
	SYSTIM	systim;
	ER		ercd;

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

//	syslog(LOG_NOTICE, "systim = %Tu", systim);
	if (systim < prev_systim) {
		syslog(LOG_ERROR, "systim decreases (%tu -> %tu)",
								(uint_t) prev_systim, (uint_t) systim);
	}
	prev_systim = systim;
}

void
wait_alarm1(volatile bool_t *p_flag)
{
	do {
		refer_systim();
		refer_alarm1();
	} while (!(*p_flag));
}

void
alarm2_handler(intptr_t exinf)
{
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(2);
		alarm1_flag1 = true;

		return;

		check_point(0);

	case 2:
		check_point(4);
		alarm1_flag2 = true;

		check_point(5);
		ercd = sta_alm(ALM1, TEST_TIME_CP);
		check_ercd(ercd, E_OK);

		prev_lefttim = UINT32_MAX;

		skip_check = true;

		return;

		check_point(0);

	case 3:
		check_point(7);
		alarm1_flag3 = true;

		return;

		check_point(0);

	case 4:
		check_point(11);
		alarm1_flag4 = true;

		return;

		check_point(0);

	case 5:
		check_point(16);
		alarm1_flag5 = true;

		return;

		check_point(0);

	case 6:
		check_point(21);
		alarm1_flag6 = true;

		return;

		check_point(0);

	case 7:
		check_point(25);
		alarm1_flag7 = true;

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	alarm3_count = 0;

void
alarm3_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm3_count) {
	case 1:
		check_point(24);
		ercd = stp_alm(ALM2);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RALM	ralm;

	test_start(__FILE__);

	check_point(1);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	wait_alarm1(&alarm1_flag1);

	check_point(3);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	wait_alarm1(&alarm1_flag2);

	check_point(6);
	wait_alarm1(&alarm1_flag3);

	check_point(8);
	ercd = sta_alm(ALM1, 3 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	refer_alarm1();

	check_point(9);
	ercd = dly_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim -= TEST_TIME_CP;

	refer_alarm1();

	check_point(10);
	ercd = adj_tim(TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	wait_alarm1(&alarm1_flag4);

	check_point(12);
	ercd = sta_alm(ALM1, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	refer_alarm1();

	check_point(13);
	ercd = dly_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim -= TEST_TIME_CP;

	refer_alarm1();

	check_point(14);
	ercd = adj_tim(-TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim += TEST_TIME_CP;

	check_point(15);
	wait_alarm1(&alarm1_flag5);

	check_point(17);
	ercd = sta_alm(ALM1, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM2, 3 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	refer_alarm1();

	check_point(18);
	ercd = dly_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim -= TEST_TIME_CP;

	refer_alarm1();

	check_point(19);
	ercd = stp_alm(ALM2);
	check_ercd(ercd, E_OK);

	ercd = ref_alm(ALM2, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	refer_alarm1();

	check_point(20);
	wait_alarm1(&alarm1_flag6);

	check_point(22);
	ercd = sta_alm(ALM1, 3 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM2, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM3, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	prev_lefttim = UINT32_MAX;

	check_point(23);
	wait_alarm1(&alarm1_flag7);

	check_point(26);
	ercd = ref_alm(ALM2, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_finish(27);
	check_point(0);
}

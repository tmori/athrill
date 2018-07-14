/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_ovrhdr1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		オーバランハンドラ機能のテスト(1)
 *
 * 【テストの目的】
 *
 * 【テスト項目】
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 高優先度タスク
 *	TASK3: 高優先度タスク
 *	ALM1:  アラームハンドラ
 *	OVR:   オーバランハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	ref_ovr(TASK2, &rovr)
 *		assert(rovr.ovrstat == TOVR_STP)
 *		// TASK2, TASK3に対して，残りプロセッサ時間(4 * TEST_TIME_CP)
 *		// μ秒で，オーバランハンドラを動作開始する
 *		sta_ovr(TASK2, 4 * TEST_TIME_CP)
 *		ref_ovr(TASK2, &rovr)
 *		assert(rovr.ovrstat == TOVR_STA)
 *		assert(rovr.leftotm == 4 * TEST_TIME_CP)
 *		sta_ovr(TASK3, 4 * TEST_TIME_CP)
 *		// (2 * TEST_TIME_CP)μ秒後にALM1を動作させる
 *		sta_alm(ALM1, 2 * TEST_TIME_CP) ... ALM1-1が実行開始するまで
 *		DO(hrtcnt1 = fch_hrt())
 *		act_tsk(TASK2)
 *	== TASK2（優先度：高）==
 *		// start_rでオーバランタイマが動作開始する
 *	2:	act_tsk(TASK3)
 *		DO(while(!task2_flag1))
 *	== ALM1-1（1回目）==
 *		// TASK2の実行途中でALM1が動作する
 *		// 割込み入口処理でオーバランタイマが停止する
 *	3:	rot_rdq(HIGH_PRIORITY)
 *		RETURN
 *	== TASK3（優先度：高）==
 *		// start_rでオーバランタイマが動作開始する
 *		// TASK2のオーバランハンドラ状態を確認する
 *	4:	ref_ovr(TASK2, &rovr)
 *		assert(rovr.ovrstat == TOVR_STA)
 *		assert(TEST_TIME_CP <= rovr.leftotm && rovr.leftotm <= 3 * TEST_TIME_CP)
 *		DO(while(!task3_flag1))
 *	== OVR-1（1回目）==
 *		// TASK3の残りプロセッサ時間がなくなる
 *	5:	assert(tskid == TASK3)
 *		DO(task3_flag1 = true)
 *		RETURN
 *	== TASK3（続き）==
 *	6:	DO(task2_flag1 = true)
 *		slp_tsk()
 *	== TASK2（続き）==
 *		// 割込み出口処理でオーバランタイマが動作開始する
 *	7:	DO(while(!task2_flag2))
 *	== OVR-2（2回目）==
 *		// TASK2の残りプロセッサ時間がなくなる
 *	8:	assert(tskid == TASK2)
 *		DO(task2_flag2 = true)
 *		RETURN
 *	== TASK2（続き）==
 *	9:	slp_tsk()
 *	== TASK1（続き）==
 *	10:	DO(hrtcnt2 = fch_hrt())
 *		assert(8 * TEST_TIME_CP <= hrtcnt2 - hrtcnt1 \
 *							&& hrtcnt2 - hrtcnt1 <= 12 * TEST_TIME_CP)
 *		ref_ovr(TASK2, &rovr)
 *		assert(rovr.ovrstat == TOVR_STP)
 *		sta_ovr(TASK2, 4 * TEST_TIME_CP)
 *		sta_alm(ALM1, 2 * TEST_TIME_CP) ... ALM1-2が実行開始するまで
 *		DO(hrtcnt1 = fch_hrt())
 *		wup_tsk(TASK2)
 *	== TASK2（続き）==
 *		// dispatch_rでオーバランタイマが動作開始する
 *	11:	wup_tsk(TASK3)
 *		DO(while(!task2_flag3))
 *	== ALM1-2（2回目）==
 *		// TASK2の実行途中でALM1が動作する
 *		// 割込み入口処理でオーバランタイマが停止する
 *	12:	DO(task2_flag3 = true)
 *		RETURN
 *	== TASK2（続き）==
 *	13:	rot_rdq(HIGH_PRIORITY)
 *		// dispatch_rでオーバランタイマが停止する
 *	== TASK3（続き）==
 *	14:	sta_alm(ALM1, 2 * TEST_TIME_CP) ... ALM1-3が実行開始するまで
 *		sta_ovr(TSK_SELF, 4 * TEST_TIME_CP)
 *		// sta_ovrでオーバランタイマが動作開始する
 *		// TASK2の残りプロセッサ時間を(4 * TEST_TIME_CP)μ秒に設定しなおす
 *		sta_ovr(TASK2, 4 * TEST_TIME_CP)
 *	15:	DO(while(!task3_flag2))
 *	== ALM1-3（3回目）==
 *		// TASK3の実行途中でALM1が動作する
 *		// 割込み入口処理でオーバランタイマが停止する
 *	16:	DO(task3_flag2 = true)
 *		RETURN
 *	== TASK3（続き）==
 *		// TASK3の残りプロセッサ時間を(4 * TEST_TIME_CP)μ秒に設定しなおす
 *	17:	sta_ovr(TSK_SELF, 4 * TEST_TIME_CP)
 *		// sta_ovrでオーバランタイマが停止＆動作開始する
 *		DO(while(!task3_flag3))
 *	== OVR-3（3回目）==
 *		// TASK3の残りプロセッサ時間がなくなる
 *	18:	DO(task3_flag3 = true)
 *		sta_ovr(TSK_SELF, 4 * TEST_TIME_CP) -> E_ID
 *		// TASK3の残りプロセッサ時間を(4 * TEST_TIME_CP)μ秒に設定しなおす
 *		sta_ovr(TASK3, 2 * TEST_TIME_CP)
 *		RETURN
 *	== TASK3（続き）==
 *	19:	DO(while(!task3_flag4))
 *	== OVR-4（4回目）==
 *		// TASK3の残りプロセッサ時間がなくなる
 *	20:	DO(task3_flag4 = true)
 *		rot_rdq(HIGH_PRIORITY)
 *		RETURN
 *	== TASK2（続き）==
 *		// dispatch_rでオーバランタイマが動作開始する
 *	21:	DO(while(!task2_flag4))
 *	== OVR-5（5回目）==
 *		// TASK2の残りプロセッサ時間がなくなる
 *	22:	DO(task2_flag4 = true)
 *		RETURN
 *	== TASK2（続き）==
 *	23:	slp_tsk()
 *	== TASK3（続き）==
 *	24:	slp_tsk()
 *	== TASK1（続き）==
 *	25:	DO(hrtcnt2 = fch_hrt())
 *		assert(14 * TEST_TIME_CP <= hrtcnt2 - hrtcnt1 \
 *							&& hrtcnt2 - hrtcnt1 <= 19 * TEST_TIME_CP)
 *	26:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_ovrhdr1.h"

volatile bool_t	task2_flag1 = false;
volatile bool_t	task2_flag2 = false;
volatile bool_t	task2_flag3 = false;
volatile bool_t	task2_flag4 = false;
volatile bool_t	task3_flag1 = false;
volatile bool_t	task3_flag2 = false;
volatile bool_t	task3_flag3 = false;
volatile bool_t	task3_flag4 = false;

HRTCNT		hrtcnt1, hrtcnt2;

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(3);
		ercd = rot_rdq(HIGH_PRIORITY);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(12);
		task2_flag3 = true;

		return;

		check_point(0);

	case 3:
		check_point(16);
		task3_flag2 = true;

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	overrun_count = 0;

void
overrun_handler(ID tskid, intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++overrun_count) {
	case 1:
		check_point(5);
		check_assert(tskid == TASK3);

		task3_flag1 = true;

		return;

		check_point(0);

	case 2:
		check_point(8);
		check_assert(tskid == TASK2);

		task2_flag2 = true;

		return;

		check_point(0);

	case 3:
		check_point(18);
		task3_flag3 = true;

		ercd = sta_ovr(TSK_SELF, 4 * TEST_TIME_CP);
		check_ercd(ercd, E_ID);

		ercd = sta_ovr(TASK3, 2 * TEST_TIME_CP);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 4:
		check_point(20);
		task3_flag4 = true;

		ercd = rot_rdq(HIGH_PRIORITY);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 5:
		check_point(22);
		task2_flag4 = true;

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
	T_ROVR	rovr;

	test_start(__FILE__);

	check_point(1);
	ercd = ref_ovr(TASK2, &rovr);
	check_ercd(ercd, E_OK);

	check_assert(rovr.ovrstat == TOVR_STP);

	ercd = sta_ovr(TASK2, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_ovr(TASK2, &rovr);
	check_ercd(ercd, E_OK);

	check_assert(rovr.ovrstat == TOVR_STA);

	check_assert(rovr.leftotm == 4 * TEST_TIME_CP);

	ercd = sta_ovr(TASK3, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM1, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	hrtcnt1 = fch_hrt();

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(10);
	hrtcnt2 = fch_hrt();

	check_assert(8 * TEST_TIME_CP <= hrtcnt2 - hrtcnt1 && hrtcnt2 - hrtcnt1 <= 12 * TEST_TIME_CP);

	ercd = ref_ovr(TASK2, &rovr);
	check_ercd(ercd, E_OK);

	check_assert(rovr.ovrstat == TOVR_STP);

	ercd = sta_ovr(TASK2, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM1, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	hrtcnt1 = fch_hrt();

	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(25);
	hrtcnt2 = fch_hrt();

	check_assert(14 * TEST_TIME_CP <= hrtcnt2 - hrtcnt1 && hrtcnt2 - hrtcnt1 <= 19 * TEST_TIME_CP);

	check_finish(26);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(2);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	while(!task2_flag1);

	check_point(7);
	while(!task2_flag2);

	check_point(9);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	while(!task2_flag3);

	check_point(13);
	ercd = rot_rdq(HIGH_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(21);
	while(!task2_flag4);

	check_point(23);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;
	T_ROVR	rovr;

	check_point(4);
	ercd = ref_ovr(TASK2, &rovr);
	check_ercd(ercd, E_OK);

	check_assert(rovr.ovrstat == TOVR_STA);

	check_assert(TEST_TIME_CP <= rovr.leftotm && rovr.leftotm <= 3 * TEST_TIME_CP);

	while(!task3_flag1);

	check_point(6);
	task2_flag1 = true;

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = sta_alm(ALM1, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_ovr(TSK_SELF, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = sta_ovr(TASK2, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(15);
	while(!task3_flag2);

	check_point(17);
	ercd = sta_ovr(TSK_SELF, 4 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	while(!task3_flag3);

	check_point(19);
	while(!task3_flag4);

	check_point(24);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

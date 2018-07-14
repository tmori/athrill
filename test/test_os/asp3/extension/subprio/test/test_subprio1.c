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
 *  $Id: test_subprio1.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		サブ優先度機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  サブ優先度を用いたスケジューリングに関して網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) サブ優先度の初期化
 *		(A-1) タスクの生成時にUINT_MAXに初期化されること［NGKI3681］
 *		(A-2) タスクの起動時には変更されないこと
 *	(B) サブ優先度を用いた優先順位の決定
 *		(B-1) サブ優先度が高いタスクが高い優先順位を持つこと［NGKI0560］
 *		(B-2) サブ優先度も同一のタスクの間ではFCFSになること［NGKI0561］
 *	(C) chg_sprによるサブ優先度の変更［NGKI3672］
 *		(C-1) 実行状態のタスクに対するchg_spr（タスク切換えなし）
 *		(C-2) 実行状態のタスクに対するchg_spr（タスク切換えあり）
 *		(C-3) 実行可能状態のタスクに対するchg_spr（タスク切換えなし）
 *		(C-4) 実行可能状態のタスクに対するchg_spr（タスク切換えあり）
 *		(C-5) 休止状態のタスクに対するchg_spr
 *		(C-6) 同じサブ優先度のタスクの中で最低優先順位になること［NGKI3673］
 *	(D) サブ優先度の参照
 *		(D-1) ref_tskにより，サブ優先度を参照できること［NGKINGKI3662］
 *	(E) chg_priによる優先順位の変更
 *		(E-1) 優先度上限ミューテックスをロックしていない時
 *		(E-2) 優先度上限ミューテックスをロックしている時 … 実施しない
 *			※［NGKI3682］により，サブ優先度機能と絡むことがない
 *	(F) ミューテックスによる現在優先度の変更
 *		(F-1) サービスコールの前後とも実行できる状態の時
 *		(F-2) サービスコールにより実行できる状態になった時 … 実施しない
 *			※［NGKI3682］により，サブ優先度機能と絡むことがない
 *	(G) サブ優先度を用いる／用いない優先度を対象としたrot_rdq
 *		(G-1) rot_rdqがE_ILUSEエラーになること［NGKI3663］
 *		(G-2) サブ優先度を用いない優先度を対象にした時
 *	(H) サブ優先度を用いる優先度がENA_SPRにより設定できること［NGKI3675］
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 中優先度タスク
 *	中優先度を，サブ優先度を使って優先順位を決めるように設定
 *	MTX1:  ミューテックス（TA_CEILING属性，上限は中優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *	1:	act_tsk(TASK2)
 *		ref_tsk(TASK2, &rtsk)						... (A-1)(D-1)
 *		assert(rtsk.subpri == UINT_MAX)
 *		act_tsk(TASK3)
 *		act_tsk(TASK4)
 *		act_tsk(TASK5)
 *		slp_tsk()
 *	== TASK2-1（優先度：中，1回め）==
 *	2:	rot_rdq(MID_PRIORITY) -> E_ILUSE			... (G-1)
 *		rot_rdq(LOW_PRIORITY)						... (G-2)
 *		chg_pri(TSK_SELF, TPRI_INI)
 *	== TASK3-1（優先度：中，1回め）==				... (B-2)
 *	3:	chg_pri(TSK_SELF, TPRI_INI)					... (E-1)
 *	== TASK4-1（優先度：中，1回め）==
 *	4:	chg_pri(TSK_SELF, TPRI_INI)
 *	== TASK5（優先度：中）==
 *	5:	chg_pri(TSK_SELF, TPRI_INI)
 *	== TASK2-1（続き）==
 *	6:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	7:	chg_spr(TASK2, 10)
 *		ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.subpri == 10)
 *		chg_spr(TASK3, 5)
 *		chg_spr(TASK4, 5)							... (C-6)
 *		slp_tsk()
 *	== TASK3-1（続き）==							... (B-1)
 *	8:	ext_tsk()
 *	== TASK4-1（続き）==
 *	9:	chg_spr(TSK_SELF, 5)						... (C-1)
 *	10:	chg_spr(TSK_SELF, 10)						... (C-2)
 *	== TASK2-1（続き）==
 *	11:	chg_spr(TASK4, 10)							... (C-3)
 *	12:	chg_spr(TASK4, 5)							... (C-4)
 *	== TASK4-1（続き）==
 *	13:	ext_tsk()
 *	== TASK2-1（続き）==
 *	14:	ext_tsk()
 *	== TASK5（続き）==
 *	15:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	16:	act_tsk(TASK4)
 *		chg_spr(TASK3, 10)							... (C-5)
 *		act_tsk(TASK3)
 *		ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.subpri == 10)
 *		act_tsk(TASK2)
 *		ref_tsk(TASK2, &rtsk)						... (A-2)
 *		assert(rtsk.subpri == 10)
 *		slp_tsk()
 *	== TASK4-2（2回め）==
 *	17:	ext_tsk()
 *	== TASK3-2（2回め）==
 *	18:	loc_mtx(MTX1)
 *		unl_mtx(MTX1)								... (F-1)
 *		ext_tsk()
 *	== TASK2-2（2回め）==
 *	19:	ext_tsk()
 *	== TASK5（続き）==
 *	20:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	21:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_subprio1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RTSK	rtsk;

	test_start(__FILE__);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.subpri == UINT_MAX);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = chg_spr(TASK2, 10);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.subpri == 10);

	ercd = chg_spr(TASK3, 5);
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK4, 5);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK3, 10);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.subpri == 10);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.subpri == 10);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_finish(21);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(2);
		ercd = rot_rdq(MID_PRIORITY);
		check_ercd(ercd, E_ILUSE);

		ercd = rot_rdq(LOW_PRIORITY);
		check_ercd(ercd, E_OK);

		ercd = chg_pri(TSK_SELF, TPRI_INI);
		check_ercd(ercd, E_OK);

		check_point(6);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(11);
		ercd = chg_spr(TASK4, 10);
		check_ercd(ercd, E_OK);

		check_point(12);
		ercd = chg_spr(TASK4, 5);
		check_ercd(ercd, E_OK);

		check_point(14);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(19);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task3_count = 0;

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task3_count) {
	case 1:
		check_point(3);
		ercd = chg_pri(TSK_SELF, TPRI_INI);
		check_ercd(ercd, E_OK);

		check_point(8);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(18);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = unl_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task4_count = 0;

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task4_count) {
	case 1:
		check_point(4);
		ercd = chg_pri(TSK_SELF, TPRI_INI);
		check_ercd(ercd, E_OK);

		check_point(9);
		ercd = chg_spr(TSK_SELF, 5);
		check_ercd(ercd, E_OK);

		check_point(10);
		ercd = chg_spr(TSK_SELF, 10);
		check_ercd(ercd, E_OK);

		check_point(13);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(17);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(5);
	ercd = chg_pri(TSK_SELF, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(0);
}

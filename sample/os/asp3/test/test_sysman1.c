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
 *  $Id: test_sysman1.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム状態管理機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  get_lodとget_nthを網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) get_lodの静的エラーのテスト
 *		(A-1) 非タスクコンテキストからの呼出し … 未実施
 *		(A-2) CPUロック状態からの呼出し … 未実施
 *		(A-3) tskpriが不正（小さすぎる）
 *		(A-4) tskpriが不正（大きすぎる）
 *	(B) get_lodに具体的な優先度を指定し，タスク数が返される
 *		(B-1) タスク数が0の場合
 *		(B-2) タスク数が1の場合
 *		(B-3) タスク数が3の場合
 *	(C) get_lodにTSK_SELFを指定し，タスク数が返される
 *		(C-1) 優先度が変更されていないタスクからの発行
 *		(C-2) 優先度上限ミューテックスをロックして，現在優先度が上がっ
 *		      たタスクからの発行
 *		(C-3) chg_priにより，ベース優先度が上がったタスクからの発行
 *	(D) get_nthの静的エラーのテスト
 *		(D-1) 非タスクコンテキストからの呼出し … 未実施
 *		(D-2) CPUロック状態からの呼出し … 未実施
 *		(D-3) tskpriが不正（小さすぎる）
 *		(D-4) tskpriが不正（大きすぎる）
 *	(E) get_nthに具体的な優先度を指定し，タスクIDまたはTSK_NONEが返される
 *		(E-1) タスク数が0で，0番目に対して，TSK_NONEが返る場合
 *		(E-2) タスク数が1で，0番目に対して，タスクIDが返る場合
 *		(E-3) タスク数が1で，1番目に対して，TSK_NONEが返る場合
 *		(E-4) タスク数が3で，0番目に対して，タスクIDが返る場合
 *		(E-5) タスク数が3で，2番目に対して，タスクIDが返る場合
 *		(E-6) タスク数が3で，3番目に対して，TSK_NONEが返る場合
 *	(F) get_nthにTSK_SELFを指定し，タスクIDまたはTSK_NONEが返される
 *		(F-1) 優先度が変更されていないタスクからの発行
 *		(F-2) 優先度上限ミューテックスをロックして，現在優先度が上がっ
 *		      たタスクからの発行
 *		(F-3) chg_priにより，ベース優先度が上がったタスクからの発行
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 低優先度タスク
 *	MTX1:  ミューテックス（TA_CEILING属性，上限は中優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *	1:	get_lod(TMIN_TPRI-2, &load) -> E_PAR			... (A-3)
 *		get_lod(TMAX_TPRI+1, &load) -> E_PAR			... (A-4)
 *		get_nth(TMIN_TPRI-2, 0U, &tskid) -> E_PAR		... (D-3)
 *		get_nth(TMAX_TPRI+1, 0U, &tskid) -> E_PAR		... (D-4)
 *	2:	get_lod(MID_PRIORITY, &load)					... (B-1)
 *		assert(load == 0U)
 *		get_nth(MID_PRIORITY, 0U, &tskid)				... (E-1)
 *		assert(tskid == TSK_NONE)
 *	3:	act_tsk(TASK2)
 *		get_lod(MID_PRIORITY, &load)					... (B-2)
 *		assert(load == 1U)
 *		get_nth(MID_PRIORITY, 0U, &tskid)				... (E-2)
 *		assert(tskid == TASK2)
 *		get_nth(MID_PRIORITY, 1U, &tskid)				... (E-3)
 *		assert(tskid == TSK_NONE)
 *	4:	act_tsk(TASK3)
 *		act_tsk(TASK4)
 *		get_lod(MID_PRIORITY, &load)					... (B-3)
 *		assert(load == 3U)
 *		get_nth(MID_PRIORITY, 0U, &tskid)				... (E-4)
 *		assert(tskid == TASK2)
 *		get_nth(MID_PRIORITY, 1U, &tskid)
 *		assert(tskid == TASK3)
 *		get_nth(MID_PRIORITY, 2U, &tskid)				... (E-5)
 *		assert(tskid == TASK4)
 *		get_nth(MID_PRIORITY, 3U, &tskid)				... (E-6)
 *		assert(tskid == TSK_NONE)
 *	5:	act_tsk(TASK5)
 *		get_lod(MID_PRIORITY, &load)
 *		assert(load == 3U)
 *	6:	slp_tsk()
 *	== TASK2（優先度：中）==
 *	7:	get_lod(TPRI_SELF, &load)
 *		assert(load == 3U)
 *		get_nth(TPRI_SELF, 0U, &tskid)
 *		assert(tskid == TASK2)
 *	8:	slp_tsk()
 *	== TASK3（優先度：中）==
 *	9:	get_lod(TPRI_SELF, &load)
 *		assert(load == 2U)
 *		get_nth(TPRI_SELF, 0U, &tskid)
 *		assert(tskid == TASK3)
 *	10:	slp_tsk()
 *	== TASK4（優先度：中）==
 *	11:	get_lod(TPRI_SELF, &load)
 *		assert(load == 1U)
 *		get_nth(TPRI_SELF, 0U, &tskid)
 *		assert(tskid == TASK4)
 *	12:	slp_tsk()
 *	== TASK5（優先度：低）==
 *	13:	get_lod(TPRI_SELF, &load)						... (C-1)
 *		assert(load == 1U)
 *		get_nth(TPRI_SELF, 0U, &tskid)					... (F-1)
 *		assert(tskid == TASK5)
 *	14:	loc_mtx(MTX1)
 *		get_lod(TPRI_SELF, &load)						... (C-2)
 *		assert(load == 0U)
 *		get_nth(TPRI_SELF, 0U, &tskid)					... (F-2)
 *		assert(tskid == TSK_NONE)
 *		get_lod(MID_PRIORITY, &load)
 *		assert(load == 1U)
 *		get_nth(MID_PRIORITY, 0U, &tskid)
 *		assert(tskid == TASK5)
 *	15:	wup_tsk(TASK2)
 *		get_lod(TPRI_SELF, &load)
 *		assert(load == 0U)
 *		get_lod(MID_PRIORITY, &load)
 *		assert(load == 2U)
 *	16:	unl_mtx(MTX1)
 *	== TASK2（続き）==
 *	17:	get_lod(TPRI_SELF, &load)
 *		assert(load == 1U)
 *	18:	chg_pri(TASK5, MID_PRIORITY)
 *		get_lod(TPRI_SELF, &load)
 *		assert(load == 2U)
 *		rot_rdq(TPRI_SELF)
 *	== TASK5（続き）==
 *	19:	get_lod(TPRI_SELF, &load)						... (C-3)
 *		assert(load == 2U)
 *		get_nth(TPRI_SELF, 0U, &tskid)					... (F-3)
 *		assert(tskid == TASK5)
 *	20: END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_sysman1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;
	uint_t	load;

	test_start(__FILE__);

	check_point(1);
	ercd = get_lod(TMIN_TPRI-2, &load);
	check_ercd(ercd, E_PAR);

	ercd = get_lod(TMAX_TPRI+1, &load);
	check_ercd(ercd, E_PAR);

	ercd = get_nth(TMIN_TPRI-2, 0U, &tskid);
	check_ercd(ercd, E_PAR);

	ercd = get_nth(TMAX_TPRI+1, 0U, &tskid);
	check_ercd(ercd, E_PAR);

	check_point(2);
	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 0U);

	ercd = get_nth(MID_PRIORITY, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	check_point(3);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 1U);

	ercd = get_nth(MID_PRIORITY, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK2);

	ercd = get_nth(MID_PRIORITY, 1U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	check_point(4);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 3U);

	ercd = get_nth(MID_PRIORITY, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK2);

	ercd = get_nth(MID_PRIORITY, 1U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK3);

	ercd = get_nth(MID_PRIORITY, 2U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK4);

	ercd = get_nth(MID_PRIORITY, 3U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	check_point(5);
	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 3U);

	check_point(6);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;
	uint_t	load;

	check_point(7);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 3U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK2);

	check_point(8);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 1U);

	check_point(18);
	ercd = chg_pri(TASK5, MID_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 2U);

	ercd = rot_rdq(TPRI_SELF);
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;
	uint_t	load;

	check_point(9);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 2U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK3);

	check_point(10);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;
	uint_t	load;

	check_point(11);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 1U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK4);

	check_point(12);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ID		tskid;
	ER_UINT	ercd;
	uint_t	load;

	check_point(13);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 1U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK5);

	check_point(14);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 0U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TSK_NONE);

	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 1U);

	ercd = get_nth(MID_PRIORITY, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK5);

	check_point(15);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 0U);

	ercd = get_lod(MID_PRIORITY, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 2U);

	check_point(16);
	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = get_lod(TPRI_SELF, &load);
	check_ercd(ercd, E_OK);

	check_assert(load == 2U);

	ercd = get_nth(TPRI_SELF, 0U, &tskid);
	check_ercd(ercd, E_OK);

	check_assert(tskid == TASK5);

	check_finish(20);
	check_point(0);
}

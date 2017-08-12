/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2010-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_rstr2.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		制約タスクのテスト(2)
 *
 * 【テストの目的】
 *
 *  優先度上限ミューテックスを用いて，制約タスクの優先度を変更できるこ
 *  とをテストする．
 *
 * 【テスト項目】
 *
 *	(A) 制約タスクがミューテックスをロックし，現在優先度が上がる
 *		(A-1) ディスパッチ許可状態で呼び出した場合
 *		(A-2) ディスパッチ禁止状態で，最高優先順位タスクが実行状態でな
 *		      い場合に呼び出した場合
 *	(B) 優先度が上がった制約タスクがミューテックスをロック解除し，現在
 *      優先度がベース優先度に戻る
 *		(B-1) ベース優先度よりも高い優先度のタスクが実行可能の場合に，
 *		      タスク切換えが起こることを確認する
 *		(B-2) ベース優先度と同じ優先度のタスクが実行可能の場合に，タス
 *		      ク切換えが起こらないことを確認する
 *		(B-3) ベース優先度よりも低い優先度のタスクが実行可能の場合に，
 *		      タスク切換えが起こらないことを確認する
 *	(C) 制約タスクが，自タスクのベース優先度よりも低い上限優先度を持つ
 *	    ミューテックスをロックしようとし，E_ILUSEになる
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク，制約タスク
 *	TASK3: 高優先度タスク
 *	MTX1:  ミューテックス（TA_CEILING属性，上限は高優先度）
 *	MTX2:  ミューテックス（TA_CEILING属性，上限は中優先度）
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2（優先度：中）==
 *	2:	get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *		ploc_mtx(MTX1)								... (A-1)
 *		get_pri(TSK_SELF, &tskpri)
 *		assert(tskpri == HIGH_PRIORITY)
 *	3:	ploc_mtx(MTX2)
 *		unl_mtx(MTX2)
 *	4:	act_tsk(TASK3)
 *		unl_mtx(MTX1)								... (B-1)
 *	== TASK3（優先度：高）==
 *	5:	get_pri(TASK2, &tskpri)
 *		assert(tskpri == MID_PRIORITY)
 *	6:	ploc_mtx(MTX2) -> E_ILUSE					... (C)
 *	7:	chg_pri(TSK_SELF, LOW_PRIORITY)
 *		slp_tsk()
 *	== TASK2（続き）==
 *	8:	dis_dsp()
 *		wup_tsk(TASK3)
 *		ploc_mtx(MTX1)								... (A-2)
 *		ena_dsp()
 *		unl_mtx(MTX1)								... (B-3)
 *		chg_pri(TASK3, TPRI_INI)
 *	== TASK3（続き）==
 *	9:	ext_tsk()
 *	== TASK2（続き）==
 *	10:	ploc_mtx(MTX1)								... (A-1)
 *		wup_tsk(TASK1)
 *		unl_mtx(MTX1)								... (B-2)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	11: END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_rstr2.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_finish(11);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(2);
	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = get_pri(TSK_SELF, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == HIGH_PRIORITY);

	check_point(3);
	ercd = ploc_mtx(MTX2);
	check_ercd(ercd, E_OK);

	ercd = unl_mtx(MTX2);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = chg_pri(TASK3, TPRI_INI);
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = ploc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	ercd = unl_mtx(MTX1);
	check_ercd(ercd, E_OK);

	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;
	PRI		tskpri;

	check_point(5);
	ercd = get_pri(TASK2, &tskpri);
	check_ercd(ercd, E_OK);

	check_assert(tskpri == MID_PRIORITY);

	check_point(6);
	ercd = ploc_mtx(MTX2);
	check_ercd(ercd, E_ILUSE);

	check_point(7);
	ercd = chg_pri(TSK_SELF, LOW_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

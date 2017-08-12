/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_cpuexc6.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		CPU例外処理のテスト(6)
 *
 * 【テストの目的】
 *
 *  タスクコンテキスト，割込ロック解除状態，CPUロック解除状態，割込み優
 *  先度マスク全解除状態，ディスパッチ許可状態で発生したCPU例外における
 *  システム状態のテスト．タスク切換えによりリカバリできることもテスト
 *  する．
 *
 * 【テスト項目】
 *
 *  いずれも，タスクコンテキスト，割込ロック解除状態，CPUロック解除状態，
 *  割込み優先度マスク全解除状態，ディスパッチ許可状態で発生したCPU例外
 *  において，
 *
 *	(A) CPU例外ハンドラ実行開始時にCPUロックフラグが変化しないこと
 *	(B) CPU例外ハンドラ実行開始時に割込み優先度マスクが変化しないこと
 *		！CPU例外ハンドラ中で割込み優先度マスクを読めないため，テストで
 *		　きない．
 *	(C) CPU例外ハンドラ実行開始時にディスパッチ禁止フラグが変化しないこと
 *	(D) CPU例外ハンドラリターン時にCPUロックフラグが元に戻ること
 *	(E) CPU例外ハンドラリターン時に割込み優先度マスクが元に戻ること
 *	(F) CPU例外ハンドラリターン時にディスパッチ禁止フラグが変化しないこと
 *	(G) xsns_dpnがfalseを返すこと
 *	(H) タスク切換えによるリカバリができること
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 高優先度タスク
 *	CPUEXC: CPU例外ハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1-1（1回目）==
 *	1:	state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *		DO(RAISE_CPU_EXCEPTION)
 *	== CPUEXC ==
 *	2:	state(true, false, false, true, false)				... (A)(C)
 *		assert(xsns_dpn(p_excinf) == false)					... (G)
 *  3:	act_tsk(TASK2)
 *		loc_cpu()	... あえてCPUロック状態にしてみる
 *  	RETURN
 *	== TASK2 ==
 *	4:	state(false, false, false, false, false)			... (D)(E)(F)
 *		ipm(TIPM_ENAALL)
 *	5:	ter_tsk(TASK1)										... (H)
 *	6:	act_tsk(TASK1)										... (H)
 *	7:	ext_tsk()
 *	== TASK1-2（2回目）==
 *	8:	state(false, false, false, false, false)			... (H)
 *		ipm(TIPM_ENAALL)
 *	9:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_cpuexc.h"

void
alarm1_handler(intptr_t exinf)
{
	check_point(0);
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
cpuexc_handler(void *p_excinf)
{
	ER_UINT	ercd;

	check_point(2);
	check_state(true, false, false, true, false);

	check_assert(xsns_dpn(p_excinf) == false);

	check_point(3);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	return;

	check_point(0);
}

static uint_t	task1_count = 0;

void
task1(intptr_t exinf)
{

	switch (++task1_count) {
	case 1:
		test_start(__FILE__);

		check_point(1);
		check_state(false, false, false, false, false);

		check_ipm(TIPM_ENAALL);

		RAISE_CPU_EXCEPTION;

		check_point(0);

	case 2:
		check_point(8);
		check_state(false, false, false, false, false);

		check_ipm(TIPM_ENAALL);

		check_finish(9);
		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(4);
	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	check_point(5);
	ercd = ter_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = act_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

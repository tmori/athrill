/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
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
 *  $Id: test_cpuexc1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		CPU例外処理のテスト(1)
 *
 * 【テストの目的】
 *
 *  非タスクコンテキスト，割込ロック状態で発生したCPU例外におけるシステ
 *  ム状態のテスト．割込み優先度マスク全解除状態，ディスパッチ禁止状態
 *  でテストする．
 *
 * 【テスト項目】
 *
 *  いずれも，非タスクコンテキスト，割込ロック状態で発生したCPU例外にお
 *  いて，
 *
 *	(A) CPU例外ハンドラ実行開始時にCPUロックフラグが変化しないこと
 *		！割込みロック状態で発生したCPU例外では，システム状態を正しく読
 *		　めることが保証されないため，テストできない．
 *	(B) CPU例外ハンドラ実行開始時に割込み優先度マスクが変化しないこと
 *		！割込みロック状態で発生したCPU例外では，システム状態を正しく読
 *		　めることが保証されないため，テストできない．
 *	(C) CPU例外ハンドラ実行開始時にディスパッチ禁止フラグが変化しないこと
 *		！割込みロック状態で発生したCPU例外では，システム状態を正しく読
 *		　めることが保証されないため，テストできない．
 *	(D) CPU例外ハンドラリターン時にCPUロックフラグが元に戻ること
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(E) CPU例外ハンドラリターン時に割込み優先度マスクが元に戻ること
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(F) CPU例外ハンドラリターン時にディスパッチ禁止フラグが変化しないこと
 *		！CPU例外ハンドラからリターンできる場合のみテストする．
 *	(G) xsns_dpnがtrueを返すこと
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	CPUEXC: CPU例外ハンドラ
 *	ALM1: アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1 ==
 *	1:	state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *		dis_dsp()
 *	2:	state(false, false, true, true, false)
 *		ipm(TIPM_ENAALL)
 *		sta_alm(ALM1, TEST_TIME_PROC) ... ALM1が実行開始するまで
 *		DO(while(!alm1_flag))
 *	== ALM1 ==
 *	3:	state(true, false, true, true, false)
 *		DO(SIL_LOC_INT())
 *		DO(RAISE_CPU_EXCEPTION)
 *	== CPUEXC ==
 *	4:	assert(xsns_dpn(p_excinf) == true)					... (G)
 *	5:	HOOK(HOOK_POINT(%d))
 *				... CPU例外ハンドラからリターンできない場合はここで終了
 *		RETURN
 *	== ALM1（続き）==
 *	6:	DO(SIL_UNL_INT())
 *	7:	state(true, false, true, true, false)				... (D)(F)
 *		DO(alm1_flag = true)
 *		RETURN
 *	== TASK1（続き）==
 *	8:	state(false, false, true, true, false)				... (E)
 *		ipm(TIPM_ENAALL)
 *	9:	END
 */

#include <sil.h>
#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_cpuexc.h"

volatile bool_t	alm1_flag = false;

void
task2(intptr_t exinf)
{
	check_point(0);
}

#ifdef PREPARE_RETURN_CPUEXC
#define HOOK_POINT(count)	do {							\
								PREPARE_RETURN_CPUEXC;		\
								check_point(count);			\
							} while (false)
#else /* PREPARE_RETURN_CPUEXC */
#define HOOK_POINT(count)	check_finish(count)
#endif /* PREPARE_RETURN_CPUEXC */

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	SIL_PRE_LOC;

	check_point(3);
	check_state(true, false, true, true, false);

	SIL_LOC_INT();

	RAISE_CPU_EXCEPTION;

	check_point(6);
	SIL_UNL_INT();

	check_point(7);
	check_state(true, false, true, true, false);

	alm1_flag = true;

	return;

	check_point(0);
}

void
cpuexc_handler(void *p_excinf)
{

	check_point(4);
	check_assert(xsns_dpn(p_excinf) == true);

	HOOK_POINT(5);
	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_point(2);
	check_state(false, false, true, true, false);

	check_ipm(TIPM_ENAALL);

	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	while(!alm1_flag);

	check_point(8);
	check_state(false, false, true, true, false);

	check_ipm(TIPM_ENAALL);

	check_finish(9);
	check_point(0);
}

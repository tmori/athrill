/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_sysstat1.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム状態に関するテスト(1)
 *
 * 【テストの目的】
 *
 *  CPUロック状態とCPUロック解除状態，割込み優先度マスク全解除状態とそ
 *  うでない状態，ディスパッチ禁止状態とディスパッチ許可状態，タスク終
 *  了禁止状態とタスク終了許可状態の間の状態遷移をテストする．また，タ
 *  スクの実行開始時と終了時のシステム状態の変化についてもテストする．
 *
 * 【テスト項目】
 *
 *	(A) 通常状態（CPUロック解除状態，割込み優先度マスク全解除状態，ディ
 *		スパッチ許可状態，タスク終了許可状態）からの状態遷移のテスト
 *		(A-1) CPUロック状態への遷移
 *		(A-2) 割込み優先度マスク全解除でない状態への遷移
 *		(A-3) ディスパッチ禁止状態への遷移
 *		(A-4) タスク終了禁止状態への遷移
 *	(B) CPUロック状態からの状態遷移のテスト
 *		(B-1) CPUロック状態への遷移 … 何も起こらない
 *		(B-2) 割込み優先度マスク全解除でない状態への遷移 … エラーに
 *		(B-3) ディスパッチ禁止状態への遷移 … エラーに
 *		(B-4) タスク終了禁止状態への遷移 … エラーに
 *		(B-5) CPUロック解除状態への遷移
 *	(C) 割込み優先度マスク全解除でない状態からの状態遷移のテスト
 *		(C-1) CPUロック状態への遷移
 *		(C-2) 割込み優先度マスク全解除でない状態への遷移 … 何も起こらない
 *		(C-3) ディスパッチ禁止状態への遷移
 *		(C-4) タスク終了禁止状態への遷移
 *		(C-5) 割込み優先度マスク全解除状態への遷移
 *	(D) ディスパッチ禁止状態からの状態遷移のテスト
 *		(D-1) CPUロック状態への遷移
 *		(D-2) 割込み優先度マスク全解除でない状態への遷移
 *		(D-3) ディスパッチ禁止状態への遷移 … 何も起こらない
 *		(D-4) タスク終了禁止状態への遷移
 *		(D-5) ディスパッチ許可状態への遷移
 *	(E) タスク終了禁止状態からの状態遷移のテスト
 *		(E-1) CPUロック状態への遷移
 *		(E-2) 割込み優先度マスク全解除でない状態への遷移
 *		(E-3) ディスパッチ禁止状態への遷移
 *		(E-4) タスク終了禁止状態への遷移 … 何も起こらない
 *		(E-5) タスク終了許可状態への遷移
 *	(F) タスクの終了・起動により通常状態に戻ることのテスト
 *		(F-1) 別のタスクの起動
 *		(F-2) 同じタスクの起動
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 中優先度タスク
 *
 * 【テストシーケンス】
 *
 *	== TASK1 ==
 *	1:	state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *	2:	loc_cpu()												... (A-1)
 *		state(false, true, false, true, false)
 *	3:	loc_cpu()												... (B-1)
 *		state(false, true, false, true, false)
 *	4:	chg_ipm(TMAX_INTPRI) -> E_CTX							... (B-2)
 *		dis_dsp() -> E_CTX										... (B-3)
 *		dis_ter() -> E_CTX										... (B-4)
 *		state(false, true, false, true, false)
 *	5:	unl_cpu()												... (B-5)
 *		state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *	6:	chg_ipm(TMAX_INTPRI)									... (A-2)
 *		state(false, false, false, true, false)
 *		ipm(TMAX_INTPRI)
 *	7:	loc_cpu()												... (C-1)
 *		state(false, true, false, true, false)
 *		chg_ipm(TIPM_ENAALL) -> E_CTX
 *		dis_dsp() -> E_CTX
 *		dis_ter() -> E_CTX
 *		unl_cpu()
 *		state(false, false, false, true, false)
 *		ipm(TMAX_INTPRI)
 *	8:	chg_ipm(TMAX_INTPRI)									... (C-2)
 *		state(false, false, false, true, false)
 *		ipm(TMAX_INTPRI)
 *	9:	dis_dsp()												... (C-3)
 *		state(false, false, true, true, false)
 *		ipm(TMAX_INTPRI)
 *	10:	dis_ter()												... (C-4)
 *		state(false, false, true, true, true)
 *		ipm(TMAX_INTPRI)
 *	11:	chg_ipm(TIPM_ENAALL)									... (C-5)
 *		state(false, false, true, true, true)
 *		ipm(TIPM_ENAALL)
 *	12:	ena_dsp()
 *		state(false, false, false, false, true)
 *		ipm(TIPM_ENAALL)
 *	13:	ena_ter()
 *		state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *	14:	dis_dsp()												... (A-3)
 *		state(false, false, true, true, false)
 *		ipm(TIPM_ENAALL)
 *	15:	loc_cpu()												... (D-1)
 *		state(false, true, true, true, false)
 *		unl_cpu()
 *		state(false, false, true, true, false)
 *		ipm(TIPM_ENAALL)
 *	16:	chg_ipm(TMAX_INTPRI)									... (D-2)
 *		state(false, false, true, true, false)
 *		ipm(TMAX_INTPRI)
 *	17:	dis_dsp()												... (D-3)
 *		state(false, false, true, true, false)
 *		ipm(TMAX_INTPRI)
 *	18:	dis_ter()												... (D-4)
 *		state(false, false, true, true, true)
 *		ipm(TMAX_INTPRI)
 *	19:	ena_dsp()												... (D-5)
 *		state(false, false, false, true, true)
 *		ipm(TMAX_INTPRI)
 *	20:	chg_ipm(TIPM_ENAALL)
 *		state(false, false, false, false, true)
 *		ipm(TIPM_ENAALL)
 *	21:	ena_ter()												... (E-5)
 *		state(false, false, false, false, false)
 *		ipm(TIPM_ENAALL)
 *	22:	dis_ter()												... (A-4)
 *		state(false, false, false, false, true)
 *		ipm(TIPM_ENAALL)
 *	23:	loc_cpu()												... (E-1)
 *		state(false, true, false, true, true)
 *		unl_cpu()
 *		state(false, false, false, false, true)
 *		ipm(TIPM_ENAALL)
 *	24:	chg_ipm(TMAX_INTPRI)									... (E-2)
 *		state(false, false, false, true, true)
 *		ipm(TMAX_INTPRI)
 *	25:	dis_dsp()												... (E-3)
 *		state(false, false, true, true, true)
 *		ipm(TMAX_INTPRI)
 *	26:	dis_ter()												... (E-4)
 *		state(false, false, true, true, true)
 *		ipm(TMAX_INTPRI)
 *	27:	act_tsk(TASK2)
 *		dis_ter()
 *		loc_cpu()
 *		state(false, true, true, true, true)
 *		ext_tsk()
 *	== TASK2-1（1回目）==
 *	28:	state(false, false, false, false, false)				... (F-1)
 *		ipm(TIPM_ENAALL)
 *	29:	act_tsk(TASK2)
 *		chg_ipm(TMAX_INTPRI)
 *		dis_dsp()
 *		dis_ter()
 *		loc_cpu()
 *		state(false, true, true, true, true)
 *		ext_tsk()
 *	== TASK2-2（2回目）==
 *	30:	state(false, false, false, false, false)				... (F-2)
 *		ipm(TIPM_ENAALL)
 *	31:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_sysstat1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	check_point(2);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, false, true, false);

	check_point(3);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, false, true, false);

	check_point(4);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_CTX);

	ercd = dis_dsp();
	check_ercd(ercd, E_CTX);

	ercd = dis_ter();
	check_ercd(ercd, E_CTX);

	check_state(false, true, false, true, false);

	check_point(5);
	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	check_point(6);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	check_state(false, false, false, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(7);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, false, true, false);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_CTX);

	ercd = dis_dsp();
	check_ercd(ercd, E_CTX);

	ercd = dis_ter();
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(8);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	check_state(false, false, false, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(9);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(10);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(11);
	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, true);

	check_ipm(TIPM_ENAALL);

	check_point(12);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, true);

	check_ipm(TIPM_ENAALL);

	check_point(13);
	ercd = ena_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	check_point(14);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, false);

	check_ipm(TIPM_ENAALL);

	check_point(15);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, true, true, false);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, false);

	check_ipm(TIPM_ENAALL);

	check_point(16);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(17);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, false);

	check_ipm(TMAX_INTPRI);

	check_point(18);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(19);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(20);
	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, true);

	check_ipm(TIPM_ENAALL);

	check_point(21);
	ercd = ena_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, false);

	check_ipm(TIPM_ENAALL);

	check_point(22);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, true);

	check_ipm(TIPM_ENAALL);

	check_point(23);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, false, true, true);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, false, false, false, true);

	check_ipm(TIPM_ENAALL);

	check_point(24);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	check_state(false, false, false, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(25);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(26);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_state(false, false, true, true, true);

	check_ipm(TMAX_INTPRI);

	check_point(27);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_state(false, true, true, true, true);

	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(28);
		check_state(false, false, false, false, false);

		check_ipm(TIPM_ENAALL);

		check_point(29);
		ercd = act_tsk(TASK2);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_state(false, true, true, true, true);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(30);
		check_state(false, false, false, false, false);

		check_ipm(TIPM_ENAALL);

		check_finish(31);
		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

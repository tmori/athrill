/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_tex5.c 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/* 
 *		タスク例外処理に関するテスト(5)
 *
 * 【テストの目的】
 *
 *  カーネルドメインに属するタスクに対して，CPUロック状態の解除（unl_cpu）
 *  と拡張サービスコールルーチンからのリターンによるタスク例外処理ルー
 *  チンの実行開始処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) CPUロック状態の解除によるタスク例外処理ルーチンの実行開始
 *		(A-1) 7条件が揃って，タスク例外処理ルーチンが実行開始
 *	(B) CPUロック状態の解除時点で揃わない条件があり，タスク例外処理ルー
 *		チンが実行開始されない
 *		(B-1) 対象タスクがタスク例外処理禁止状態
 *		(B-2) 対象タスクの保留例外要因が0
 *		(B-3) 対象タスクが実行状態でない
 *		(B-4) 割込み優先度マスク全解除状態でない
 *		(B-5) 対象タスクがタスク例外処理マスク状態（拡張サービスコール
 *			  を実行している間）
 *	(C) 拡張サービスコールルーチンからのリターンによるタスク例外処理ルー
 *		チンの実行開始
 *		(C-1) 7条件が揃って，タスク例外処理ルーチンが実行開始
 *	(D) 拡張サービスコールルーチンからのリターン時点で揃わない条件があ
 *		り，タスク例外処理ルーチンが実行開始されない
 *		(D-1) 対象タスクがタスク例外処理禁止状態
 *		(D-2) 対象タスクの保留例外要因が0
 *		(D-3) 対象タスクが実行状態でない
 *		(D-4) 非タスクコンテキストが実行されている
 *		(D-5) 割込み優先度マスク全解除状態でない
 *		(D-6) CPUロック状態である
 *		(D-7) 対象タスクがタスク例外処理マスク状態（拡張サービスコール
 *			  を実行している間）
 *
 * 【使用リソース】
 *
 *	TASK1: メインのタスク．CPUロック状態の解除を行うタスク
 *	TASK2: 対象タスクが実行状態でない条件をテストするためのタスク
 *	CPUEXC: CPU例外ハンドラ
 *	EXTSVC1: 拡張サービスコール1
 *	EXTSVC2: 拡張サービスコール2
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-1（1回目）==
 *	2:	ena_tex()
 *		ras_tex(TSK_SELF, 0x0001)
 *		chg_ipm(TIPM_ENAALL)
 *		loc_cpu()
 *	3:	unl_cpu()								... (B-5)
 *	4:	loc_cpu()
 *	5:	RETURN(E_OK)							... (D-6)
 *	== TASK1（続き）==
 *	6:	unl_cpu()								... (A-1)
 *	== TASK1-TEX-1（1回目）==
 *	7:	assert(texptn == 0x0001)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	8:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-2（2回目）==
 *	9:	ena_tex()
 *		chg_ipm(TIPM_ENAALL)
 *		loc_cpu()
 *		RETURN(E_OK)
 *	== TASK1（続き）==
 *	10:	unl_cpu()								... (B-2)
 *	11:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-3（3回目）==
 *	12:	dis_tex()
 *		ras_tex(TSK_SELF, 0x0002)
 *		chg_ipm(TIPM_ENAALL)
 *		loc_cpu()
 *		RETURN(E_OK)
 *	== TASK1（続き）==
 *	13:	unl_cpu()								... (B-1)
 *	14:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-4（4回目）==
 *	15:	ena_tex()
 *		ras_tex(TSK_SELF, 0x0004)
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		RETURN(E_OK)
 *	== TASK1（続き）==
 *	16:	unl_cpu()								... (B-4)
 *		chg_ipm(TIPM_ENAALL)
 *	== TASK1-TEX-2（2回目）==
 *	17:	assert(texptn == 0x0006)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	18:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2（優先度：11）==
 *	19:	ena_tex()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	20:	ras_tex(TASK2, 0x0008)
 *		loc_cpu()
 *	21:	unl_cpu()								... (B-3)
 *	22:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-5（5回目）==
 *	23:	RETURN(E_OK)							... (D-3)
 *	== TASK1（続き）==
 *	24:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-6（6回目）==
 *	25:	ena_tex()
 *		ras_tex(TSK_SELF, 0x0010)
 *		chg_ipm(TIPM_ENAALL)
 *	26:	RETURN(E_OK)							... (C-1)(D-3)
 *	== TASK1-TEX-3（3回目）==
 *	27:	assert(texptn == 0x0010)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	28:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-7（7回目）==
 *	29:	ena_tex()
 *		chg_ipm(TIPM_ENAALL)
 *	30:	RETURN(E_OK)							... (D-2)
 *	== TASK1（続き）==
 *	31:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-8（8回目）==
 *	32:	dis_tex()
 *		ras_tex(TSK_SELF, 0x0020)
 *		chg_ipm(TIPM_ENAALL)
 *	33:	RETURN(E_OK)							... (D-1)
 *	== TASK1（続き）==
 *	34:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-9（9回目）==
 *	35:	ena_tex()
 *		ras_tex(TSK_SELF, 0x0040)
 *		chg_ipm(TMAX_INTPRI)
 *	36:	RETURN(E_OK)							... (D-5)
 *	== TASK1（続き）==
 *	37:	cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0)
 *	== EXTSVC2-1（1回目）==
 *	38:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-10（10回目）==
 *	39:	ena_tex()
 *		ras_tex(TSK_SELF, 0x0080)
 *		chg_ipm(TIPM_ENAALL)
 *	40:	RETURN(E_OK)							... (D-7)
 *	== EXTSVC2-1（続き）==
 *	41:	RETURN(E_OK)
 *	== TASK1-TEX-4（4回目）==
 *	42:	assert(texptn == 0x00e0)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *		RETURN
 *	== TASK1（続き）==
 *	43:	ena_tex()
 *		chg_ipm(TIPM_ENAALL)
 *		DO(RAISE_CPU_EXCEPTION)
 *	== CPUEXC ==
 *	44:	cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)
 *	== EXTSVC1-11（11回目）==
 *	45:	iras_tex(TASK1, 0x0100)
 *	46:	RETURN(E_OK)							... (D-4)
 *	== CPUEXC（続き）==
 *	47:	RETURN
 *	== TASK1-TEX-5（5回目）==
 *	48:	assert(texptn == 0x0100)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	49:	ext_tsk()
 *	== TASK2-TEX ==
 *	50:	assert(texptn == 0x0008)
 *		state(false, false, TIPM_ENAALL, false, false, true)
 *	51:	END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex5.h"

void
cpuexc_handler(void *p_excinf)
{
	ER		ercd;

	check_point(44);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(47);

	return;

	check_point(0);
}

static uint_t	extsvc1_count = 0;

ER_UINT
extsvc1_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	ER		ercd;

	switch (++extsvc1_count) {
	case 1:
		check_point(2);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0001);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_point(3);

		ercd = unl_cpu();
		check_ercd(ercd, E_OK);

		check_point(4);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		check_point(5);

		return(E_OK);

		check_point(0);

	case 2:
		check_point(9);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		return(E_OK);

		check_point(0);

	case 3:
		check_point(12);

		ercd = dis_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0002);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		return(E_OK);

		check_point(0);

	case 4:
		check_point(15);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0004);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		return(E_OK);

		check_point(0);

	case 5:
		check_point(23);

		return(E_OK);

		check_point(0);

	case 6:
		check_point(25);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0010);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		check_point(26);

		return(E_OK);

		check_point(0);

	case 7:
		check_point(29);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		check_point(30);

		return(E_OK);

		check_point(0);

	case 8:
		check_point(32);

		ercd = dis_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0020);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		check_point(33);

		return(E_OK);

		check_point(0);

	case 9:
		check_point(35);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0040);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		check_point(36);

		return(E_OK);

		check_point(0);

	case 10:
		check_point(39);

		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		ercd = ras_tex(TSK_SELF, 0x0080);
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		check_point(40);

		return(E_OK);

		check_point(0);

	case 11:
		check_point(45);

		ercd = iras_tex(TASK1, 0x0100);
		check_ercd(ercd, E_OK);

		check_point(46);

		return(E_OK);

		check_point(0);
	}
	check_point(0);
	return(E_SYS);
}

static uint_t	extsvc2_count = 0;

ER_UINT
extsvc2_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	ER		ercd;

	switch (++extsvc2_count) {
	case 1:
		check_point(38);

		ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
		check_ercd(ercd, E_OK);

		check_point(41);

		return(E_OK);

		check_point(0);
	}
	check_point(0);
	return(E_SYS);
}

void
task1(intptr_t exinf)
{
	ER		ercd;

	check_point(1);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(6);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(8);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(10);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(11);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(13);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(14);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(16);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	check_point(18);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(20);

	ercd = ras_tex(TASK2, 0x0008);
	check_ercd(ercd, E_OK);

	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	check_point(21);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(22);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(24);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(28);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(31);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(34);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(37);

	ercd = cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(43);

	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	RAISE_CPU_EXCEPTION;

	check_point(0);
}

static uint_t	tex_task1_count = 0;

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	switch (++tex_task1_count) {
	case 1:
		check_point(7);
		check_assert(texptn == 0x0001);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 2:
		check_point(17);
		check_assert(texptn == 0x0006);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 3:
		check_point(27);
		check_assert(texptn == 0x0010);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 4:
		check_point(42);
		check_assert(texptn == 0x00e0);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		return;

		check_point(0);

	case 5:
		check_point(48);
		check_assert(texptn == 0x0100);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		check_point(49);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);
	}
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	check_point(19);

	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
tex_task2(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	check_point(50);
	check_assert(texptn == 0x0008);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	check_finish(51);

	check_point(0);
}

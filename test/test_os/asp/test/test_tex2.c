/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2007-2009 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: test_tex2.c 1577 2009-05-31 14:30:51Z ertl-hiro $
 */

/* 
 *		タスク例外処理に関するテスト(2)
 *
 * 【テストの目的】
 *
 *  割込みハンドラ（アラームハンドラ）およびCPU例外ハンドラからタスク例
 *  外処理ルーチンを起動する処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) iras_texのエラー検出
 *		(A-1) 対象タスクが休止状態
 *		(A-2) 対象タスクのタスク例外処理ルーチンが定義されていない
 *	(B) 割込みハンドラから呼ばれたiras_texの正常処理
 *		(B-1) 対象タスクが実行状態のタスクかつタスク例外処理許可
 *		(B-2) 対象タスクが実行状態のタスクでない
 *		(B-3) 対象タスクが実行状態のタスクだがタスク例外処理禁止
 *	(C) CPU例外ハンドラから呼ばれたiras_texの正常処理
 *		(C-1) 対象タスクが実行状態のタスクかつタスク例外処理許可
 *		(C-2) 対象タスクが実行状態のタスクでない
 *		(C-3) 対象タスクが実行状態のタスクだがタスク例外処理禁止
 *				→ 実施しない（ターゲット非依存に実現できない）
 *	(D) 割込みハンドラの出口処理による起動
 *		(D-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(E) CPU例外ハンドラの出口処理による起動
 *		(E-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(F) sns_texで実行状態のタスクがない
 *
 * 【使用リソース】
 *
 *	TASK1: メインのタスク．実行状態のタスクに対してタスク例外処理を要求す
 *		   る対象タスク
 *	TASK2: 実行状態でないタスクに対してタスク例外処理を要求する対象タスク
 *	TASK3: タスク例外処理ルーチンが定義されていないタスク
 *	TASK4: 休止状態のタスク
 *	ALM1:  アラームハンドラ1
 *	ALM2:  アラームハンドラ2
 *	ALM3:  アラームハンドラ3
 *	CPUEXC1: CPU例外ハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	初期状態のチェック
 *		ref_tex(TSK_SELF, &rtex)
 *	2:	sta_alm(ALM1, 1U)
 *		アラームハンドラ1の実行を待つ
 *	== ALM1 ==
 *	3:	初期状態のチェック
 *		iras_tex(TASK3, 0x0001)		... (A-2)
 *		iras_tex(TASK4, 0x0001)		... (A-1)
 *		iras_tex(TASK2, 0x0001)		... (B-2)
 *		iras_tex(TASK1, 0x0001)		... (B-3)
 *		リターン
 *	== TASK1（続き）==
 *	4:	ena_tex()
 *	== TASK1タスク例外処理ルーチン（1回目）==
 *	5:	初期状態のチェック
 *	6:	dis_dsp() ... 4つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		loc_cpu()
 *		リターン
 *	== TASK1（続き）==
 *	7:	戻ってきた状態のチェック
 *		ref_tex(TSK_SELF, &rtex)
 *	8:	sta_alm(ALM2, 1U)
 *		アラームハンドラ2の実行を待つ
 *	== ALM2 ==
 *	9:	初期状態のチェック
 *		iras_tex(TASK1, 0x0002)		... (B-1)
 *		リターン					... (D-1)
 *	== TASK1タスク例外処理ルーチン（2回目）==
 *	10:	初期状態のチェック
 *		リターン
 *	== TASK1（続き）==
 *	11:	sus_tsk(TASK2)
 *		sus_tsk(TASK3)
 *	12:	sta_alm(ALM3, 10U)
 *	13:	dly_tsk(50U)
 *	== ALM3 ==
 *	14:	初期状態のチェック
 *		［sns_tex()を含む］			... (F)
 *		iget_tid(&tskid)
 *		iras_tex(TASK1, 0x0004)		... (B-2)
 *		リターン
 *	== TASK1タスク例外処理ルーチン（3回目）==
 *	15:	初期状態のチェック
 *		リターン
 *	== TASK1（続き）==
 *	16:	rsm_tsk(TASK2)
 *		rsm_tsk(TASK3)
 *		dis_dsp()
 *	17:	RAISE_CPU_EXCEPTION
 *	== CPUEXC1 ==
 *	18:	初期状態のチェック
 *		xsns_xpn(p_excinf)
 *		iras_tex(TASK3, 0x0010)		... (A-2)
 *		iras_tex(TASK4, 0x0010)		... (A-1)
 *		iras_tex(TASK2, 0x0010)		... (C-2)
 *		iras_tex(TASK1, 0x0010)		... (C-1)
 *		リターン					... (E-1)
 *	== TASK1タスク例外処理ルーチン（4回目）==
 *	19:	初期状態のチェック
 *	20:	ext_tsk()
 *	== TASK2（優先度：10）==
 *	21:	初期状態のチェック
 *	22:	ena_tex()
 *	== TASK2タスク例外処理ルーチン ==
 *	23:	初期状態のチェック
 *		リターン
 *	== TASK2（続き）==
 *	24:	sus_tsk(TASK3)
 *	25:	テスト終了
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex2.h"

volatile bool_t	alm1_flag = false;
volatile bool_t	alm2_flag = false;
volatile bool_t	alm3_flag = false;

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	switch (texptn) {
	case 0x0001:
		check_point(5);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		/*
		 *  ディスパッチ禁止，割込み優先度マスク変更，タスク例外処理許可，
		 *  CPUロック
		 */
		check_point(6);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = ena_tex();
		check_ercd(ercd, E_OK);
		ercd = loc_cpu();
		check_ercd(ercd, E_OK);
		check_state(false, true, TMAX_INTPRI, true, true, false);
		break;

	case 0x0002:
		check_point(10);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		break;

	case 0x0004:
		check_point(15);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		break;

	case 0x0010:
		check_point(19);
		check_state(false, false, TIPM_ENAALL, true, true, true);

		/*
		 *  タスク終了
		 */
		check_point(20);
		ercd = ext_tsk();
		check_point(0);
		break;

	default:
		check_point(0);
		break;
	}
}

void
task1(intptr_t exinf)
{
	ER		ercd;
	T_RTEX	rtex;

	/*
	 *  初期状態のチェック
	 */
	check_point(1);
	check_state(false, false, TIPM_ENAALL, false, false, true);
	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_DIS) != 0);
	check_assert(rtex.pndptn == 0);

	/*
	 *  アラームハンドラ1の動作開始
	 */
	check_point(2);
	ercd = sta_alm(ALM1, 1U);
	check_ercd(ercd, E_OK);

	/*
	 *  アラームハンドラ1の実行を待つ
	 */
	while (!(alm1_flag));

	/*
	 *  タスク例外処理を許可
	 */
	check_point(4);
	ercd = ena_tex();
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク例外処理からのリターンにより元の状態に戻っていることを
	 *  チェック
	 */
	check_point(7);
	check_state(false, false, TIPM_ENAALL, false, false, false);
	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_ENA) != 0);
	check_assert(rtex.pndptn == 0);

	/*
	 *  アラームハンドラ2の動作開始
	 */
	check_point(8);
	ercd = sta_alm(ALM2, 1U);
	check_ercd(ercd, E_OK);

	/*
	 *  アラームハンドラ2の実行を待つ
	 */
	while (!(alm2_flag));

	/*
	 *  TASK2とTASK3を止める．
	 */
	check_point(11);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);
	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_OK);

	/*
	 *  アラームハンドラ3の動作開始
	 */
	check_point(12);
	ercd = sta_alm(ALM3, 10U);
	check_ercd(ercd, E_OK);

	/*
	 *  実行遅延
	 */
	check_point(13);
	ercd = dly_tsk(50U);
	/* アラームハンドラ3が動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  TASK2とTASK3を再開する．
	 */
	check_point(16);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);
	ercd = rsm_tsk(TASK3);
	check_ercd(ercd, E_OK);

	/*
	 *  ディスパッチ禁止
	 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	/*
	 *  CPU例外を発生させる
	 */
	check_point(17);
	RAISE_CPU_EXCEPTION;

	check_point(0);
}

void
tex_task2(TEXPTN texptn, intptr_t exinf)
{
	switch (texptn) {
	case 0x0011:
		check_point(23);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		break;

	default:
		check_point(0);
		break;
	}
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(21);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  タスク例外処理を許可
	 */
	check_point(22);
	ercd = ena_tex();
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  TASK3を止める．
	 */
	check_point(24);
	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_OK);

	/*
	 *  テスト終了
	 */
	check_finish(25);
}

void
task3(intptr_t exinf)
{
	check_point(0);
}

void
task4(intptr_t exinf)
{
	check_point(0);
}

void
tex_task4(TEXPTN texptn, intptr_t exinf)
{
	check_point(0);
}

void
alarm1_handler(intptr_t exinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(3);
	check_state_i(true, false, false, true, true);

	/*
	 *  iras_texのエラー検出
	 */
	ercd = iras_tex(TASK3, 0x0001);
	check_ercd(ercd, E_OBJ);
	ercd = iras_tex(TASK4, 0x0001);
	check_ercd(ercd, E_OBJ);

	/*
	 *  タスク例外処理を要求
	 */
	ercd = iras_tex(TASK2, 0x0001);
	check_ercd(ercd, E_OK);
	ercd = iras_tex(TASK1, 0x0001);
	check_ercd(ercd, E_OK);

	alm1_flag = true;
}

void
alarm2_handler(intptr_t exinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(9);
	check_state_i(true, false, false, true, false);

	/*
	 *  タスク例外処理を要求
	 */
	ercd = iras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);

	alm2_flag = true;
}

void
alarm3_handler(intptr_t exinf)
{
	ER		ercd;
	ID		tskid;

	check_point(14);
	check_state_i(true, false, false, true, true);
	ercd = iget_tid(&tskid);
	check_ercd(ercd, E_OK);
	check_assert(tskid == TSK_NONE);

	/*
	 *  タスク例外処理を要求
	 */
	ercd = iras_tex(TASK1, 0x0004);
	check_ercd(ercd, E_OK);

	alm3_flag = true;
}

void
cpuexc_handler(void *p_excinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(18);
	check_state_i(true, false, true, true, false);
	check_assert(xsns_xpn(p_excinf) == false);

	/*
	 *  iras_texのエラー検出
	 */
	ercd = iras_tex(TASK3, 0x0010);
	check_ercd(ercd, E_OBJ);
	ercd = iras_tex(TASK4, 0x0010);
	check_ercd(ercd, E_OBJ);

	/*
	 *  タスク例外処理を要求
	 */
	ercd = iras_tex(TASK2, 0x0010);
	check_ercd(ercd, E_OK);
	ercd = iras_tex(TASK1, 0x0010);
	check_ercd(ercd, E_OK);
}

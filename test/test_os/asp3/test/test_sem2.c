/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2008-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_sem2.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		セマフォ機能のテスト(2)
 *
 * 【テストの目的】
 *
 *  sig_semを，非タスクコンテキストからの呼び出した場合について，タスク
 *  コンテキストからの呼び出した場合との違いを中心にテストする．
 *  pol_sem，twai_semを，wai_semとの違いを中心にテストする．
 *
 * 【テスト項目】
 *
 *	(A) sig_sem（非タスクコンテキストからの呼出し）の静的エラーのテスト
 *		(A-1) CPUロック状態からの呼出し
 *	(B) sig_sem（非タスクコンテキストからの呼出し）によりセマフォ待ち状
 *		態のタスクが待ち解除される場合のテスト
 *		(B-1) アイドル状態から，待ち解除されたタスクに切り換わる
 *		(B-2) 実行状態のタスクから，待ち解除されたタスクに切り換わる
 *		(B-3) ディスパッチ保留状態で，切り換わらない
 *		(B-4) 待ち解除されたタスクが強制待ち状態で，切り換わらない
 *		(B-5) 待ち解除されたタスクが優先度が低く，切り換わらない
 *	(C) pol_semの静的エラーのテスト
 *		(C-1) 非タスクコンテキストからの呼出し
 *		(C-2) CPUロック状態からの呼出し
 *		(C-3) ディスパッチ禁止状態からの呼出し（E_CTXエラーにならない）
 *		(C-4) 割込み優先度マスク全解除でない状態からの呼出し（E_CTXエラー
 *		      にならない）
 *	(D) pol_semでポーリング失敗する
 *	(E) twai_semの静的エラーのテスト
 *		(E-1) 非タスクコンテキストからの呼出し
 *		(E-2) CPUロック状態からの呼出し
 *		(E-3) ディスパッチ禁止状態からの呼出し
 *		(E-4) 割込み優先度マスク全解除でない状態からの呼出し
 *		(E-5) tmoutが不正
 *	(F) twai_semでtmout=TMO_POLの時にポーリング失敗する
 *	(G) twai_semでtmout=TMO_FEVRの時にセマフォ待ち状態になる
 *	(H) twai_semでtmoutにタイムアウトを設定した時に，タイムアウト付きの
 *		セマフォ待ち状態になる
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 高優先度タスク
 *	TASK3: 低優先度タスク
 *	TASK4: 中優先度タスク
 *	TASK5: 中優先度タスク
 *	ALM1:  アラームハンドラ
 *  SEM1:  TA_NULL属性，初期資源数1，最大資源数1
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	loc_cpu()
 *		pol_sem(SEM1) -> E_CTX				... (C-2)
 *		twai_sem(SEM1, TMO_POL) -> E_CTX	... (E-2)
 *		unl_cpu()
 *	2:	dis_dsp()
 *		pol_sem(SEM1)						... (C-3)
 *		twai_sem(SEM1, TMO_POL) -> E_CTX	... (E-3)
 *		ena_dsp()
 *	3:	chg_ipm(TMAX_INTPRI)
 *		pol_sem(SEM1) -> E_TMOUT			... (C-4)(D)
 *		twai_sem(SEM1, TMO_POL) -> E_CTX	... (E-4)
 *		chg_ipm(TIPM_ENAALL)
 *	4:	twai_sem(SEM1, TMO_NBLK) -> E_PAR	... (E-5)
 *		twai_sem(SEM1, TMO_POL) -> E_TMOUT	... (F)
 *	5:	sta_alm(ALM1, TEST_TIME_CP) ... ALM1-1が実行開始するまで
 *	6:	twai_sem(SEM1, TMO_FEVR)			... (G)
 *	== ALM1-1 ==
 *	7:	pol_sem(SEM1) -> E_CTX				... (C-1)
 *		twai_sem(SEM1, TMO_POL) -> E_CTX	... (E-1)
 *	8:	loc_cpu()
 *		sig_sem(SEM1) -> E_CTX				... (A-1)
 *		unl_cpu()
 *	9:	sig_sem(SEM1)						... (B-1)
 *		RETURN
 *	== TASK1（続き）==
 *	10:	act_tsk(TASK3)
 *	11:	wai_sem(SEM1)
 *	== TASK3（優先度：低）==
 *	12:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-2が実行開始するまで
 *		call(wait_var())
 *	== ALM1-2 ==
 *	13:	sig_sem(SEM1)						... (B-2)
 *		call(signal_var())
 *		RETURN
 *	== TASK1（続き）==
 *	14:	wai_sem(SEM1)
 *	== TASK3（続き）==
 *	15:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-3が実行開始するまで
 *		dis_dsp()
 *		call(wait_var())
 *	== ALM1-3 ==
 *	16:	sig_sem(SEM1)						... (B-3)
 *		call(signal_var())
 *		RETURN
 *	== TASK3（続き）==
 *	17:	ena_dsp()
 *	== TASK1（続き）==
 *	18:	wai_sem(SEM1)
 *	== TASK3（続き）==
 *	19:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-4が実行開始するまで
 *		sus_tsk(TASK1)
 *		call(wait_var())
 *	== ALM1-4 ==
 *	20:	sig_sem(SEM1)						... (B-4)
 *		call(signal_var())
 *		RETURN
 *	== TASK3（続き）==
 *	21:	rsm_tsk(TASK1)
 *	== TASK1（続き）==
 *	22:	act_tsk(TASK2)
 *	== TASK2（優先度：高）==
 *	23:	tslp_tsk(2 * TEST_TIME_CP) -> E_TMOUT ... TASK2が実行再開するまで
 *	== TASK1（続き）==
 *	24:	wai_sem(SEM1)
 *	== TASK3（続き）==
 *	25:	ext_tsk() -> noreturn
 *	== TASK2（続き）==
 *	26:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-5が実行開始するまで
 *		call(wait_var())
 *	== ALM1-5 ==
 *	27:	sig_sem(SEM1)						... (B-5)
 *		call(signal_var())
 *		RETURN
 *	== TASK2（続き）==
 *	28:	ext_tsk() -> noreturn
 *	== TASK1（続き）==
 *	29:	sta_alm(ALM1, TEST_TIME_CP) ... ALM1-6が実行開始するまで
 *	30:	twai_sem(SEM1, 2 * TEST_TIME_CP) -> E_RLWAI ... rel_waiされるまで
 *	== ALM1-6 ==
 *	31:	rel_wai(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	32:	sta_alm(ALM1, 3 * TEST_TIME_CP) ... stp_almされるまで
 *	33:	twai_sem(SEM1, TEST_TIME_CP) -> E_TMOUT		... (H)
 *	34:	stp_alm(ALM1)
 *	35:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_sem2.h"

static volatile bool_t	flagvar;

static void
wait_var(void)
{
	flagvar = false;
	while (!flagvar);
}

static void
signal_var(void)
{
	flagvar = true;
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(7);
		ercd = pol_sem(SEM1);
		check_ercd(ercd, E_CTX);

		ercd = twai_sem(SEM1, TMO_POL);
		check_ercd(ercd, E_CTX);

		check_point(8);
		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_CTX);

		ercd = unl_cpu();
		check_ercd(ercd, E_OK);

		check_point(9);
		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(13);
		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_OK);

		signal_var();

		return;

		check_point(0);

	case 3:
		check_point(16);
		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_OK);

		signal_var();

		return;

		check_point(0);

	case 4:
		check_point(20);
		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_OK);

		signal_var();

		return;

		check_point(0);

	case 5:
		check_point(27);
		ercd = sig_sem(SEM1);
		check_ercd(ercd, E_OK);

		signal_var();

		return;

		check_point(0);

	case 6:
		check_point(31);
		ercd = rel_wai(TASK1);
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

	test_start(__FILE__);

	check_point(1);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = pol_sem(SEM1);
	check_ercd(ercd, E_CTX);

	ercd = twai_sem(SEM1, TMO_POL);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = pol_sem(SEM1);
	check_ercd(ercd, E_OK);

	ercd = twai_sem(SEM1, TMO_POL);
	check_ercd(ercd, E_CTX);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	ercd = pol_sem(SEM1);
	check_ercd(ercd, E_TMOUT);

	ercd = twai_sem(SEM1, TMO_POL);
	check_ercd(ercd, E_CTX);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = twai_sem(SEM1, TMO_NBLK);
	check_ercd(ercd, E_PAR);

	ercd = twai_sem(SEM1, TMO_POL);
	check_ercd(ercd, E_TMOUT);

	check_point(5);
	ercd = sta_alm(ALM1, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = twai_sem(SEM1, TMO_FEVR);
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = sta_alm(ALM1, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = twai_sem(SEM1, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_RLWAI);

	check_point(32);
	ercd = sta_alm(ALM1, 3 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = twai_sem(SEM1, TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(34);
	ercd = stp_alm(ALM1);
	check_ercd(ercd, E_OK);

	check_finish(35);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(23);
	ercd = tslp_tsk(2 * TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(26);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	wait_var();

	check_point(28);
	ercd = ext_tsk();

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(12);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	wait_var();

	check_point(15);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	wait_var();

	check_point(17);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = sus_tsk(TASK1);
	check_ercd(ercd, E_OK);

	wait_var();

	check_point(21);
	ercd = rsm_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = ext_tsk();

	check_point(0);
}

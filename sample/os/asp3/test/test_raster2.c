/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_raster2.c 740 2016-04-05 15:44:39Z ertl-hiro $
 */

/* 
 *		タスク終了要求機能に関するテスト(2)
 *
 * 【テストの目的】
 *
 *	dis_terの振舞いのテスト［NGKI3482］
 *	  ・dis_terに関するすべての要求をテストする．
 *	  ・dis_ter関数のC1カバレッジを達成する．
 *	ena_terの振舞いのテスト［NGKI3487］
 *	  ・ena_terに関するすべての要求をテストする．
 *	  ・ena_ter関数のC1カバレッジを達成する．
 *	sns_terの振舞いのテスト［NGKI3494］
 *	  ・sns_terに関するすべての要求をテストする．
 *	  ・sns_ter関数のC1カバレッジを達成する．
 *	タスク終了時に行うべき処理に関するテスト
 *	  ・タスク終了時に行うべき処理に関するすべての要求をテストする．
 *	  ・task_terminate関数のC1カバレッジを達成する（ena_terから呼び出し
 *		た場合に実行されることがないパスを除く）．
 *	ディスパッチ保留状態の解除に伴うタスク終了のテスト
 *	  ・dis_dspとchg_ipmでタスクが終了することをテストする．
 *
 * 【テスト項目】
 *
 *	(A) dis_terのエラー検出
 *		(A-1) 非タスクコンテキストからの呼出し［NGKI3483］
 *		(A-2) CPUロック状態からの呼出し［NGKI3484］
 *	(B) dis_terによるタスク終了禁止フラグのセット
 *		(B-1) 自タスクのタスク終了禁止フラグがセット［NGKI3486］
 *	(C) ena_terのエラー検出
 *		(C-1) 非タスクコンテキストからの呼出し［NGKI3488］
 *		(C-2) CPUロック状態からの呼出し［NGKI3489］
 *	(D) ena_terによるタスク終了禁止フラグのクリア
 *		(D-1) 自タスクのタスク終了禁止フラグがクリア［NGKI3491］
 *	(E) ena_terによるタスク終了［NGKI3683］
 *		(E-1) 自タスクが実行可能状態から休止状態に［NGKI1178］
 *		(E-2) ロックしているミューテックスのロック解除［NGKI2019］
 *		(E-3) (E-2)の処理により待ち解除されたタスクに切換え
 *	(F) ena_terによるタスク終了と再起動
 *		(F-1) 自タスクが実行可能状態に
 *		(F-2) (F-1)の時に起動要求キューイング数が1減少［NGKI1180］
 *		(F-3) 自タスクが実行状態に
 *	(G) タスクコンテキストから呼び出したsns_terの振舞い
 *		(G-1) タスク終了禁止状態の場合にtrueが返る［NGKI3495］
 *		(G-2) タスク終了許可状態の場合にfalseが返る［NGKI3495］
 *	(H) 非タスクコンテキストから呼び出したsns_terの振舞い
 *		(H-1) タスク終了禁止状態の場合にtrueが返る［NGKI3495］
 *		(H-2) タスク終了許可状態の場合にfalseが返る［NGKI3495］
 *		(H-3) 実行状態のタスクがない場合にtrueが返る［NGKI3496］
 *	(I) ena_dspによるタスク終了［NGKI3683］
 *		(I-1) 自タスクが実行可能状態から休止状態に［NGKI1178］
 *	(J) chg_ipmによるタスク終了［NGKI3683］
 *		(J-1) 自タスクが実行可能状態から休止状態に［NGKI1178］
 *	ASP3カーネルに適用されない要求：
 *		［NGKI3764］［NGKI3765］［NGKI3497］
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 低優先度タスク
 *	TASK3: 高優先度タスク
 *	ALM1: アラームハンドラ
 *	MTX1: ミューテックス
 *
 * 【テストシーケンス】
 *
 *	== TASK1 ==
 *	1:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-1が実行開始するまで
 *		slp_tsk()
 *	== ALM1-1（1回目）==
 *	2:	dis_ter() -> E_CTX							... (A-1)
 *		ena_ter() -> E_CTX							... (C-1)
 *		assert(sns_ter() == true)					... (H-3)
 *		wup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	3:	loc_cpu()
 *		dis_ter() -> E_CTX							... (A-2)
 *		ena_ter() -> E_CTX							... (C-2)
 *		unl_cpu()
 *		assert(sns_ter() == false)
 *	4:	dis_ter()									... (B-1)
 *		assert(sns_ter() == true)					... (G-1)
 *		ref_tsk(TSK_SELF, &rtsk)
 *		assert(rtsk.tskstat == TTS_RUN)
 *		assert(rtsk.raster == false)
 *		assert(rtsk.dister == true)
 *	5:	ena_ter()									... (D-1)
 *		assert(sns_ter() == false)					... (G-2)
 *		ref_tsk(TSK_SELF, &rtsk)
 *		assert(rtsk.tskstat == TTS_RUN)
 *		assert(rtsk.raster == false)
 *		assert(rtsk.dister == false)
 *	6:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-1（1回目）==
 *	7:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	8:	ras_ter(TASK2)
 *		tslp_tsk(TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-1（続き）==
 *	9:	ena_ter()									... (E-1)
 *	== TASK1（続き）==
 *	10:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	11:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-2（2回目）==
 *	12:	dis_ter()
 *		loc_mtx(MTX1)
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	13:	act_tsk(TASK3)
 *	== TASK3-1（1回目）==
 *	14:	loc_mtx(MTX1)
 *	== TASK1（続き）==
 *	15:	ras_ter(TASK2)
 *		slp_tsk()
 *	== TASK2-2（続き）==
 *	16:	ena_ter()									... (E-2)(E-3)
 *	== TASK3-1（続き）==
 *	17:	wup_tsk(TASK1)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	18:	act_tsk(TASK2)
 *		act_tsk(TASK2)
 *		ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.actcnt == 1U)
 *		slp_tsk()
 *	== TASK2-3（3回目）==
 *	19:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	20:	ras_ter(TASK2)
 *		chg_pri(TASK2, TASK3_PRIORITY)
 *	== TASK2-3（続き）==
 *	21:	ena_ter()									... (F-1)
 *	== TASK1（続き）==
 *	22:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.actcnt == 0U)					... (F-2)
 *		act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-4（4回目）==
 *	23:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	24:	ras_ter(TASK2)
 *		tslp_tsk(2 * TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-4（続き）==
 *	25:	ena_ter()									... (F-3)
 *	== TASK2-5（5回目）==
 *	26:	dis_ter()
 *		DO(while(true))
 *	== TASK1（続き）==
 *	27:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-2が実行開始するまで
 *		slp_tsk()
 *	== ALM1-2（2回目）==
 *	28:	assert(sns_ter() == true)					... (H-1)
 *		wup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	29:	ter_tsk(TASK2)
 *		act_tsk(TASK2)
 *		tslp_tsk(TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-6（6回目）==
 *	30:	ena_ter()
 *		DO(while(true))
 *	== TASK1（続き）==
 *	31:	sta_alm(ALM1, TEST_TIME_PROC) ... ALM1-3が実行開始するまで
 *		slp_tsk()
 *	== ALM1-3（3回目）==
 *	32:	assert(sns_ter() == false)					... (H-2)
 *		wup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	33:	ter_tsk(TASK2)
 *		act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-7（7回目）==
 *	34:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	35:	ras_ter(TASK2)
 *		tslp_tsk(TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-7（続き）==
 *	36:	dis_dsp()
 *		ena_ter()
 *		ena_dsp()									... (I-1)
 *	== TASK1（続き）==
 *	37:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	38:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-8（8回目）==
 *	39:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	40:	ras_ter(TASK2)
 *		tslp_tsk(TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-8（続き）==
 *	41:	chg_ipm(TMAX_INTPRI)
 *		ena_ter()
 *		chg_ipm(TIPM_ENAALL)						... (I-1)
 *	== TASK1（続き）==
 *	42:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	43:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_raster2.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(2);
		ercd = dis_ter();
		check_ercd(ercd, E_CTX);

		ercd = ena_ter();
		check_ercd(ercd, E_CTX);

		check_assert(sns_ter() == true);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(28);
		check_assert(sns_ter() == true);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 3:
		check_point(32);
		check_assert(sns_ter() == false);

		ercd = wup_tsk(TASK1);
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
	T_RTSK	rtsk;

	test_start(__FILE__);

	check_point(1);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = dis_ter();
	check_ercd(ercd, E_CTX);

	ercd = ena_ter();
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_assert(sns_ter() == false);

	check_point(4);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_assert(sns_ter() == true);

	ercd = ref_tsk(TSK_SELF, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RUN);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == true);

	check_point(5);
	ercd = ena_ter();
	check_ercd(ercd, E_OK);

	check_assert(sns_ter() == false);

	ercd = ref_tsk(TSK_SELF, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RUN);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == false);

	check_point(6);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(10);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_point(11);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(15);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.actcnt == 1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = chg_pri(TASK2, TASK3_PRIORITY);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.actcnt == 0U);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(2 * TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(27);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(31);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(37);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_point(38);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	ercd = tslp_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(42);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_finish(43);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(7);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(9);
		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(12);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(16);
		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(19);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(21);
		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 4:
		check_point(23);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(25);
		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 5:
		check_point(26);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		while(true);

		check_point(0);

	case 6:
		check_point(30);
		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		while(true);

		check_point(0);

	case 7:
		check_point(34);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(36);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		ercd = ena_dsp();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 8:
		check_point(39);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(41);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
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
		check_point(14);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(17);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

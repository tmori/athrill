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
 *  $Id: test_raster1.c 756 2016-10-03 10:47:38Z ertl-hiro $
 */

/* 
 *		タスク終了要求機能に関するテスト(1)
 *
 * 【テストの目的】
 *
 *	ras_terの振舞いのテスト［NGKI3469］
 *	  ・ras_terに関するすべての要求をテストする．
 *	  ・ras_ter関数のC1カバレッジを達成する．
 *	タスク終了時に行うべき処理に関するテスト
 *	  ・タスク終了時に行うべき処理に関するすべての要求をテストする．
 *	  ・task_terminate関数のC1カバレッジを達成する（ras_terから呼び出し
 *		た場合に実行されることがないパスを除く）．
 *	タスク終了要求機能に関連してタスクが持つ情報に関するテスト
 *	ref_tskのタスク終了要求機能に関するテスト
 *
 * 【テスト項目】
 *
 *	(A) ras_terのエラー検出
 *		(A-1) 非タスクコンテキストからの呼出し［NGKI3470］
 *		(A-2) CPUロック状態からの呼出し［NGKI3471］
 *		(A-3) tskidが有効範囲外（小さすぎる）［NGKI3472］
 *		(A-4) tskidが有効範囲外（大きすぎる）［NGKI3472］
 *		(A-5) 対象タスクが自タスク［NGKI3475］
 *		(A-6) 対象タスクが休止状態［NGKI3476］
 *	(B) ras_terによるタスク終了［NGKI3477］
 *		(B-1) 対象タスクが実行可能状態から休止状態に［NGKI1178］
 *		(B-2) 対象タスクが待ち状態から休止状態に［NGKI1178］
 *		(B-3) ロックしているミューテックスのロック解除［NGKI2019］
 *		(B-4) (B-3)の処理により待ち解除されたタスクに切換え
 *	(C) ras_terによるタスク終了と再起動［NGKI1179］
 *		(C-1) 対象タスクが実行可能状態に
 *		(C-2) (C-1)の時に起動要求キューイング数が1減少［NGKI1180］
 *		(C-3) 対象タスクが実行状態に
 *	(D) ras_terによるタスク終了要求
 *		(D-1) タスク終了要求がセットされる［NGKI3478］
 *		(D-2) 対象タスクが待ち解除され実行可能状態に［NGKI3479］
 *		(D-3) 待ち解除されたタスクからE_RASTERが返る［NGKI3480］
 *		(D-4) 対象タスクが待ち解除され実行状態に［NGKI3479］
 *		(D-5) 対象タスクが強制待ちから再開され実行状態に［NGKI3606］
 *	(E) タスク起動時の初期化に関連するテスト
 *		(E-1) タスク終了要求フラグがクリアされること［NGKI3451］
 *		(E-2) タスク終了禁止フラグがクリアされること［NGKI3454］
 *	(F) タスク終了要求フラグがセットされた時の振舞い（網羅はしない）
 *		(F-1) セットされた場合にタスクが待ち解除されること［NGKI3452］
 *		(F-2) セットされたタスクが待ち状態に遷移しようとするとE_RASTERエ
 *			  ラーとなること［NGKI3453］
 *		(F-3) セットされた場合にタスクが強制待ちから再開されること［NGKI3452］
 *		(F-4) セットされたタスクを強制待ち状態に遷移しようとすると
 *			  E_RASTERエラーとなること［NGKI3607］
 *	(G) ref_tskのタスク終了要求機能に関連するテスト
 *		(G-1) タスク終了要求フラグがセットされている場合にrasterにtrueが
 *			  返ること［NGKI3467］
 *		(G-2) タスク終了要求フラグがクリアされている場合にrasterにfalseが
 *			  返ること［NGKI3467］
 *		(G-3) タスク終了禁止状態の場合にdisterにtrueが返ること［NGKI3468］
 *		(G-4) タスク終了許可状態の場合にdisterにfalseが返ること［NGKI3468］
 *	ASPカーネルに適用されない要求：
 *		［NGKI3473］［NGKI3474］［NGKI3481］
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 低優先度タスク
 *	TASK3: 高優先度タスク
 *	TASK4: 高優先度タスク
 *	ALM1: アラームハンドラ
 *	MTX1: ミューテックス
 *
 * 【テストシーケンス】
 *
 *	== TASK1 ==
 *		call(set_bit_service(get_bit_kernel()))
 *	1:	act_tsk(TASK2)
 *		sta_alm(ALM1, TEST_TIME_CP) ... ALM1が実行開始するまで
 *		slp_tsk()
 *	== TASK2-1（1回目）==
 *	2:	DO(while(true))
 *	== ALM1 ==
 *	3:	ras_ter(TASK2) -> E_CTX						... (A-1)
 *		wup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	4:	loc_cpu()
 *		ras_ter(TASK2) -> E_CTX						... (A-2)
 *		unl_cpu()
 *	5:	ras_ter(0U) -> E_ID							... (A-3)
 *		ras_ter(TNUM_TSKID + 1) -> E_ID				... (A-4)
 *		ras_ter(TASK1) -> E_ILUSE					... (A-5)
 *		ras_ter(TASK3) -> E_OBJ						... (A-6)
 *	6:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *	7:	ras_ter(TASK2)								... (B-1)
 *	8:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	9:	act_tsk(TASK3)
 *	== TASK3-1（1回目）==
 *	10:	tloc_mtx(MTX1, 3 * TEST_TIME_CP) ... ras_terされるまで
 *	11:	slp_tsk()
 *	== TASK1（続き）==
 *	12:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_WAI)
 *		ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *	13:	ras_ter(TASK3)								... (B-2)
 *	14:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *		ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TSK_NONE)				... (B-3)
 *		assert(rmtx.wtskid == TSK_NONE)
 *	15:	act_tsk(TASK3)
 *	== TASK3-2（2回目）==
 *	16:	loc_mtx(MTX1)
 *	17:	slp_tsk()
 *	== TASK1（続き）==
 *	18:	act_tsk(TASK4)
 *	== TASK4 ==
 *	19:	loc_mtx(MTX1)
 *	== TASK1（続き）==
 *	20:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_WAI)
 *		ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK3)
 *		assert(rmtx.wtskid == TASK4)
 *	21:	ras_ter(TASK3)								... (B-4)
 *	== TASK4（続き）==
 *	22:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *		ref_mtx(MTX1, &rmtx)
 *		assert(rmtx.htskid == TASK4)
 *		assert(rmtx.wtskid == TSK_NONE)
 *	23:	slp_tsk()
 *	== TASK1（続き）==
 *	24:	act_tsk(TASK2)
 *		act_tsk(TASK2)
 *	25:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.actcnt == 1U)
 *	26:	ras_ter(TASK2)								... (C-1)
 *	27:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.actcnt == 0U)					... (C-2)
 *	28:	act_tsk(TASK3)
 *	== TASK3-3（3回目）==
 *	29:	slp_tsk()
 *	== TASK1（続き）==
 *	30:	act_tsk(TASK3)
 *	31:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_WAI)
 *		assert(rtsk.actcnt == 1U)
 *	32:	ras_ter(TASK3)								... (C-3)
 *	== TASK3-4（4回目）==
 *	33:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_RUN)
 *		assert(rtsk.actcnt == 0U)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	34:	act_tsk(TASK2)
 *		slp_tsk()
 *	== TASK2-2（2回目）==
 *	35:	dis_ter()
 *		wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	36:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.raster == false)				... (G-2)
 *		assert(rtsk.dister == true)					... (G-3)
 *	37:	ras_ter(TASK2)								... (D-1)
 *	38:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.raster == true)					... (G-1)
 *		assert(rtsk.dister == true)
 *	39:	ter_tsk(TASK2)
 *		act_tsk(TASK2)
 *	40:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.raster == false)				... (E-1)
 *		assert(rtsk.dister == false)				... (E-2)(G-4)
 *		tslp_tsk(TEST_TIME_CP) -> E_TMOUT ... TASK1が実行再開するまで
 *	== TASK2-3（3回目）==
 *	41:	dis_ter()
 *		slp_tsk() -> E_RASTER						... (D-3)
 *	== TASK1（続き）==
 *	42:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_WAI)
 *		assert(rtsk.raster == false)
 *		assert(rtsk.dister == true)
 *	43:	ras_ter(TASK2)								... (D-2)(F-1)
 *	44:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.tskstat == TTS_RDY)
 *		assert(rtsk.raster == true)
 *		assert(rtsk.dister == true)
 *		slp_tsk()
 *	== TASK2-3（続き）==
 *	45:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	46:	act_tsk(TASK3)
 *	== TASK3-5（5回目）==
 *	47:	dis_ter()
 *		slp_tsk() -> E_RASTER
 *	== TASK1（続き）==
 *	48:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_WAI)
 *		assert(rtsk.raster == false)
 *		assert(rtsk.dister == true)
 *	49:	ras_ter(TASK3)								... (D-4)
 *	== TASK3-5（続き）==
 *	50:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_RUN)
 *		assert(rtsk.raster == true)
 *		assert(rtsk.dister == true)
 *	51:	slp_tsk() -> E_RASTER						... (F-2)
 *		ena_ter()
 *	== TASK1（続き）==
 *	52:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	53:	act_tsk(TASK3)
 *	== TASK3-6（6回目）==
 *	54:	dis_ter()
 *		sus_tsk(TSK_SELF)
 *	== TASK1（続き）==
 *	55:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_SUS)
 *	56:	ras_ter(TASK3)								... (D-5)(F-3)
 *	== TASK3-6（続き）==
 *	57:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_RUN)
 *		assert(rtsk.raster == true)
 *		assert(rtsk.dister == true)
 *		sus_tsk(TSK_SELF) -> E_RASTER				... (F-4)
 *		ena_ter()
 *	== TASK1（続き）==
 *	58:	ref_tsk(TASK3, &rtsk)
 *		assert(rtsk.tskstat == TTS_DMT)
 *	59:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_raster1.h"

extern ER	bit_kernel(void);

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(3);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_CTX);

	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RTSK	rtsk;
	T_RMTX	rmtx;

	test_start(__FILE__);

	set_bit_service(get_bit_kernel());

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM1, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(5);
	ercd = ras_ter(0U);
	check_ercd(ercd, E_ID);

	ercd = ras_ter(TNUM_TSKID + 1);
	check_ercd(ercd, E_ID);

	ercd = ras_ter(TASK1);
	check_ercd(ercd, E_ILUSE);

	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OBJ);

	check_point(6);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_point(7);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_point(9);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_WAI);

	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(13);
	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TSK_NONE);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(15);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	check_point(20);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_WAI);

	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK3);

	check_assert(rmtx.wtskid == TASK4);

	check_point(21);
	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.actcnt == 1U);

	check_point(26);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.actcnt == 0U);

	check_point(28);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_WAI);

	check_assert(rtsk.actcnt == 1U);

	check_point(32);
	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == true);

	check_point(37);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.raster == true);

	check_assert(rtsk.dister == true);

	check_point(39);
	ercd = ter_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == false);

	ercd = tslp_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_TMOUT);

	check_point(42);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_WAI);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == true);

	check_point(43);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	check_point(44);
	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_RDY);

	check_assert(rtsk.raster == true);

	check_assert(rtsk.dister == true);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(46);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(48);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_WAI);

	check_assert(rtsk.raster == false);

	check_assert(rtsk.dister == true);

	check_point(49);
	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_point(53);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_SUS);

	check_point(56);
	ercd = ras_ter(TASK3);
	check_ercd(ercd, E_OK);

	check_point(58);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	check_finish(59);
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
		while(true);

		check_point(0);

	case 2:
		check_point(35);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(41);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = slp_tsk();
		check_ercd(ercd, E_RASTER);

		check_point(45);
		ercd = wup_tsk(TASK1);
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
	T_RTSK	rtsk;

	switch (++task3_count) {
	case 1:
		check_point(10);
		ercd = tloc_mtx(MTX1, 3 * TEST_TIME_CP);
		check_ercd(ercd, E_OK);

		check_point(11);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(16);
		ercd = loc_mtx(MTX1);
		check_ercd(ercd, E_OK);

		check_point(17);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(29);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 4:
		check_point(33);
		ercd = ref_tsk(TASK3, &rtsk);
		check_ercd(ercd, E_OK);

		check_assert(rtsk.tskstat == TTS_RUN);

		check_assert(rtsk.actcnt == 0U);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 5:
		check_point(47);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = slp_tsk();
		check_ercd(ercd, E_RASTER);

		check_point(50);
		ercd = ref_tsk(TASK3, &rtsk);
		check_ercd(ercd, E_OK);

		check_assert(rtsk.tskstat == TTS_RUN);

		check_assert(rtsk.raster == true);

		check_assert(rtsk.dister == true);

		check_point(51);
		ercd = slp_tsk();
		check_ercd(ercd, E_RASTER);

		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 6:
		check_point(54);
		ercd = dis_ter();
		check_ercd(ercd, E_OK);

		ercd = sus_tsk(TSK_SELF);
		check_ercd(ercd, E_OK);

		check_point(57);
		ercd = ref_tsk(TASK3, &rtsk);
		check_ercd(ercd, E_OK);

		check_assert(rtsk.tskstat == TTS_RUN);

		check_assert(rtsk.raster == true);

		check_assert(rtsk.dister == true);

		ercd = sus_tsk(TSK_SELF);
		check_ercd(ercd, E_RASTER);

		ercd = ena_ter();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RTSK	rtsk;
	T_RMTX	rmtx;

	check_point(19);
	ercd = loc_mtx(MTX1);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = ref_tsk(TASK3, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert(rtsk.tskstat == TTS_DMT);

	ercd = ref_mtx(MTX1, &rmtx);
	check_ercd(ercd, E_OK);

	check_assert(rmtx.htskid == TASK4);

	check_assert(rmtx.wtskid == TSK_NONE);

	check_point(23);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

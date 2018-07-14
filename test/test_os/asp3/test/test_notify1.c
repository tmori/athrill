/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2015-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_notify1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		通知処理のテスト(1)
 *
 * 【テストの目的】
 *
 *	タイムイベントとエラーの通知処理が正しく動作することを，アラーム通
 *	知機能を用いて確認する．ただし，以下はこのテストの範囲外とする．
 *	・タイムイベントハンドラの呼出しによる通知（他のテストでカバーされ
 *	　ているため）
 *	・通知のタイミングの妥当性（他のテストでカバーされているため）
 *	・付随情報のエラーチェック（コンフィギュレータで実施しているため）
 *
 * 【テスト項目】
 *
 *	(A) タイムイベントの通知
 *		(A-1) 変数の設定
 *		(A-2) 変数のインクリメント
 *		(A-3) タスクの起動
 *		(A-4) タスクの起床
 *		(A-5) セマフォの資源の返却
 *		(A-6) イベントフラグのセット
 *		(A-7) データキューへの送信
 *	(B) エラーの通知
 *		(B-1) 変数の設定
 *		(B-2) 変数のインクリメント
 *		(B-3) タスクの起動
 *		(B-4) タスクの起床
 *		(B-5) セマフォの資源の返却
 *		(B-6) イベントフラグのセット
 *		(B-7) データキューへの送信
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 低優先度タスク
 *	SEM1: セマフォ
 *	FLG1: イベントフラグ
 *	DTQ1: データキュー
 *	ALM1:  アラーム通知，変数の設定によるタイムイベントの通知
 *	ALM2:  アラーム通知，タスクの起動によるタイムイベントの通知
 *	ALM3:  アラーム通知，タスクの起床によるタイムイベントの通知，
 *						 変数の設定によるエラーの通知
 *	ALM4:  アラーム通知，セマフォの資源の返却によるタイムイベントの通知，
 *						 タスクの起動によるエラーの通知
 *	ALM5:  アラーム通知，イベントフラグのセットによるタイムイベントの通知
 *					   ※ASP3カーネルでは，イベントフラグのセットでエラー
 *						 になることはない
 *	ALM6:  アラーム通知，データキューへの送信によるタイムイベントの通知，
 *						 タスクの起床によるエラーの通知
 *	ALM7:  アラーム通知，タスクの起動によるタイムイベントの通知，
 *						 セマフォの資源の返却によるエラーの通知
 *	ALM8:  アラーム通知，タスクの起動によるタイムイベントの通知，
 *						 イベントフラグのセットによるエラーの通知
 *	ALM9:  アラーム通知，タスクの起動によるタイムイベントの通知，
 *						 データキューへの送信によるエラーの通知
 *	ALM10:  アラーム通知，変数のインクリメントによるタイムイベントの通知
 *	ALM11:  アラーム通知，タスクの起動によるタイムイベントの通知，
 *						  変数のインクリメントによるエラーの通知
 *
 * 【テストシーケンス】
 *
 *	== TASK1 ==
 *	1:	sta_alm(ALM1, TEST_TIME_PROC) ... ref_almで動作確認するまで
 *		assert(event_variable == false)
 *		ref_alm(ALM1, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *		DO(wait_event())								... (A-1)
 *	2:	ref_alm(ALM1, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	3:	sta_alm(ALM2, TEST_TIME_CP) ... TASK2-1が実行開始するまで
 *		ref_tsk(TASK2, &rtsk)
 *		assert((rtsk.tskstat & TTS_DMT) != 0U)
 *		ref_alm(ALM2, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	4:	slp_tsk()
 *	== TASK2-1（1回目）==
 *	5:	ref_alm(ALM2, &ralm)							... (A-3)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	6:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	7:	sta_alm(ALM3, TEST_TIME_CP) ... TASK2-1が実行再開するまで
 *		ref_tsk(TASK2, &rtsk)
 *		assert((rtsk.tskstat & TTS_RDY) != 0U)
 *		ref_alm(ALM3, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	8:	slp_tsk()
 *	== TASK2-1（続き）==
 *	9:	slp_tsk()										... (A-4)
 *	10:	ref_alm(ALM3, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	11:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	12:	dly_tsk(TEST_TIME_CP) ... TASK1が実行再開するまで
 *	== TASK2-1（続き）==
 *	13:	ext_tsk()
 *	== TASK1（続き）==
 *	14:	sta_alm(ALM3, TEST_TIME_PROC) ... すぐに発生しても良い
 *		DO(wait_error())								... (B-1)
 *		assert(error_variable == E_OBJ)
 *	15:	sta_alm(ALM4, TEST_TIME_PROC) ... ref_almで動作確認するまで
 *		ref_sem(SEM1, &rsem)
 *		assert(rsem.semcnt == 0U)
 *		ref_alm(ALM4, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	16:	dly_tsk(2 * TEST_TIME_PROC) ... ALM4が実行されるまで
 *	17:	ref_sem(SEM1, &rsem)							... (A-5)
 *		assert(rsem.semcnt == 1U)
 *		ref_alm(ALM4, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	18:	sta_alm(ALM4, TEST_TIME_CP) ... TASK2-2が実行開始するまで
 *	19:	slp_tsk()
 *	== TASK2-2（2回目）==
 *	20:	wup_tsk(TASK1)									... (B-3)
 *	== TASK1（続き）==
 *	21:	pol_sem(SEM1)
 *	22:	sta_alm(ALM5, 2 * TEST_TIME_CP) ... TASK1が実行開始するまで
 *		ref_flg(FLG1, &rflg)
 *		assert(rflg.flgptn == 0x00U)
 *		ref_alm(ALM5, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	23:	wai_flg(FLG1, 0x01U, TWF_ORW, &flgptn)			... (A-6)
 *	== TASK2-2（続き）==
 *	24:	slp_tsk()
 *	== TASK1（続き）==
 *	25:	assert(flgptn == 0x01U)
 *		ref_alm(ALM5, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	26:	sta_alm(ALM6, TEST_TIME_CP) ... ref_almで動作確認するまで
 *		ref_dtq(DTQ1, &rdtq)
 *		assert(rdtq.sdtqcnt == 0U)
 *		ref_alm(ALM6, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	27:	dly_tsk(2 * TEST_TIME_CP) ... ALM6が実行されるまで
 *	28:	ref_dtq(DTQ1, &rdtq)							... (A-7)
 *		assert(rdtq.sdtqcnt == 1U)
 *		ref_alm(ALM6, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	29:	sta_alm(ALM6, TEST_TIME_CP) ... TASK2-2が実行開始するまで
 *	30:	slp_tsk()
 *	== TASK2-2（続き）==
 *	31:	wup_tsk(TASK1)									... (B-4)
 *	== TASK1（続き）==
 *	32:	prcv_dtq(DTQ1, &data)
 *		assert(data == 0x01)
 *	33:	act_tsk(TASK2)
 *		sta_alm(ALM7, 2 * TEST_TIME_CP) ... TASK1が実行開始するまで
 *		ref_sem(SEM1, &rsem)
 *		assert(rsem.semcnt == 0U)
 *		ref_alm(ALM7, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	34:	wai_sem(SEM1)									... (B-5)
 *	== TASK2-2（続き）==
 *	35:	slp_tsk()
 *	== TASK1（続き）==
 *	36:	ref_alm(ALM7, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	37:	sta_alm(ALM8, TEST_TIME_CP) ... wai_flgで待ちに入るまで
 *		ref_flg(FLG1, &rflg)
 *		assert(rflg.flgptn == 0x01U)
 *		ref_alm(ALM8, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	38:	wai_flg(FLG1, 0x02U, TWF_ORW, &flgptn)			... (B-6)
 *	39:	assert(flgptn == 0x03U)
 *		ref_alm(ALM8, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	40:	sta_alm(ALM9,  TEST_TIME_CP) ... rcv_dtqで待ちに入るまで
 *		ref_dtq(DTQ1, &rdtq)
 *		assert(rdtq.sdtqcnt == 0U)
 *		ref_alm(ALM9, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *	41:	rcv_dtq(DTQ1, &data)							... (B-7)
 *	42:	assert(data == E_QOVR)
 *		ref_alm(ALM9, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	43:	sta_alm(ALM10, TEST_TIME_CP) ... ref_almで動作確認するまで
 *	44:	assert(count_variable == 1)
 *		ref_alm(ALM10, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *		DO(wait_count(1))								... (A-2)
 *	45:	ref_alm(ALM10, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	46:	sta_alm(ALM11, TEST_TIME_CP) ... ref_almで動作確認するまで
 *	47:	assert(count_variable == 2)
 *		ref_alm(ALM11, &ralm)
 *		assert((ralm.almstat & TALM_STA) != 0U)
 *		DO(wait_count(2))								... (B-2)
 *	48:	ref_alm(ALM9, &ralm)
 *		assert((ralm.almstat & TALM_STP) != 0U)
 *	49:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_notify1.h"

volatile bool_t	event_variable = false;
volatile int_t	count_variable = 1;
volatile ER		error_variable = E_INVALID;

void
wait_event(void)
{
	while (event_variable == false);
}

void
wait_count(int_t current)
{
	while (count_variable == current);
}

void
wait_error(void)
{
	while (error_variable == E_INVALID);
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RALM	ralm;
	T_RTSK	rtsk;
	T_RSEM	rsem;
	T_RFLG	rflg;
	FLGPTN	flgptn;
	T_RDTQ	rdtq;
	intptr_t	data;

	test_start(__FILE__);

	check_point(1);
	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	check_assert(event_variable == false);

	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	wait_event();

	check_point(2);
	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(3);
	ercd = sta_alm(ALM2, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert((rtsk.tskstat & TTS_DMT) != 0U);

	ercd = ref_alm(ALM2, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(4);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = sta_alm(ALM3, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK2, &rtsk);
	check_ercd(ercd, E_OK);

	check_assert((rtsk.tskstat & TTS_RDY) != 0U);

	ercd = ref_alm(ALM3, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(8);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = dly_tsk(TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = sta_alm(ALM3, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	wait_error();

	check_assert(error_variable == E_OBJ);

	check_point(15);
	ercd = sta_alm(ALM4, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.semcnt == 0U);

	ercd = ref_alm(ALM4, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(16);
	ercd = dly_tsk(2 * TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.semcnt == 1U);

	ercd = ref_alm(ALM4, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(18);
	ercd = sta_alm(ALM4, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(19);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = pol_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(22);
	ercd = sta_alm(ALM5, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.flgptn == 0x00U);

	ercd = ref_alm(ALM5, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(23);
	ercd = wai_flg(FLG1, 0x01U, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(25);
	check_assert(flgptn == 0x01U);

	ercd = ref_alm(ALM5, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(26);
	ercd = sta_alm(ALM6, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.sdtqcnt == 0U);

	ercd = ref_alm(ALM6, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(27);
	ercd = dly_tsk(2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.sdtqcnt == 1U);

	ercd = ref_alm(ALM6, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(29);
	ercd = sta_alm(ALM6, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = prcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_assert(data == 0x01);

	check_point(33);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM7, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_sem(SEM1, &rsem);
	check_ercd(ercd, E_OK);

	check_assert(rsem.semcnt == 0U);

	ercd = ref_alm(ALM7, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(34);
	ercd = wai_sem(SEM1);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = ref_alm(ALM7, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(37);
	ercd = sta_alm(ALM8, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.flgptn == 0x01U);

	ercd = ref_alm(ALM8, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(38);
	ercd = wai_flg(FLG1, 0x02U, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(39);
	check_assert(flgptn == 0x03U);

	ercd = ref_alm(ALM8, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(40);
	ercd = sta_alm(ALM9,  TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.sdtqcnt == 0U);

	ercd = ref_alm(ALM9, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	check_point(41);
	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_point(42);
	check_assert(data == E_QOVR);

	ercd = ref_alm(ALM9, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(43);
	ercd = sta_alm(ALM10, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(44);
	check_assert(count_variable == 1);

	ercd = ref_alm(ALM10, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	wait_count(1);

	check_point(45);
	ercd = ref_alm(ALM10, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_point(46);
	ercd = sta_alm(ALM11, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(47);
	check_assert(count_variable == 2);

	ercd = ref_alm(ALM11, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STA) != 0U);

	wait_count(2);

	check_point(48);
	ercd = ref_alm(ALM9, &ralm);
	check_ercd(ercd, E_OK);

	check_assert((ralm.almstat & TALM_STP) != 0U);

	check_finish(49);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RALM	ralm;

	switch (++task2_count) {
	case 1:
		check_point(5);
		ercd = ref_alm(ALM2, &ralm);
		check_ercd(ercd, E_OK);

		check_assert((ralm.almstat & TALM_STP) != 0U);

		check_point(6);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(9);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(10);
		ercd = ref_alm(ALM3, &ralm);
		check_ercd(ercd, E_OK);

		check_assert((ralm.almstat & TALM_STP) != 0U);

		check_point(11);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(13);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(20);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(24);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(31);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(35);
		ercd = slp_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

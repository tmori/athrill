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
 *  $Id: test_flg1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		イベントフラグ機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  set_flg（タスクコンテキストからの呼出し），wai_flg，CRE_FLGを網羅的
 *  にテストする（ただし，CRE_FLGのエラーのテストは除く）．
 *
 * 【テスト項目】
 *
 *	(A) set_flgの静的エラーのテスト
 *		(A-1) CPUロック状態からの呼出し［NGKI1604］
 *		(A-2) flgidが不正（小さすぎる）［NGKI1605］
 *		(A-3) flgidが不正（大きすぎる）［NGKI1605］
 *	(B) set_flgによりイベントフラグがセットされる［NGKI1608］
 *		(B-1) 現在パターンが0x00の時に，0x00をセットする
 *		(B-2) 現在パターンが0x00の時に，0x01をセットする
 *		(B-3) 現在パターンが0x01の時に，0x01をセットする
 *		(B-4) 現在パターンが0x01の時に，0x02をセットする
 *		(B-5) 現在パターンが0x01の時に，0x03をセットする
 *	(C) set_flgによりイベントフラグ待ち状態のタスクが待ち解除される
 *		(C-1) 待ち解除されたタスクに切り換わる
 *		(C-2) ディスパッチ保留状態で，切り換わらない
 *		(C-3) 待ち解除されたタスクが強制待ち状態で，切り換わらない
 *		(C-4) 待ち解除されたタスクが優先度が低く，切り換わらない
 *	(D) wai_flgの静的エラーのテスト
 *		(D-1) 非タスクコンテキストからの呼出し［NGKI1621］
 *		(D-2) CPUロック状態からの呼出し［NGKI1622］
 *		(D-3) ディスパッチ禁止状態からの呼出し［NGKI1623］
 *		(D-4) 割込み優先度マスク全解除でない状態からの呼出し［NGKI1623］
 *		(D-5) flgidが不正（小さすぎる）［NGKI1625］
 *		(D-6) flgidが不正（大きすぎる）［NGKI1625］
 *		(D-7) waiptnが0［NGKI1626］
 *		(D-8) wfmodeが0［NGKI1627］
 *		(D-9) wfmodeが(TWF_ORW|TWF_ANDW)［NGKI1627］
 *	(E) wai_flgの待ち解除条件のテスト［NGKI1636］
 *		TA_CLR属性でないイベントフラグでテストする．以下の各パターンで，
 *		ビットパターンの現在値が*p_flgptnに返ることを確認する
 *		(E-1) TWF_ORWで，いずれかのビットがセットされていれば成立すること
 *		(E-2) TWF_ORWで，対象外のビットがセットされていても不成立になること
 *		(E-3) TWF_ANDWで，すべてのビットがセットされていれば成立すること
 *		(E-4) TWF_ANDWで，一部のビットがセットされていても不成立になること
 *	(F) wai_flgでのTA_CLR属性のテスト［NGKI1637］
 *		TA_CLR属性のイベントフラグでテストする．以下の各パターンで，ビッ
 *		トパターンが0にクリアされることと，ビットパターンの現在値が
 *		*p_flgptnに返ることを確認する
 *		(F-1) TWF_ORWで，いずれかのビットがセットされていれば成立すること
 *		(F-2) TWF_ORWで，対象外のビットがセットされていても不成立になること
 *		(F-3) TWF_ANDWで，すべてのビットがセットされていれば成立すること
 *		(F-4) TWF_ANDWで，一部のビットがセットされていても不成立になること
 *	(G) 複数タスク待ちエラーのテスト［NGKI1632］
 *		TA_WMUL属性でないイベントフラグでテストする．以下の各パターンで，
 *		待ちタスクある時に，wai_flgを呼ぶとエラーになることを確認する
 *		(G-1) 待ち解除条件が成立している時
 *		(G-2) 待ち解除条件が成立していない時
 *	(H) 複数タスク待ちのテスト［NGKI1609］
 *		TA_WMUL属性，TA_CLR属性で，TA_TPRI属性でないイベントフラグでテ
 *		ストする．
 *		(H-1) 複数のタスクが待ちになれること
 *		(H-2) 複数のタスクがFIFO順で待ち解除されること
 *		(H-3) 待ち解除条件によっては，FIFO順で後ろのタスクのみが待ち解
 *			  除されること
 *	(I) 複数タスク待ちのテスト［NGKI1609］
 *		TA_WMUL属性，TA_CLR属性かつTA_TPRI属性のイベントフラグでテスト
 *		する．
 *		(I-1) 複数のタスクが待ちになれること
 *		(I-2) 複数のタスクが優先度順で待ち解除されること
 *		(I-3) 待ち解除条件によっては，優先度順で後ろのタスクのみが待ち
 *			  解除されること
 *	(J) イベントフラグのビットパターンの初期値が正しく設定される［NGKI1570］
 *		(J-1) 初期ビットパターンが0x00
 *		(J-2) 初期ビットパターンが0x01
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 高優先度タスク
 *	TASK3: 低優先度タスク
 *	ALM1:  アラームハンドラ
 *  FLG1:  TA_NULL属性，初期ビットパターン0x00
 *  FLG2:  TA_CLR属性，初期ビットパターン0x01
 *  FLG3:  TA_WMUL属性，初期ビットパターン0x00
 *  FLG4:  TA_WMUL|TA_TPRI属性，初期ビットパターン0x00
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *  1:	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)									... (J-1)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x01)									... (J-2)
 *	2:	loc_cpu()
 *		set_flg(FLG1, 0x01) -> E_CTX								... (A-1)
 *		wai_flg(FLG1, 0x01, TWF_ORW, &flgptn) -> E_CTX				... (D-2)
 *		unl_cpu()
 *		dis_dsp()
 *		wai_flg(FLG1, 0x01, TWF_ORW, &flgptn) -> E_CTX				... (D-3)
 *		ena_dsp()
 *		chg_ipm(TMAX_INTPRI)
 *		wai_flg(FLG1, 0x01, TWF_ORW, &flgptn) -> E_CTX				... (D-4)
 *		chg_ipm(TIPM_ENAALL)
 *		set_flg(0, 0x01) -> E_ID									... (A-2)
 *		wai_flg(0, 0x01, TWF_ORW, &flgptn) -> E_ID					... (D-5)
 *		set_flg(TNUM_FLGID+1, 0x01) -> E_ID							... (A-3)
 *		wai_flg(TNUM_FLGID+1, 0x01, TWF_ORW, &flgptn) -> E_ID		... (D-6)
 *		wai_flg(FLG1, 0x00, TWF_ORW, &flgptn) -> E_PAR				... (D-7)
 *		wai_flg(FLG1, 0x01, 0U, &flgptn) -> E_PAR					... (D-8)
 *		wai_flg(FLG1, 0x01, (TWF_ORW|TWF_ANDW), &flgptn) -> E_PAR	... (D-9)
 *		set_flg(FLG1, 0x00)											... (B-1)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *		set_flg(FLG1, 0x01)											... (B-2)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x01)
 *		set_flg(FLG1, 0x01)											... (B-3)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x01)
 *		set_flg(FLG1, 0x02)											... (B-4)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x03)
 *		clr_flg(FLG1, ~0x01)
 *		set_flg(FLG1, 0x03)											... (B-5)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x03)
 *	3:	act_tsk(TASK2)
 *	== TASK2（優先度：高）==
 *	4:	wai_flg(FLG1, 0x05, TWF_ORW, &flgptn)						... (E-1)
 *		assert(flgptn == 0x03)
 *	5:	wai_flg(FLG1, 0x04, TWF_ORW, &flgptn)						... (E-2)
 *	== TASK1（続き）==
 *	6:	set_flg(FLG1, 0x04)											... (C-1)
 *	== TASK2（続き）==
 *	7:	assert(flgptn == 0x07)
 *		wai_flg(FLG1, 0x03, TWF_ANDW, &flgptn)						... (E-3)
 *		assert(flgptn == 0x07)
 *	8:	wai_flg(FLG1, 0x0c, TWF_ANDW, &flgptn)						... (E-4)
 *	== TASK1（続き）==
 *	9:	dis_dsp()
 *		set_flg(FLG1, 0x08)											... (C-2)
 *  	ref_flg(FLG1, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x0f)
 *	10:	ena_dsp()
 *	== TASK2（続き）==
 *	11:	assert(flgptn == 0x0f)
 *		wai_flg(FLG2, 0x05, TWF_ORW, &flgptn)						... (F-1)
 *		assert(flgptn == 0x01)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *		set_flg(FLG2, 0x01)
 *	12:	wai_flg(FLG2, 0x04, TWF_ORW, &flgptn)						... (F-2)
 *	== TASK1（続き）==
 *	13:	chg_ipm(TMAX_INTPRI)
 *		set_flg(FLG2, 0x04)											... (C-2)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *	14:	chg_ipm(TIPM_ENAALL)
 *	== TASK2（続き）==
 *	15:	assert(flgptn == 0x05)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *		set_flg(FLG2, 0x03)
 *		wai_flg(FLG2, 0x03, TWF_ANDW, &flgptn)						... (F-3)
 *		assert(flgptn == 0x03)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *		set_flg(FLG2, 0x03)
 *	16:	wai_flg(FLG2, 0x05, TWF_ANDW, &flgptn)						... (F-4)
 *	== TASK1（続き）==
 *	17:	sus_tsk(TASK2)
 *		set_flg(FLG2, 0x04)											... (C-3)
 *  	ref_flg(FLG2, &rflg)
 *		assert(rflg.wtskid == TSK_NONE)
 *		assert(rflg.flgptn == 0x00)
 *	18:	rsm_tsk(TASK2)
 *	== TASK2（続き）==
 *	19:	assert(flgptn == 0x07)
 *	20:	wai_flg(FLG2, 0x01, TWF_ORW, &flgptn)
 *	== TASK1（続き）==
 *	21:	wai_flg(FLG2, 0x02, TWF_ORW, &flgptn) -> E_ILUSE			... (G-2)
 *		set_flg(FLG2, 0x02)
 *		wai_flg(FLG2, 0x02, TWF_ORW, &flgptn) -> E_ILUSE			... (G-1)
 *	22:	set_flg(FLG2, 0x01)
 *	== TASK2（続き）==
 *	23:	assert(flgptn == 0x03)
 *		act_tsk(TASK3)
 *	24:	slp_tsk()
 *	== TASK1（続き）==
 *	25:	wai_flg(FLG3, 0x01, TWF_ORW, &flgptn)
 *	== TASK3（優先度：低）==
 *	26:	wup_tsk(TASK2)
 *	== TASK2（続き）==
 *	27:	wai_flg(FLG3, 0x02, TWF_ORW, &flgptn)						... (H-1)
 *	== TASK3（続き）==
 *	28: set_flg(FLG3, 0x02)											... (H-3)
 *	== TASK2（続き）==
 *	29:	wai_flg(FLG3, 0x02, TWF_ORW, &flgptn)
 *	== TASK3（続き）==
 *	30: set_flg(FLG3, 0x03)											... (H-2)
 *	== TASK1（続き）==
 *	31:	wai_flg(FLG4, 0x01, TWF_ORW, &flgptn)
 *	== TASK3（優先度：低）==
 *	32: set_flg(FLG3, 0x03)
 *	== TASK2（続き）==
 *	33:	wai_flg(FLG4, 0x02, TWF_ORW, &flgptn)						... (I-1)
 *	== TASK3（続き）==
 *	34: set_flg(FLG4, 0x03)											... (I-2)
 *	== TASK2（続き）==
 *	35:	wai_flg(FLG4, 0x02, TWF_ORW, &flgptn)
 *	== TASK3（続き）==
 *	36: set_flg(FLG4, 0x01)											... (I-3)
 *	== TASK1（続き）==
 *	37:	clr_flg(FLG3, 0)
 *		sta_alm(ALM1, 2 * TEST_TIME_CP) ... ALM1が実行開始するまで
 *	38:	wai_flg(FLG3, 0x01, TWF_ORW, &flgptn)
 *	== TASK3（続き）==
 *	39:	wai_flg(FLG3, 0x02, TWF_ORW, &flgptn)
 *	== ALM1 ==
 *	40:	wai_flg(FLG1, 0x01, TWF_ORW, &flgptn) -> E_CTX				... (D-1)
 *		set_flg(FLG3, 0x01)
 *	41:	RETURN
 *	== TASK1（続き）==
 *	42:	set_flg(FLG3, 0x02)											... (C-4)
 *	43:	slp_tsk()
 *	== TASK3（続き）==
 *	44:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_flg1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;
	FLGPTN	flgptn;

	check_point(40);
	ercd = wai_flg(FLG1, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_CTX);

	ercd = set_flg(FLG3, 0x01);
	check_ercd(ercd, E_OK);

	check_point(41);
	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RFLG	rflg;
	FLGPTN	flgptn;

	test_start(__FILE__);

	check_point(1);
	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x01);

	check_point(2);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = set_flg(FLG1, 0x01);
	check_ercd(ercd, E_CTX);

	ercd = wai_flg(FLG1, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = wai_flg(FLG1, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_CTX);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	ercd = wai_flg(FLG1, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_CTX);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	ercd = set_flg(0, 0x01);
	check_ercd(ercd, E_ID);

	ercd = wai_flg(0, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_ID);

	ercd = set_flg(TNUM_FLGID+1, 0x01);
	check_ercd(ercd, E_ID);

	ercd = wai_flg(TNUM_FLGID+1, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_ID);

	ercd = wai_flg(FLG1, 0x00, TWF_ORW, &flgptn);
	check_ercd(ercd, E_PAR);

	ercd = wai_flg(FLG1, 0x01, 0U, &flgptn);
	check_ercd(ercd, E_PAR);

	ercd = wai_flg(FLG1, 0x01, (TWF_ORW|TWF_ANDW), &flgptn);
	check_ercd(ercd, E_PAR);

	ercd = set_flg(FLG1, 0x00);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	ercd = set_flg(FLG1, 0x01);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x01);

	ercd = set_flg(FLG1, 0x01);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x01);

	ercd = set_flg(FLG1, 0x02);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x03);

	ercd = clr_flg(FLG1, ~0x01);
	check_ercd(ercd, E_OK);

	ercd = set_flg(FLG1, 0x03);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x03);

	check_point(3);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = set_flg(FLG1, 0x04);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = set_flg(FLG1, 0x08);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG1, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x0f);

	check_point(10);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	ercd = set_flg(FLG2, 0x04);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	check_point(14);
	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = sus_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = set_flg(FLG2, 0x04);
	check_ercd(ercd, E_OK);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	check_point(18);
	ercd = rsm_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = wai_flg(FLG2, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_ILUSE);

	ercd = set_flg(FLG2, 0x02);
	check_ercd(ercd, E_OK);

	ercd = wai_flg(FLG2, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_ILUSE);

	check_point(22);
	ercd = set_flg(FLG2, 0x01);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = wai_flg(FLG3, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = wai_flg(FLG4, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(37);
	ercd = clr_flg(FLG3, 0);
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM1, 2 * TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = wai_flg(FLG3, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = set_flg(FLG3, 0x02);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	FLGPTN	flgptn;
	T_RFLG	rflg;

	check_point(4);
	ercd = wai_flg(FLG1, 0x05, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_assert(flgptn == 0x03);

	check_point(5);
	ercd = wai_flg(FLG1, 0x04, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(7);
	check_assert(flgptn == 0x07);

	ercd = wai_flg(FLG1, 0x03, TWF_ANDW, &flgptn);
	check_ercd(ercd, E_OK);

	check_assert(flgptn == 0x07);

	check_point(8);
	ercd = wai_flg(FLG1, 0x0c, TWF_ANDW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(11);
	check_assert(flgptn == 0x0f);

	ercd = wai_flg(FLG2, 0x05, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_assert(flgptn == 0x01);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	ercd = set_flg(FLG2, 0x01);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = wai_flg(FLG2, 0x04, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(15);
	check_assert(flgptn == 0x05);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	ercd = set_flg(FLG2, 0x03);
	check_ercd(ercd, E_OK);

	ercd = wai_flg(FLG2, 0x03, TWF_ANDW, &flgptn);
	check_ercd(ercd, E_OK);

	check_assert(flgptn == 0x03);

	ercd = ref_flg(FLG2, &rflg);
	check_ercd(ercd, E_OK);

	check_assert(rflg.wtskid == TSK_NONE);

	check_assert(rflg.flgptn == 0x00);

	ercd = set_flg(FLG2, 0x03);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = wai_flg(FLG2, 0x05, TWF_ANDW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(19);
	check_assert(flgptn == 0x07);

	check_point(20);
	ercd = wai_flg(FLG2, 0x01, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(23);
	check_assert(flgptn == 0x03);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = wai_flg(FLG3, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = wai_flg(FLG3, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(33);
	ercd = wai_flg(FLG4, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = wai_flg(FLG4, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;
	FLGPTN	flgptn;

	check_point(26);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = set_flg(FLG3, 0x02);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = set_flg(FLG3, 0x03);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = set_flg(FLG3, 0x03);
	check_ercd(ercd, E_OK);

	check_point(34);
	ercd = set_flg(FLG4, 0x03);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = set_flg(FLG4, 0x01);
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = wai_flg(FLG3, 0x02, TWF_ORW, &flgptn);
	check_ercd(ercd, E_OK);

	check_finish(44);
	check_point(0);
}

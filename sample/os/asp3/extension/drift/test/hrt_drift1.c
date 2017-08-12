/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: hrt_drift1.c 314 2015-02-08 14:54:01Z ertl-hiro $
 */

/* 
 *		ドリフト調整機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  ドリフト調整機能を網羅的にテストする．具体的には，ドリフト調整機能
 *  を追加することで追加・拡張された関数を中心にテストする．
 *
 * 【テスト項目】
 *
 *	(A) set_timの要求ベーステスト
 *	  (A-1) CPUロック状態からの呼出し［NGKI3598］
 *	  (A-2) driftがTMIN_DRIFTより小さい［NGKI3599］
 *	  (A-3) driftがTMAX_DRIFTより大きい［NGKI3599］
 *	  (A-4) ドリフト量がdriftで指定した値に設定される［NGKI3601］
 *	(B) update_current_evttim
 *	  (B-1) ドリフト量が正で端数の繰り上がりがない時
 *	  (B-2) ドリフト量が正で端数の繰り上がりがある時
 *	  (B-3) ドリフト量が負で端数の繰り上がりがない時
 *	  (B-4) ドリフト量が負で端数の繰り上がりがある時
 *	  ※ (current_evttim_frac >= 2000000U)のパスは単体でテストしたい
 *	(C) calc_current_evttim_ub（tmevtb_enqueueからの呼び出し）
 *	  (C-1) ドリフト量が正で端数の繰り上がりがない時
 *	  (C-2) ドリフト量が正で端数の繰り上がりがある時
 *	  (C-3) ドリフト量が負で端数の繰り上がりがない時
 *	  (C-4) ドリフト量が負で端数の繰り上がりがある時
 *	(D) calc_current_evttim_ub（tmevt_lefttimからの呼び出し）
 *	  (D-1) ドリフト量が正で端数の繰り上がりがない時
 *	  (D-2) ドリフト量が正で端数の繰り上がりがある時
 *	  (D-3) ドリフト量が負で端数の繰り上がりがない時
 *	  (D-4) ドリフト量が負で端数の繰り上がりがある時
 *	(E) set_hrt_event
 *	  (E-1) ドリフト量が正でタイムイベントヒープが空の時
 *	  (E-2) ドリフト量が正でhrtcntがHRTCNT_BOUNDを超える時 … 別にテスト
 *	  (E-3) ドリフト量が正でhrtcntがHRTCNT_BOUNDを超えない時
 *	  (E-4) ドリフト量が負でタイムイベントヒープが空の時
 *	  (E-5) ドリフト量が負でhrtcntがHRTCNT_BOUNDを超える時 … 別にテスト
 *	  (E-6) ドリフト量が負でhrtcntがHRTCNT_BOUNDを超えない時
 *	  ※ (c1 > HRTCNT_BOUND / 1000000U)のパスは単体でテストしたい
 *
 * 【使用リソース】
 *
 *	高分解能タイマモジュールの性質：HRT_CONFIG1
 *		TCYC_HRTCNT		未定義（2^32の意味）
 *		TSTEP_HRTCNT	1U
 *		HRTCNT_BOUND	4000000002U
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	ALM1:  アラームハンドラ
 *
 * 【補足説明】
 *
 *	以下のテストシーケンスのコメント中で，「発生：xxx」とは，高分解能タ
 *	イマのカウント値がxxxになった時にタイムイベントが発生することを意味
 *	する．タイムイベントのイベント発生時刻ではない．
 *
 * 【テストシーケンス】
 *
 *	== START ==
 *	// カーネル起動．高分解能タイマのカウント値とイベント時刻は10ずれる
 *	1:		[target_hrt_get_current -> 10U]
 *	2:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（優先度：中）==
 *	// ドリフト量を+1%に設定
 *	//		現時点：hrtcnt＝20，evttim＝10，evttim_frac＝0
 *	3:	set_dft(+10000)
 *	4:		[target_hrt_get_current -> 20U]
 *	5:		[target_hrt_set_event <- HRTCNT_EMPTY_P1P]	... (E-1)
 *	// タイムイベントを登録
 *	6:	sta_alm(ALM1, 100)							... (C-2)
 *	7:		[target_hrt_get_current -> 30U]
 *	//		現時点：hrtcnt＝30，evttim＝20，evttim_frac＝100000
 *	//		大きい方に丸めた ： evttim＝21，evttim_frac＝110000
 *	//		相対時間：100，イベント発生時刻：122
 *	//		カウント値：(101900000 / 1010000)の切り上げ → 101
 *	8:		[target_hrt_set_event <- 101U]			... (E-3)
 *	// ここで時間が経過したことを想定
 *	// ドリフト調整が効く直前
 *	9:	get_tim(&systim)							... (B-1)
 *	10:		[target_hrt_get_current -> 119U]
 *	//		現時点：hrtcnt＝119，evttim＝109，evttim_frac＝990000
 *	11:	assert(systim == 109U)
 *	// ここでドリフト調整が効く（システム時刻が2進む）
 *	12:	get_tim(&systim)							... (B-2)
 *	13:		[target_hrt_get_current -> 120U]
 *	//		現時点：hrtcnt＝120，evttim＝111，evttim_frac＝0
 *	14:	assert(systim == 111U)
 *	// ここで時間が経過したことを想定
 *	15:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	16:		[target_hrt_get_current -> 131U]		// ALM1が発生
 *	//		現時点：hrtcnt＝131，evttim＝122，evttim_frac＝110000
 *	== ALM1-1（1回目）==
 *	17:	RETURN
 *	// タイムイベントがなくなった場合
 *	18:		[target_hrt_get_current -> 140U]
 *	19:		[target_hrt_set_event <- HRTCNT_EMPTY_P1P]
 *	== TASK1（続き）==
 *	// ここで時間が経過したことを想定
 *	// ドリフト調整が効く直前
 *	20:	get_tim(&systim)							... (B-1)
 *	21:		[target_hrt_get_current -> 219U]
 *	//		現時点：hrtcnt＝219，evttim＝210，evttim_frac＝990000
 *	22:	assert(systim == 210U)
 *	// タイムイベントを登録
 *	23:	sta_alm(ALM1, 100U)							... (C-1)
 *	24:		[target_hrt_get_current -> 219U]
 *	//		現時点：hrtcnt＝219，evttim＝210，evttim_frac＝990000
 *	//		大きい方に丸めた ：  evttim＝212，evttim_frac＝0
 *	//		相対時間：100，イベント発生時刻：312
 *	//		カウント値：(101010000 / 1010000)の切り上げ → 101
 *	25:		[target_hrt_set_event <- 101U]
 *	// ここでドリフト調整が効く
 *	26:	get_tim(&systim)							... (B-2)
 *	27:		[target_hrt_get_current -> 220U]
 *	//		現時点：hrtcnt＝220，evttim＝212，evttim_frac＝0
 *	28:	assert(systim == 212U)
 *
 *	// ドリフト量を-1%に設定（端数が0になるタイミングで設定）
 *	29:	set_dft(-10000)
 *	30:		[target_hrt_get_current -> 220U]
 *	//		現時点：hrtcnt＝220，evttim＝212，evttim_frac＝0
 *	//		カウント値：(100000000 / 990000)の切り上げ → 102
 *	31:		[target_hrt_set_event <- 102U]			... (E-6)
 *	// ドリフト調整がすぐに効く
 *	32:	get_tim(&systim)							... (B-3)
 *	33:		[target_hrt_get_current -> 221U]
 *	//		現時点：hrtcnt＝221，evttim＝212，evttim_frac＝990000
 *	34:	assert(systim == 212U)
 *	// ここで時間が経過したことを想定
 *	// ドリフト調整が次に効く直前
 *	35:	get_tim(&systim)
 *	36:		[target_hrt_get_current -> 320U]		... (B-4)
 *	//		現時点：hrtcnt＝320，evttim＝311，evttim_frac＝0
 *	37:	assert(systim == 311U)
 *	// ここでドリフト調整が効く
 *	38:	get_tim(&systim)
 *	39:		[target_hrt_get_current -> 321U]		... (B-3)
 *	//		現時点：hrtcnt＝321，evttim＝311，evttim_frac＝990000
 *	40:	assert(systim == 311U)
 *	41:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	42:		[target_hrt_get_current -> 322U]		// ALM1が発生
 *	//		現時点：hrtcnt＝322，evttim＝312，evttim_frac＝980000
 *	== ALM1-2（2回目）==
 *	43:	RETURN
 *	// タイムイベントがなくなった場合
 *	44:		[target_hrt_get_current -> 330U]
 *	//		現時点：hrtcnt＝330，evttim＝320，evttim_frac＝900000
 *	45:		[target_hrt_set_event <- HRTCNT_EMPTY_M1P]	... (E-4)
 *	== TASK1（続き）==
 *	// ここで時間が経過したことを想定
 *
 *	// ドリフト量を+1%に設定（端数が0.5になるタイミングで設定）
 *	46:	set_dft(+10000)
 *	47:		[target_hrt_get_current -> 370U]
 *	//		現時点：hrtcnt＝370，evttim＝360，evttim_frac＝500000
 *	48:		[target_hrt_set_event <- HRTCNT_EMPTY_P1P]
 *	49:	get_tim(&systim)
 *	50:		[target_hrt_get_current -> 370U]
 *	//		現時点：hrtcnt＝370，evttim＝360，evttim_frac＝500000
 *	51:	assert(systim == 360U)
 *	// ドリフト調整が効く直前
 *	52:	get_tim(&systim)
 *	53:		[target_hrt_get_current -> 419U]
 *	//		現時点：hrtcnt＝419，evttim＝409，evttim_frac＝990000
 *	54:	assert(systim == 409U)
 *	// ここでドリフト調整が効く
 *	55:	get_tim(&systim)
 *	56:		[target_hrt_get_current -> 420U]
 *	//		現時点：hrtcnt＝420，evttim＝411，evttim_frac＝0
 *	57:	assert(systim == 411U)
 *	58:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "hrt_drift1.h"
#include "target_timer.h"

#if !defined(HRT_CONFIG1) || defined(HRT_CONFIG2)
#error Compiler option "-DHRT_CONFIG1" is missing.
#endif

#define HRTCNT_EMPTY		4000000000U
#define HRTCNT_EMPTY_P1P	3960396040U
#define HRTCNT_EMPTY_M1P	HRTCNT_BOUND

#define target_hrt_get_current	_kernel_target_hrt_get_current
#define target_hrt_set_event	_kernel_target_hrt_set_event
#define target_hrt_raise_event	_kernel_target_hrt_raise_event

void
target_hrt_raise_event(void)
{
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{

	switch (++alarm1_count) {
	case 1:
		check_point(17);
		return;

		check_point(0);

	case 2:
		check_point(43);
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
	SYSTIM	systim;

	check_point(3);
	ercd = set_dft(+10000);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = sta_alm(ALM1, 100);
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(11);
	check_assert(systim == 109U);

	check_point(12);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(14);
	check_assert(systim == 111U);

	check_point(15);
	target_raise_hrt_int(0U);

	check_point(20);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(22);
	check_assert(systim == 210U);

	check_point(23);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(28);
	check_assert(systim == 212U);

	check_point(29);
	ercd = set_dft(-10000);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(34);
	check_assert(systim == 212U);

	check_point(35);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(37);
	check_assert(systim == 311U);

	check_point(38);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(40);
	check_assert(systim == 311U);

	check_point(41);
	target_raise_hrt_int(0U);

	check_point(46);
	ercd = set_dft(+10000);
	check_ercd(ercd, E_OK);

	check_point(49);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(51);
	check_assert(systim == 360U);

	check_point(52);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(54);
	check_assert(systim == 409U);

	check_point(55);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(57);
	check_assert(systim == 411U);

	check_finish(58);
	check_point(0);
}

static uint_t	target_hrt_get_current_count = 0;

HRTCNT
target_hrt_get_current(void)
{

	switch (++target_hrt_get_current_count) {
	case 1:
		test_start(__FILE__);

		check_point(1);
		return(10U);

		check_point(0);

	case 2:
		check_point(4);
		return(20U);

		check_point(0);

	case 3:
		check_point(7);
		return(30U);

		check_point(0);

	case 4:
		check_point(10);
		return(119U);

		check_point(0);

	case 5:
		check_point(13);
		return(120U);

		check_point(0);

	case 6:
		check_point(16);
		return(131U);

		check_point(0);

	case 7:
		check_point(18);
		return(140U);

		check_point(0);

	case 8:
		check_point(21);
		return(219U);

		check_point(0);

	case 9:
		check_point(24);
		return(219U);

		check_point(0);

	case 10:
		check_point(27);
		return(220U);

		check_point(0);

	case 11:
		check_point(30);
		return(220U);

		check_point(0);

	case 12:
		check_point(33);
		return(221U);

		check_point(0);

	case 13:
		check_point(36);
		return(320U);

		check_point(0);

	case 14:
		check_point(39);
		return(321U);

		check_point(0);

	case 15:
		check_point(42);
		return(322U);

		check_point(0);

	case 16:
		check_point(44);
		return(330U);

		check_point(0);

	case 17:
		check_point(47);
		return(370U);

		check_point(0);

	case 18:
		check_point(50);
		return(370U);

		check_point(0);

	case 19:
		check_point(53);
		return(419U);

		check_point(0);

	case 20:
		check_point(56);
		return(420U);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
	return(0U);
}

static uint_t	target_hrt_set_event_count = 0;

void
target_hrt_set_event(HRTCNT hrtcnt)
{

	switch (++target_hrt_set_event_count) {
	case 1:
		check_point(2);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 2:
		check_point(5);
		check_assert(hrtcnt == HRTCNT_EMPTY_P1P);

		return;

		check_point(0);

	case 3:
		check_point(8);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 4:
		check_point(19);
		check_assert(hrtcnt == HRTCNT_EMPTY_P1P);

		return;

		check_point(0);

	case 5:
		check_point(25);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 6:
		check_point(31);
		check_assert(hrtcnt == 102U);

		return;

		check_point(0);

	case 7:
		check_point(45);
		check_assert(hrtcnt == HRTCNT_EMPTY_M1P);

		return;

		check_point(0);

	case 8:
		check_point(48);
		check_assert(hrtcnt == HRTCNT_EMPTY_P1P);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

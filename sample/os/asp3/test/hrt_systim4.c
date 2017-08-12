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
 *  $Id: hrt_systim4.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム時刻管理機能のテスト(4)
 *
 * 【テストの目的】
 *
 *  高分解能タイマモジュールの性質が異なる場合のテスト．高分解能タイマ
 *  モジュールの性質を表す3つの定数を参照している箇所を網羅的にテストす
 *  る．
 *
 * 【テスト項目】
 *
 *	(A) update_current_evttimでTCYC_HRTCNTを参照している箇所のテスト
 *	  (A-1) カウント値が周回していない場合
 *	  (A-2) カウント値が周回している場合
 *	  (A-3) カウント値が前回と同じ場合（境界のケース）
 *	(B) calc_current_evttim_ub（TSTEP_HRTCNTを参照している）を呼び出し
 *		ている処理のテスト
 *	  (B-1) tmevtb_enqueueでのタイムイベントの発生時刻の計算
 *	  (B-2) tmevt_lefttimでのタイムイベントが発生するまでの時間の計算
 *	(C) set_hrt_eventでHRTCNT_BOUNDを参照している箇所のテスト
 *	  (C-1) 登録されているタイムイベントがない時
 *	  (C-2) 先頭のタイムイベントまでの時間がHRTCNT_BOUNDを超える時
 *	  (C-3) 先頭のタイムイベントまでの時間がHRTCNT_BOUND以下の時
 *
 * 【使用リソース】
 *
 *	高分解能タイマモジュールの性質：HRT_CONFIG2
 *		TCYC_HRTCNT		(0x10000U * 10U)
 *		TSTEP_HRTCNT	10U
 *		HRTCNT_BOUND	(0x10000U * 9U)
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	ALM1:  アラームハンドラ
 *	ALM2:  アラームハンドラ
 *	ALM3:  アラームハンドラ
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
 *	// タイムイベントを2つ登録
 *	3:	sta_alm(ALM1, 100U)
 *	4:		[target_hrt_get_current -> 20U]			... (A-1) // 発生：130
 *	5:		[target_hrt_set_event <- 110U]			... (B-1)(C-3)
 *	6:	sta_alm(ALM2, 105U)
 *	7:		[target_hrt_get_current -> 20U]			... (A-3) // 発生：135
 *	8:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==
 *	9:		[target_hrt_get_current -> 130U]		// ALM1が発生
 *	== ALM1-1（1回目）==
 *	10:	wup_tsk(TASK1)
 *		RETURN
 *	11:		[target_hrt_get_current -> 130U]		... (A-3)
 *	12:		[target_hrt_set_event <- 5U]			... (B-1)
 *	== TASK1（続き）== 								// ALM2までの時間は5
 *	13:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	14:		[target_hrt_get_current -> 140U]		// ALM2が発生
 *	== ALM2-1（1回目）==
 *	15:	RETURN
 *	// タイムイベントがなくなった場合
 *	16:		[target_hrt_get_current -> 150U]
 *	17:		[target_hrt_set_event <- HRTCNT_EMPTY]	... (C-1)
 *	== TASK1（続き）==
 *	// 発生時刻までの長いタイムイベントを登録
 *	18:	sta_alm(ALM1, 1000000U)
 *	19:		[target_hrt_get_current -> 160U]		// 発生：1000170
 *	20:		[target_hrt_set_event <- HRTCNT_BOUND]	... (C-2)
 *	21:	ref_alm(ALM1, &ralm)
 *	22:		[target_hrt_get_current -> 170U]
 *	23:	assert(ralm.lefttim == 999990U)
 *	// ここで長い時間が経過したと想定
 *	24:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	25:		[target_hrt_get_current -> 600000U]
 *	26:		[target_hrt_set_event <- 400170U]
 *	== TASK1（続き）==
 *	// ここで長い時間が経過したと想定
 *	27:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	28:		[target_hrt_get_current -> 344810U]		... (A-2) // ALM1が発生
 *	== ALM1-2（2回目）==
 *	29:	RETURN
 *	30:		[target_hrt_get_current -> 344810U]
 *	31:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	32:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "hrt_systim4.h"
#include "target_timer.h"

#if defined(HRT_CONFIG1) || !defined(HRT_CONFIG2)
#error Compiler option "-DHRT_CONFIG2" is missing.
#endif

#define HRTCNT_EMPTY	HRTCNT_BOUND

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
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(10);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(29);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	alarm2_count = 0;

void
alarm2_handler(intptr_t exinf)
{

	switch (++alarm2_count) {
	case 1:
		check_point(15);
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
	T_RALM	ralm;

	check_point(3);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = sta_alm(ALM2, 105U);
	check_ercd(ercd, E_OK);

	check_point(8);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(13);
	target_raise_hrt_int(0U);

	check_point(18);
	ercd = sta_alm(ALM1, 1000000U);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	check_point(23);
	check_assert(ralm.lefttim == 999990U);

	check_point(24);
	target_raise_hrt_int(0U);

	check_point(27);
	target_raise_hrt_int(0U);

	check_finish(32);
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
		return(20U);

		check_point(0);

	case 4:
		check_point(9);
		return(130U);

		check_point(0);

	case 5:
		check_point(11);
		return(130U);

		check_point(0);

	case 6:
		check_point(14);
		return(140U);

		check_point(0);

	case 7:
		check_point(16);
		return(150U);

		check_point(0);

	case 8:
		check_point(19);
		return(160U);

		check_point(0);

	case 9:
		check_point(22);
		return(170U);

		check_point(0);

	case 10:
		check_point(25);
		return(600000U);

		check_point(0);

	case 11:
		check_point(28);
		return(344810U);

		check_point(0);

	case 12:
		check_point(30);
		return(344810U);

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
		check_assert(hrtcnt == 110U);

		return;

		check_point(0);

	case 3:
		check_point(12);
		check_assert(hrtcnt == 5U);

		return;

		check_point(0);

	case 4:
		check_point(17);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 5:
		check_point(20);
		check_assert(hrtcnt == HRTCNT_BOUND);

		return;

		check_point(0);

	case 6:
		check_point(26);
		check_assert(hrtcnt == 400170U);

		return;

		check_point(0);

	case 7:
		check_point(31);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

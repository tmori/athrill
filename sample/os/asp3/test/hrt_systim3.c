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
 *  $Id: hrt_systim3.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム時刻管理機能のテスト(3)
 *
 * 【テストの目的】
 *
 *	周期ハンドラの起動タイミングをテストする．
 *
 *	また，以下の関数のC1カバレッジを達成する．
 *		tmevtb_dequeue
 *		tmevtb_lefttim
 *
 * 【テスト項目】
 *
 *  (A) 周期ハンドラの起動タイミングの確認
 *	  (A-1) TA_STA属性で，起動位相（初回の起動時刻）が0の場合
 *	  (A-2) TA_STA属性で，起動位相（初回の起動時刻）が0でない場合
 *	  (A-3) sta_cycで，起動位相（初回の起動時刻）が0の場合
 *	  (A-4) sta_cycで，起動位相（初回の起動時刻）が0でない場合
 *	  (A-5) 決められた周期で再起動されること
 *  (B) tmevtb_dequeueの実行/分岐パスの網羅
 *	  (B-1) signal_time中からの呼び出し
 *	  (B-2) 先頭以外のタイムイベントを削除した場合
 *	  (B-3) 先頭のタイムイベントを削除した場合
 *  (C) tmevtb_lefttimの実行/分岐パスの網羅
 *	  (C-1) イベントの発生時刻を過ぎている場合
 *	  (C-2) イベントの発生時刻を過ぎていない場合
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
 *	CYC1:  周期ハンドラ（周期：1000，初期位相：0）
 *	CYC2:  周期ハンドラ（周期：500，初期位相：499）
 *
 * 【補足説明】
 *
 *	タイムイベントが登録されていない時に高分解能タイマに設定する相対は，
 *	ドリフト調整機能を持たない場合はHRTCNT_BOUNDであるのに対して，ドリ
 *	フト調整機能を持つ場合はTMAX_RELTIMをイベント時刻の伸縮率で割った値
 *	とHRTCNT_BOUNDの小さい方の値（このテストでは，ドリフト量を設定せず，
 *	HRTCNT_BOUND＞TMAX_RELTIMであるため，TMAX_RELTIMに一致）となる．そ
 *	こで，HRTCNT_EMPTYをこの値に定義し，ドリフト調整機能の有無によらず
 *	同じテストシーケンスが使えるようにする．
 *
 *	以下のテストシーケンスのコメント中で，「発生：xxx」とは，高分解能タ
 *	イマのカウント値がxxxになった時にタイムイベントが発生することを意味
 *	する．タイムイベントのイベント発生時刻ではない．
 *
 * 【テストシーケンス】
 *
 *	== START ==
 *	// カーネル起動．高分解能タイマのカウント値とイベント時刻は10ずれる
 *	1:		[target_hrt_get_current -> 10U]			// CYC1の次回発生：10
 *	2:		[target_hrt_raise_event]				// CYC2の次回発生：509
 *	== TASK1（優先度：中）==
 *	3:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	4:		[target_hrt_get_current -> 10U]			... (A-1)	// CYC1の発生
 *	== CYC1-1（1回目）==							// CYC1の次回発生：1010
 *	5:	RETURN
 *	6:		[target_hrt_get_current -> 20U]
 *	7:		[target_hrt_set_event <- 489U]
 *	== TASK1（続き）==
 *	// ここで時間が経過したことを想定
 *	8:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	9:		[target_hrt_get_current -> 509U]		... (A-2)	// CYC2の発生
 *	== CYC2-1（1回目）==							// CYC2の次回発生：1009
 *	10:	RETURN
 *	11:		[target_hrt_get_current -> 519U]
 *	12:		[target_hrt_set_event <- 490U]
 *	== TASK1（続き）==
 *	// ここで時間が経過したことを想定
 *	13:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	14:		[target_hrt_get_current -> 1009U]		... (A-5)	// CYC2の発生
 *	== CYC2-2（2回目）==							// CYC2の次回発生：1509
 *	15:	RETURN
 *	16:		[target_hrt_get_current -> 1010U]		... (A-5)	// CYC1の発生
 *	== CYC1-2（2回目）==							// CYC1の次回発生：2010
 *	17:	RETURN
 *	18:		[target_hrt_get_current -> 1020U]
 *	19:		[target_hrt_set_event <- 489U]
 *	== TASK1（続き）==
 *	20:	ref_cyc(CYC1, &rcyc)						... (C-2)
 *	21:		[target_hrt_get_current -> 1030U]
 *	22:	assert(rcyc.lefttim == 979U)
 *	23:	stp_cyc(CYC1)								... (B-2)
 *	24:	stp_cyc(CYC2)								... (B-3)
 *	25:		[target_hrt_get_current -> 1040U]
 *	26:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	27:	sta_cyc(CYC1)								... (A-3)
 *	28:		[target_hrt_get_current -> 1050U]		// CYC1の発生：1051
 *	29:		[target_hrt_set_event <- 1U]
 *	30:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	31:		[target_hrt_get_current -> 1051U]		// CYC1が発生
 *	== CYC1-3（3回目）==							// CYC1の次回発生：2051
 *	32:	RETURN
 *	33:		[target_hrt_get_current -> 1052U]
 *	34:		[target_hrt_set_event <- 999U]
 *	== TASK1（続き）==
 *	35:	sta_cyc(CYC2)								... (A-4)
 *	36:		[target_hrt_get_current -> 1060U]		// CYC2の発生：1560
 *	37:		[target_hrt_set_event <- 500U]
 *	// ここで時間が経過したことを想定
 *	38:	ref_cyc(CYC2, &rcyc)						... (C-2)
 *	39:		[target_hrt_get_current -> 1558U]
 *	40:	assert(rcyc.lefttim == 1U)
 *	// 以下のシーケンスは，タイマ割込みの受付が遅れた場合にしか発生しない．
 *	41:	ref_cyc(CYC2, &rcyc)						... (C-1)
 *	42:		[target_hrt_get_current -> 1561U]		// CYC2の発生時刻を
 *	43:	assert(rcyc.lefttim == 0U)					//  過ぎている
 *	44:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	45:		[target_hrt_get_current -> 1562U]		// CYC2が発生
 *	== CYC2-3（3回目）==							// CYC2の次回発生：2060
 *	46:	RETURN
 *	47:		[target_hrt_get_current -> 1563U]
 *	48:		[target_hrt_set_event <- 488U]
 *	== TASK1（続き）==
 *	// ここで時間が経過したことを想定
 *	49:	sta_alm(ALM1, 100U)
 *	50:		[target_hrt_get_current -> 1620U]		// ALM1の発生：1721
 *	51:		[target_hrt_set_event <- 101U]
 *	52:	sta_alm(ALM2, 200U)
 *	53:		[target_hrt_get_current -> 1630U]		// ALM2の発生：1831
 *	// ここで時間が経過したことを想定
 *	54:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	55:		[target_hrt_get_current -> 1721]		// ALM1が発生
 *	== ALM1-1（1回目）==
 *	56:	stp_alm(ALM2)								... (B-1)
 *		RETURN
 *	57:		[target_hrt_get_current -> 1730U]
 *	58:		[target_hrt_set_event <- 321U]
 *	== TASK1（続き）==
 *	59:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "hrt_systim3.h"
#include "target_timer.h"

#if !defined(HRT_CONFIG1) || defined(HRT_CONFIG2)
#error Compiler option "-DHRT_CONFIG1" is missing.
#endif

#ifdef TOPPERS_SUPPORT_DRIFT
#define HRTCNT_EMPTY	TMAX_RELTIM
#else /* TOPPERS_SUPPORT_DRIFT */
#define HRTCNT_EMPTY	HRTCNT_BOUND
#endif /* TOPPERS_SUPPORT_DRIFT */

#define target_hrt_get_current	_kernel_target_hrt_get_current
#define target_hrt_set_event	_kernel_target_hrt_set_event
#define target_hrt_raise_event	_kernel_target_hrt_raise_event

void
alarm2_handler(intptr_t exinf)
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
		check_point(56);
		ercd = stp_alm(ALM2);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	cyclic1_count = 0;

void
cyclic1_handler(intptr_t exinf)
{

	switch (++cyclic1_count) {
	case 1:
		check_point(5);
		return;

		check_point(0);

	case 2:
		check_point(17);
		return;

		check_point(0);

	case 3:
		check_point(32);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	cyclic2_count = 0;

void
cyclic2_handler(intptr_t exinf)
{

	switch (++cyclic2_count) {
	case 1:
		check_point(10);
		return;

		check_point(0);

	case 2:
		check_point(15);
		return;

		check_point(0);

	case 3:
		check_point(46);
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
	T_RCYC	rcyc;

	check_point(3);
	target_raise_hrt_int(0U);

	check_point(8);
	target_raise_hrt_int(0U);

	check_point(13);
	target_raise_hrt_int(0U);

	check_point(20);
	ercd = ref_cyc(CYC1, &rcyc);
	check_ercd(ercd, E_OK);

	check_point(22);
	check_assert(rcyc.lefttim == 979U);

	check_point(23);
	ercd = stp_cyc(CYC1);
	check_ercd(ercd, E_OK);

	check_point(24);
	ercd = stp_cyc(CYC2);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = sta_cyc(CYC1);
	check_ercd(ercd, E_OK);

	check_point(30);
	target_raise_hrt_int(0U);

	check_point(35);
	ercd = sta_cyc(CYC2);
	check_ercd(ercd, E_OK);

	check_point(38);
	ercd = ref_cyc(CYC2, &rcyc);
	check_ercd(ercd, E_OK);

	check_point(40);
	check_assert(rcyc.lefttim == 1U);

	check_point(41);
	ercd = ref_cyc(CYC2, &rcyc);
	check_ercd(ercd, E_OK);

	check_point(43);
	check_assert(rcyc.lefttim == 0U);

	check_point(44);
	target_raise_hrt_int(0U);

	check_point(49);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(52);
	ercd = sta_alm(ALM2, 200U);
	check_ercd(ercd, E_OK);

	check_point(54);
	target_raise_hrt_int(0U);

	check_finish(59);
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
		return(10U);

		check_point(0);

	case 3:
		check_point(6);
		return(20U);

		check_point(0);

	case 4:
		check_point(9);
		return(509U);

		check_point(0);

	case 5:
		check_point(11);
		return(519U);

		check_point(0);

	case 6:
		check_point(14);
		return(1009U);

		check_point(0);

	case 7:
		check_point(16);
		return(1010U);

		check_point(0);

	case 8:
		check_point(18);
		return(1020U);

		check_point(0);

	case 9:
		check_point(21);
		return(1030U);

		check_point(0);

	case 10:
		check_point(25);
		return(1040U);

		check_point(0);

	case 11:
		check_point(28);
		return(1050U);

		check_point(0);

	case 12:
		check_point(31);
		return(1051U);

		check_point(0);

	case 13:
		check_point(33);
		return(1052U);

		check_point(0);

	case 14:
		check_point(36);
		return(1060U);

		check_point(0);

	case 15:
		check_point(39);
		return(1558U);

		check_point(0);

	case 16:
		check_point(42);
		return(1561U);

		check_point(0);

	case 17:
		check_point(45);
		return(1562U);

		check_point(0);

	case 18:
		check_point(47);
		return(1563U);

		check_point(0);

	case 19:
		check_point(50);
		return(1620U);

		check_point(0);

	case 20:
		check_point(53);
		return(1630U);

		check_point(0);

	case 21:
		check_point(55);
		return(1721);

		check_point(0);

	case 22:
		check_point(57);
		return(1730U);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
	return(0U);
}

static uint_t	target_hrt_raise_event_count = 0;

void
target_hrt_raise_event(void)
{

	switch (++target_hrt_raise_event_count) {
	case 1:
		check_point(2);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	target_hrt_set_event_count = 0;

void
target_hrt_set_event(HRTCNT hrtcnt)
{

	switch (++target_hrt_set_event_count) {
	case 1:
		check_point(7);
		check_assert(hrtcnt == 489U);

		return;

		check_point(0);

	case 2:
		check_point(12);
		check_assert(hrtcnt == 490U);

		return;

		check_point(0);

	case 3:
		check_point(19);
		check_assert(hrtcnt == 489U);

		return;

		check_point(0);

	case 4:
		check_point(26);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 5:
		check_point(29);
		check_assert(hrtcnt == 1U);

		return;

		check_point(0);

	case 6:
		check_point(34);
		check_assert(hrtcnt == 999U);

		return;

		check_point(0);

	case 7:
		check_point(37);
		check_assert(hrtcnt == 500U);

		return;

		check_point(0);

	case 8:
		check_point(48);
		check_assert(hrtcnt == 488U);

		return;

		check_point(0);

	case 9:
		check_point(51);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 10:
		check_point(58);
		check_assert(hrtcnt == 321U);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

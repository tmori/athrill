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
 *  $Id: hrt_systim1.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム時刻管理機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  設計書の「高分解能タイマ割込みの発生タイミングの設定」「高分解能タ
 *  イマ割込みの処理」「相対時間指定によるタイムイベントの登録」の節に
 *  記述内容をテストする．
 *
 *  以下の関数のC1カバレッジを達成する．
 *		set_hrt_event（高分解能タイマ割込みの発生タイミングの設定）
 *		tmevtb_enqueue（相対時間指定によるタイムイベントの登録）
 *		signal_time（高分解能タイマ割込みの処理）
 *
 * 【テスト項目】
 *
 *	(A) 「高分解能タイマ割込みの発生タイミングの設定」の節の記述内容の
 *		テストとset_hrt_eventの実行/分岐パスの網羅
 *	  (A-1) タイムイベントが登録されていない時［ASPD1007］
 *	  (A-2) タイムイベントが登録されており，発生時刻を過ぎている時［ASPD1017］
 *	  (A-3) タイムイベントが登録されており，先頭のタイムイベントの発生
 *			時刻までの高分解タイマのカウントアップ値がHRTCNT_BOUND以下
 *			の時［ASPD1002］
 *	  (A-4) タイムイベントが登録されており，先頭のタイムイベントの発生
 *			時刻までの高分解タイマのカウントアップ値がHRTCNT_BOUNDを超
 *			える時［ASPD1006］
 *  (B) 「相対時間指定によるタイムイベントの登録」の節の記述内容のテス
 *		トとtmevtb_enqueueの実行/分岐パスの網羅
 *	  (B-1) 発生時刻が正しく設定されること［ASPD1026］［ASPD1027］
 *	  (B-2) タイムイベントヒープに挿入されること［ASPD1030］
 *	  (B-3) signal_timeの中で呼ばれた時［ASPD1034］
 *	  (B-4) 登録したタイムイベントが先頭でない時［ASPD1031］
 *	  (B-5) 登録したタイムイベントが先頭になった時（高分解能タイマを設
 *			定する）［ASPD1031］
 *  (C) signal_timeの実行/分岐パスの網羅
 *	  (C-1) タイムイベントヒープが空の状態で，signal_timeが呼び出された
 *			場合
 *	  (C-2) タイムイベントヒープの先頭のイベントの発生時刻前に，
 *			signal_timeが呼び出された場合
 *	  (C-3) signal_timeで，タイムイベントヒープの先頭のイベントのみを処
 *			理する場合
 *	  (C-4) signal_timeで，タイムイベントヒープの先頭から2つのイベント
 *			を，内側のループで処理する場合
 *	  (C-5) signal_timeで，タイムイベントヒープの先頭から2つのイベント
 *			を，外側のループで処理する場合
 *  (D) signal_timeから呼んだ処理で現在時刻が更新された場合
 *	  (D-1) 更新された現在時刻が，次のタイムイベントを過ぎており，次の
 *			タイムイベントを内側のループで処理する場合
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
 *	ALM2:  アラームハンドラ
 *	ALM3:  アラームハンドラ
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
 *	1:		[target_hrt_get_current -> 10U]
 *	2:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（優先度：中）==
 *	// タイムイベントを1つだけ登録
 *	3:	sta_alm(ALM1, 100U)
 *	4:		[target_hrt_get_current -> 20U]			// 発生：121
 *	5:		[target_hrt_set_event <- 101U]			... (A-3)
 *	6:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==								... (C-3)
 *	7:		[target_hrt_get_current -> 130U]		// ALM1が発生
 *	== ALM1-1（1回目）==
 *	8:	wup_tsk(TASK1)
 *		RETURN
 *	// タイムイベントがなくなった場合
 *	9:		[target_hrt_get_current -> 140U]
 *	10:		[target_hrt_set_event <- HRTCNT_EMPTY]	... (A-1)
 *	== TASK1（続き）==
 *	// 3つのタイムイベントを登録．その内の2つを内側のループで処理
 *	11:	sta_alm(ALM1, 100U)							... (B-5)
 *	12:		[target_hrt_get_current -> 150U]		// 発生：251
 *	13:		[target_hrt_set_event <- 101U]			... (B-1)(B-2)
 *	14:	sta_alm(ALM2, 100U)							... (B-4)
 *	15:		[target_hrt_get_current -> 160U]		// 発生：261
 *	16:	sta_alm(ALM3, 110U)
 *	17:		[target_hrt_get_current -> 170U]		// 発生：281
 *	18:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==								... (C-4)
 *	19:		[target_hrt_get_current -> 261U]		// ALM1とALM2が発生
 *	== ALM1-2（2回目）==
 *	20:	RETURN
 *	== ALM2-1（1回目）==							// 連続して処理
 *	21:	DO(target_raise_hrt_int(1U))
 *		RETURN
 *	22:		[target_hrt_get_current -> 280U]
 *	23:		[target_hrt_set_event <- 1U]			// ALM3までの時間は1
 *	== HRT_HANDLER ==
 *	24:		[target_hrt_get_current -> 290U]		// ALM3が発生
 *	== ALM3-1（1回目）==
 *	25:	wup_tsk(TASK1)
 *		RETURN
 *	26:		[target_hrt_get_current -> 300U]
 *	27:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// 2つのタイムイベントを登録．その2つを外側のループで処理
 *	28:	sta_alm(ALM1, 100U)
 *	29:		[target_hrt_get_current -> 310U]		// 発生：411
 *	30:		[target_hrt_set_event <- 101U]
 *	31:	sta_alm(ALM2, 100U)
 *	32:		[target_hrt_get_current -> 320U]		// 発生：421
 *	33:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==								... (C-5)
 *	34:		[target_hrt_get_current -> 420U]		// ALM1が発生
 *	== ALM1-3（3回目）==
 *	35:	RETURN
 *	36:		[target_hrt_get_current -> 430U]		// ALM2が発生
 *	== ALM2-2（2回目）==
 *	37:	wup_tsk(TASK1)
 *		RETURN
 *	38:		[target_hrt_get_current -> 440U]
 *	39:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// 2つのタイムイベントを登録
 *	40:	sta_alm(ALM1, 100U)
 *	41:		[target_hrt_get_current -> 450U]		// 発生：551
 *	42:		[target_hrt_set_event <- 101U]
 *	43:	sta_alm(ALM2, 100U)
 *	44:		[target_hrt_get_current -> 460U]		// 発生：561
 *	45:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==								... (D-1)
 *	46:		[target_hrt_get_current -> 560U]		// ALM1が発生
 *	== ALM1-4（4回目）==
 *	// アラームハンドラ中でさらに1つのタイムイベントを登録
 *	47:	sta_alm(ALM3, 10U)							... (B-3)
 *	48:		[target_hrt_get_current -> 570U]		// 発生：581，ALM2が発生
 *	49:	RETURN
 *	== ALM2-3（3回目）==
 *	50:	RETURN
 *	51:		[target_hrt_get_current -> 590U]		// ALM3が発生
 *	== ALM3-2（2回目）==
 *	52:	wup_tsk(TASK1)
 *		RETURN
 *	53:		[target_hrt_get_current -> 600U]
 *	54:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// 2つのタイムイベントを登録．2つめの方が発生時刻が早い場合
 *	55:	sta_alm(ALM1, 100U)							// 発生：711
 *	56:		[target_hrt_get_current -> 610U]
 *	57:		[target_hrt_set_event <- 101U]
 *	58:	sta_alm(ALM2, 50U)							// 発生：671
 *	59:		[target_hrt_get_current -> 620U]
 *	60:		[target_hrt_set_event <- 51U]
 *	61:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==
 *	62:		[target_hrt_get_current -> 680U]		// ALM2が発生
 *	== ALM2-4（4回目）==
 *	63:	DO(target_raise_hrt_int(1U))
 *		RETURN
 *	64:		[target_hrt_get_current -> 690U]
 *	65:		[target_hrt_set_event <- 21U]
 *	== HRT_HANDLER ==
 *	66:		[target_hrt_get_current -> 720U]		// ALM1が発生
 *	== ALM1-5（5回目）==
 *	67:	wup_tsk(TASK1)
 *	68:	RETURN
 *	69:		[target_hrt_get_current -> 730U]
 *	70:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// 2つのタイムイベントを登録．2つめのタイムイベントの登録時点で，
 *	// 1つめのタイムイベントの発生時刻を過ぎている場合
 *	71:	sta_alm(ALM1, 10U)
 *	72:		[target_hrt_get_current -> 740U]		// 発生：751
 *	73:		[target_hrt_set_event <- 11U]
 *	74:	sta_alm(ALM2, 100U)
 *	75:		[target_hrt_get_current -> 760U]		// 発生：861，ALM1が発生
 *	76:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==
 *	77:		[target_hrt_get_current -> 770U]		// ALM1が発生
 *	== ALM1-6（6回目）==
 *	78:	DO(target_raise_hrt_int(1U))
 *		RETURN
 *	79:		[target_hrt_get_current -> 780U]
 *	80:		[target_hrt_set_event <- 81U]
 *	== HRT_HANDLER ==								... (C-2)
 *	81:		[target_hrt_get_current -> 810U]		// スプリアス割込み
 *	== target_hrt_set_event-N						// target_raise_hrt_intを
 *	82:	assert(hrtcnt == 51U)						//	呼ぶためにこの形で記述
 *		DO(target_raise_hrt_int(1U))
 *		RETURN
 *	== HRT_HANDLER ==
 *	83:		[target_hrt_get_current -> 870U]		// ALM2が発生
 *	== ALM2-5（5回目）==
 *	84:	wup_tsk(TASK1)
 *		RETURN
 *	85:		[target_hrt_get_current -> 880U]
 *	86:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	87:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==								... (C-1)
 *	88:		[target_hrt_get_current -> 890U]		// スプリアス割込み
 *	89:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// タイムイベントを登録後に時間を進め，先頭のタイムイベントの発生時
 *	// 刻が過ぎた状況を作る
 *	90:	sta_alm(ALM1, 100U)
 *	91:		[target_hrt_get_current -> 900U]		// 発生：1001
 *	92:		[target_hrt_set_event <- 101U]
 *	93:	adj_tim(200)								// ALM1の発生：801
 *	94:		[target_hrt_get_current -> 910U]		// ALM1が発生
 *	95:		[target_hrt_raise_event]				... (A-2)
 *	96:	DO(target_raise_hrt_int(1U))
 *		slp_tsk()
 *	== HRT_HANDLER ==
 *	97:		[target_hrt_get_current -> 920U]
 *	== ALM1-7（7回目）==
 *	98:	wup_tsk(TASK1)
 *		RETURN
 *	99:		[target_hrt_get_current -> 930U]
 *	100:	[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// タイムイベントを登録後に時間を戻し，先頭のタイムイベントの発生時
 *	// 刻までの高分解タイマのカウントアップ値がHRTCNT_BOUNDを超える状況
 *	// を作る
 *	101:sta_alm(ALM1, TMAX_RELTIM)
 *	102:	[target_hrt_get_current -> 940U]		// 発生：4,000,000,941
 *	103:	[target_hrt_set_event <- 4000000001U]
 *	104:adj_tim(-200)								// 発生：4,000,001,141
 *	105:	[target_hrt_get_current -> 950U]
 *	106:	[target_hrt_set_event <- HRTCNT_BOUND]	... (A-4)
 *	// 後始末
 *	107:stp_alm(ALM1)
 *	108:	[target_hrt_get_current -> 960U]
 *	109:	[target_hrt_set_event <- HRTCNT_EMPTY]
 *	110:END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "hrt_systim1.h"
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

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm1_count) {
	case 1:
		check_point(8);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(20);
		return;

		check_point(0);

	case 3:
		check_point(35);
		return;

		check_point(0);

	case 4:
		check_point(47);
		ercd = sta_alm(ALM3, 10U);
		check_ercd(ercd, E_OK);

		check_point(49);
		return;

		check_point(0);

	case 5:
		check_point(67);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		check_point(68);
		return;

		check_point(0);

	case 6:
		check_point(78);
		target_raise_hrt_int(1U);

		return;

		check_point(0);

	case 7:
		check_point(98);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

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
	ER_UINT	ercd;

	switch (++alarm2_count) {
	case 1:
		check_point(21);
		target_raise_hrt_int(1U);

		return;

		check_point(0);

	case 2:
		check_point(37);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 3:
		check_point(50);
		return;

		check_point(0);

	case 4:
		check_point(63);
		target_raise_hrt_int(1U);

		return;

		check_point(0);

	case 5:
		check_point(84);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	alarm3_count = 0;

void
alarm3_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++alarm3_count) {
	case 1:
		check_point(25);
		ercd = wup_tsk(TASK1);
		check_ercd(ercd, E_OK);

		return;

		check_point(0);

	case 2:
		check_point(52);
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

	check_point(3);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(6);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(11);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(14);
	ercd = sta_alm(ALM2, 100U);
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = sta_alm(ALM3, 110U);
	check_ercd(ercd, E_OK);

	check_point(18);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(28);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = sta_alm(ALM2, 100U);
	check_ercd(ercd, E_OK);

	check_point(33);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(40);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(43);
	ercd = sta_alm(ALM2, 100U);
	check_ercd(ercd, E_OK);

	check_point(45);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(55);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(58);
	ercd = sta_alm(ALM2, 50U);
	check_ercd(ercd, E_OK);

	check_point(61);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(71);
	ercd = sta_alm(ALM1, 10U);
	check_ercd(ercd, E_OK);

	check_point(74);
	ercd = sta_alm(ALM2, 100U);
	check_ercd(ercd, E_OK);

	check_point(76);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(87);
	target_raise_hrt_int(0U);

	check_point(90);
	ercd = sta_alm(ALM1, 100U);
	check_ercd(ercd, E_OK);

	check_point(93);
	ercd = adj_tim(200);
	check_ercd(ercd, E_OK);

	check_point(96);
	target_raise_hrt_int(1U);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(101);
	ercd = sta_alm(ALM1, TMAX_RELTIM);
	check_ercd(ercd, E_OK);

	check_point(104);
	ercd = adj_tim(-200);
	check_ercd(ercd, E_OK);

	check_point(107);
	ercd = stp_alm(ALM1);
	check_ercd(ercd, E_OK);

	check_finish(110);
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
		return(130U);

		check_point(0);

	case 4:
		check_point(9);
		return(140U);

		check_point(0);

	case 5:
		check_point(12);
		return(150U);

		check_point(0);

	case 6:
		check_point(15);
		return(160U);

		check_point(0);

	case 7:
		check_point(17);
		return(170U);

		check_point(0);

	case 8:
		check_point(19);
		return(261U);

		check_point(0);

	case 9:
		check_point(22);
		return(280U);

		check_point(0);

	case 10:
		check_point(24);
		return(290U);

		check_point(0);

	case 11:
		check_point(26);
		return(300U);

		check_point(0);

	case 12:
		check_point(29);
		return(310U);

		check_point(0);

	case 13:
		check_point(32);
		return(320U);

		check_point(0);

	case 14:
		check_point(34);
		return(420U);

		check_point(0);

	case 15:
		check_point(36);
		return(430U);

		check_point(0);

	case 16:
		check_point(38);
		return(440U);

		check_point(0);

	case 17:
		check_point(41);
		return(450U);

		check_point(0);

	case 18:
		check_point(44);
		return(460U);

		check_point(0);

	case 19:
		check_point(46);
		return(560U);

		check_point(0);

	case 20:
		check_point(48);
		return(570U);

		check_point(0);

	case 21:
		check_point(51);
		return(590U);

		check_point(0);

	case 22:
		check_point(53);
		return(600U);

		check_point(0);

	case 23:
		check_point(56);
		return(610U);

		check_point(0);

	case 24:
		check_point(59);
		return(620U);

		check_point(0);

	case 25:
		check_point(62);
		return(680U);

		check_point(0);

	case 26:
		check_point(64);
		return(690U);

		check_point(0);

	case 27:
		check_point(66);
		return(720U);

		check_point(0);

	case 28:
		check_point(69);
		return(730U);

		check_point(0);

	case 29:
		check_point(72);
		return(740U);

		check_point(0);

	case 30:
		check_point(75);
		return(760U);

		check_point(0);

	case 31:
		check_point(77);
		return(770U);

		check_point(0);

	case 32:
		check_point(79);
		return(780U);

		check_point(0);

	case 33:
		check_point(81);
		return(810U);

		check_point(0);

	case 34:
		check_point(83);
		return(870U);

		check_point(0);

	case 35:
		check_point(85);
		return(880U);

		check_point(0);

	case 36:
		check_point(88);
		return(890U);

		check_point(0);

	case 37:
		check_point(91);
		return(900U);

		check_point(0);

	case 38:
		check_point(94);
		return(910U);

		check_point(0);

	case 39:
		check_point(97);
		return(920U);

		check_point(0);

	case 40:
		check_point(99);
		return(930U);

		check_point(0);

	case 41:
		check_point(102);
		return(940U);

		check_point(0);

	case 42:
		check_point(105);
		return(950U);

		check_point(0);

	case 43:
		check_point(108);
		return(960U);

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
		check_point(95);
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
		check_point(2);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 2:
		check_point(5);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 3:
		check_point(10);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 4:
		check_point(13);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 5:
		check_point(23);
		check_assert(hrtcnt == 1U);

		return;

		check_point(0);

	case 6:
		check_point(27);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 7:
		check_point(30);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 8:
		check_point(39);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 9:
		check_point(42);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 10:
		check_point(54);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 11:
		check_point(57);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 12:
		check_point(60);
		check_assert(hrtcnt == 51U);

		return;

		check_point(0);

	case 13:
		check_point(65);
		check_assert(hrtcnt == 21U);

		return;

		check_point(0);

	case 14:
		check_point(70);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 15:
		check_point(73);
		check_assert(hrtcnt == 11U);

		return;

		check_point(0);

	case 16:
		check_point(80);
		check_assert(hrtcnt == 81U);

		return;

		check_point(0);

	case 17:
		check_point(82);
		check_assert(hrtcnt == 51U);

		target_raise_hrt_int(1U);

		return;

		check_point(0);

	case 18:
		check_point(86);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 19:
		check_point(89);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 20:
		check_point(92);
		check_assert(hrtcnt == 101U);

		return;

		check_point(0);

	case 21:
		check_point(100);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 22:
		check_point(103);
		check_assert(hrtcnt == 4000000001U);

		return;

		check_point(0);

	case 23:
		check_point(106);
		check_assert(hrtcnt == HRTCNT_BOUND);

		return;

		check_point(0);

	case 24:
		check_point(109);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

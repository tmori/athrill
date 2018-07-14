/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2006-2017 by Embedded and Real-Time Systems Laboratory
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
 *  $Id:  $
 */

/*
 *		タイマドライバ（RZ/A1 OSタイマ用）
 *
 *  RZ/A1は2チャンネルのOSタイマを持つが，その内の1つを用いて高分解能タ
 *  イマを，もう1つを用いてオーバランタイマを実現する．
 */

#include "kernel_impl.h"
#include "time_event.h"
#ifdef TOPPERS_SUPPORT_OVRHDR
#include "overrun.h"
#endif /* TOPPERS_SUPPORT_OVRHDR */
#include "target_timer.h"
#include <sil.h>
#include <t_stddef.h>
/*
 *  割込み要求のクリア
 */
Inline void
HwcounterClearInterrupt(uint32_t intno)
{
	(void)x_clear_int(intno);
}
/*
 *  割込み禁止／許可設定
 */
Inline void
HwcounterDisableInterrupt(uint32_t intno)
{
	(void)x_disable_int(intno);
}

Inline void
HwcounterEnableInterrupt(uint32_t intno)
{
	(void)x_enable_int(intno);
}

 /*
 *  TAA タイマの動作開始／停止処理
 */
Inline void
SetTimerStartTAA(uint8_t ch)
{
	/* タイマ開始処理 */
	sil_wrb_mem((void *) TAAnCTL0(ch),
		( sil_reb_mem((void *) TAAnCTL0(ch)) | (1U << 7U) )	//TAAnCEビットセット
	);
}

Inline void
SetTimerStopTAA(uint8_t ch)
{
	/* タイマ停止処理 */
	sil_wrb_mem((void *) TAAnCTL0(ch),
		( sil_reb_mem((void *) TAAnCTL0(ch)) & ~(1U << 7U) )	//TAAnCEビットクリア
	);
}

/*
 *  TAAハードウェアカウンタ現在ティック値取得
 */
Inline uint16_t
GetCurrentTimeTAA(uint8_t ch)
{
	uint16_t	count;

	count = sil_reh_mem((void *) (TAAnCNT(ch)));
	return(count);
}
/*
 *  タイマの起動処理
 */
void
target_hrt_initialize(intptr_t exinf)
{
	uint8_t wk;
	/********************************************************
	 * 差分タイマ初期化
	 ********************************************************/
	/* 差分タイマのプリスケーラ設定 */
	wk = sil_reb_mem((void *) TAAnCTL0(TIMER_DTIM_ID));
	wk &= ~0x07;
	wk |= 0x05;//PLK5
	sil_wrb_mem((void *) TAAnCTL0(TIMER_DTIM_ID), wk);

	/* 差分タイマのインターバルタイマモード設定 */
	wk = sil_reb_mem((void *) TAAnCTL1(TIMER_DTIM_ID));
	wk = 0x00;
	sil_wrb_mem((void *) TAAnCTL1(TIMER_DTIM_ID), wk);


	/*******************************************************
	 * 現在値タイマ初期化
	 *******************************************************/
	/* 現在値タイマのプリスケーラ設定 */
	wk = sil_reb_mem((void *) TAAnCTL0(TIMER_CTIM_ID));
	wk &= ~0x07;
	wk |= 0x05;//PLK5
	sil_wrb_mem((void *) TAAnCTL0(TIMER_CTIM_ID), wk);

	/* 現在値タイマのインターバルタイマモード設定 */
	wk = sil_reb_mem((void *) TAAnCTL1(TIMER_CTIM_ID));
	wk = 0x00;
	sil_wrb_mem((void *) TAAnCTL1(TIMER_CTIM_ID), wk);

	/* 現在値タイマカウント周期設定 */
	/*
	 * TAAnCCR1は未使用とするため，FFFFを設定する．
	 */
	/*
	 *  OSタイマの設定値を最大値にしておく．
	 */
	sil_wrh_mem((void *) TAAnCCR0(TIMER_CTIM_ID), 0xFFFF);
	sil_wrh_mem((void *) TAAnCCR1(TIMER_CTIM_ID), 0xFFFF);

	/*
	 *  OSタイマを動作開始する．
	 */
	SetTimerStartTAA(TIMER_CTIM_ID);
	/*
	 *  タイマ割込み要求をクリアする．
	 */
	return;
}

/*
 *  タイマの停止処理
 */
void
target_hrt_terminate(intptr_t exinf)
{
	/* 差分タイマ停止 */
	SetTimerStopTAA(TIMER_DTIM_ID);

	/* 現在値タイマ停止 */
	SetTimerStopTAA(TIMER_CTIM_ID);

	return;
}

/*
 *  高分解能タイマへの割込みタイミングの設定
 */
void
target_hrt_set_event(HRTCNT hrtcnt)
{
	uint32_t intcnt;
	uint32_t curr;

	SetTimerStopTAA(TIMER_DTIM_ID);

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(TIMER_DTIM_INTNO);

	/*
	 *  hrtcnt後に割込みが発生するように設定する．
	 */
	curr = sil_reh_mem((void *) (TAAnCNT(TIMER_DTIM_ID)));

	intcnt = (curr + ((uint32_t)hrtcnt));
	if (intcnt > 0xFFFF) {
		intcnt -= 0xFFFF;
	}
	intcnt = ( (curr + ((uint32_t)hrtcnt)) & 0x0000FFFF);

	/* 差分タイマのタイマ値設定 */
	sil_wrh_mem((void *) TAAnCCR0(TIMER_DTIM_ID), (uint16_t)intcnt);

	/*
	 * カウント開始
	 */
	SetTimerStartTAA(TIMER_DTIM_ID);
	return;
}

/*
 *  高分解能タイマ割込みの要求
 */
void
target_hrt_raise_event(void)
{
	uint32_t intcnt;
	uint32_t curr;

	SetTimerStopTAA(TIMER_DTIM_ID);

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(TIMER_DTIM_INTNO);

	/*
	 *  hrtcnt後に割込みが発生するように設定する．
	 */
	curr = sil_reh_mem((void *) (TAAnCNT(TIMER_DTIM_ID)));

	intcnt = ( (curr + 1U) & 0x0000FFFF);

	/* 差分タイマのタイマ値設定 */
	sil_wrh_mem((void *) TAAnCCR0(TIMER_DTIM_ID), (uint16_t)intcnt);

	/*
	 * カウント開始
	 */
	SetTimerStartTAA(TIMER_DTIM_ID);
	return;
}

/*
 *  タイマ割込みハンドラ
 */
void
target_hrt_handler(void)
{
	HwcounterClearInterrupt(TIMER_DTIM_INTNO);
	/*
	 *  高分解能タイマ割込みを処理する．
	 */
	signal_time();
}

HRTCNT target_hrt_get_current(void)
{
	return GetCurrentTimeTAA(TIMER_CTIM_ID);
}


/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2014 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2014 by Spansion LLC, USA
 *  Copyright (C) 2014 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: tauj_hw_counter.c 549 2015-12-30 10:06:17Z ertl-honda $
 */

/*
 *		ハードウェアカウンタプログラムのターゲット依存定義（TAUJ用）
 *
 *  使用するタイマ：
 *    差分タイマ：目的の時間を設定する時の現在時間(現在値タイマ)
 *              と次の満了時間との相対時間をカウントすることで
 *              目的の絶対時間に満了したこととする
 *              count mode:count down once
 *
 *    現在値タイマ：カウンタ周期分のベースタイマを実現
 *              (絶対時間をカウント)
 *              count mode:continuous count down
 *
 *    また上記のタイマは32bitのダウンカウンタタイマである
 *
 *  制御方針：
 *
 *   現在値タイマはユーザ定義カウンタ最大値2倍+1までカウントし，
 *   周期タイマとして連続カウントダウンして，常に現在時刻を
 *   取得する．割込み発生する必要がないため，割込みなしモード
 *
 *   差分タイマは，満了処理を行うため，割込みありモードで動く
 *   アラームなどの満了点とタイマー1で示した現在時刻の差を
 *   現在値タイマに設定する
 *
 *  ポイント：
 *   満了処理は，現在時刻を影響しないため，現在値タイマを設けている
 *
 */

#include "Os.h"
#include "prc_sil.h"
#include "tauj_hw_counter.h"
#include "target_hw_counter.h"

/*
 *  =begin ハードウェアカウンタのTAUJn依存部
 */
/*
 *  ハードウェアカウンタの初期化
 *   n_d  : 差分タイマのユニット番号
 *   ch_d : 差分タイマのチャネル
 *   n_c  : 現在値タイマのユニット番号
 *   ch_c : 現在値タイマのチャネル
 */
/* カウンタの最大値の2倍+1 */
static TickType MAIN_HW_COUNTER_maxval[TNUM_HWCORE];
typedef struct {
	uint8	ctim_id;
	uint8	dtim_id;
	uint8	dtim_intno;
} HwCounterIdType;
static const HwCounterIdType HwCounterId[TNUM_HWCORE] = {
		{
				CORE0_HWC_CTIM_ID,
				CORE0_HWC_DTIM_ID,
				CORE0_HWC_DTIM_INTNO,
		},
		{
				CORE1_HWC_CTIM_ID,
				CORE1_HWC_DTIM_ID,
				CORE1_HWC_DTIM_INTNO,
		},
};
void
init_hwcounter_tauj(uint8 n_d, uint8 ch_d, uint8 n_c, uint8 ch_c, TickType maxval, TimeType nspertick, TickRefType cycle)
{
	uint8 wk;
	uint16 coreId = current_peid() - 1U;

	MAIN_HW_COUNTER_maxval[coreId] = maxval;
	/********************************************************
	 * 差分タイマ初期化
	 ********************************************************/
	/* 差分タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	/* 差分タイマ割込み禁止 */
	HwcounterDisableInterrupt(HwCounterId[coreId].dtim_id);

	/* 差分タイマ割込み要求クリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);

	/* 差分タイマのプリスケーラ設定 */
	wk = sil_reb_mem((void *) TAAnCTL0(HwCounterId[coreId].dtim_id));
	wk &= ~0x07;
	wk |= 0x05;//PLK5
	sil_wrb_mem((void *) TAAnCTL0(HwCounterId[coreId].dtim_id), wk);

	/* 差分タイマのインターバルタイマモード設定 */
	wk = sil_reb_mem((void *) TAAnCTL1(HwCounterId[coreId].dtim_id));
	wk = 0x00;
	sil_wrb_mem((void *) TAAnCTL1(HwCounterId[coreId].dtim_id), wk);


	/*******************************************************
	 * 現在値タイマ初期化
	 *******************************************************/
	/* 現在値タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].ctim_id);

	/* 現在値タイマのプリスケーラ設定 */
	wk = sil_reb_mem((void *) TAAnCTL0(HwCounterId[coreId].ctim_id));
	wk &= ~0x07;
	wk |= 0x05;//PLK5
	sil_wrb_mem((void *) TAAnCTL0(HwCounterId[coreId].ctim_id), wk);

	/* 現在値タイマのフリー・ランニング・モードモード設定 */
	wk = sil_reb_mem((void *) TAAnCTL1(HwCounterId[coreId].ctim_id));
	wk = 0x05;
	sil_wrb_mem((void *) TAAnCTL1(HwCounterId[coreId].ctim_id), wk);
}

/*
 *  ハードウェアカウンタの開始
 */
void
start_hwcounter_tauj(uint8 n_c, uint8 ch_c)
{
	uint16 coreId = current_peid() - 1U;
	/* 現在値タイマカウント周期設定 */
	/*
	 * TAAnCCR1は未使用とするため，FFFFを設定する．
	 */
	sil_wrh_mem((void *) TAAnCCR0(HwCounterId[coreId].ctim_id), (uint16)MAIN_HW_COUNTER_maxval[coreId]);
	sil_wrh_mem((void *) TAAnCCR1(HwCounterId[coreId].ctim_id), 0xFFFF);

	SetTimerStartTAA(HwCounterId[coreId].ctim_id);
}

/*
 *  ハードウェアカウンタの停止
 *   n_d  : 差分タイマのユニット番号
 *   ch_d : 差分タイマのチャネル
 *   n_c  : 現在値タイマのユニット番号
 *   ch_c : 現在値タイマのチャネル
 */
void
stop_hwcounter_tauj(uint8 n_d, uint8 ch_d, uint8 n_c, uint8 ch_c)
{
	uint16 coreId = current_peid() - 1U;

	/* 差分タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	/* 差分タイマ割込み禁止 */
	HwcounterDisableInterrupt(HwCounterId[coreId].dtim_intno);

	/* 差分タイマ割込み要求クリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);

	/* 現在値タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].ctim_id);

}

/*
 *  ハードウェアカウンタへの満了時間の設定
 *   n_d  : 差分タイマのユニット番号
 *   ch_d : 差分タイマのチャネル
 *   n_c  : 現在値タイマのユニット番号
 *   ch_c : 現在値タイマのチャネル
 */
void
set_hwcounter_tauj(uint8 n_d, uint8 ch_d, uint8 n_c, uint8 ch_c, TickType exprtick, TickType maxval)
{
	TickType	curr_time;
	TickType	value;
	uint16 coreId = current_peid() - 1U;

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);

	/* 差分タイマ停止 */
	/*
	 * TAAnCE=0にすると，TAAnCNT=FFFFになる．
	 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	exprtick = exprtick * 2;
	/* 差分タイマに設定する値を算出	*/
	curr_time = GetCurrentTimeTAA(HwCounterId[coreId].ctim_id, MAIN_HW_COUNTER_maxval[coreId]);
	if (exprtick >= curr_time) {
		value = exprtick - curr_time;
	}
	else {
		value = (exprtick - curr_time) + (MAIN_HW_COUNTER_maxval[coreId] + 1U);
	}

	/*
	 *  タイマに0x00を設定し，割込み発生後，再度0を設定した場合，2回目の
	 *  0x00設定後の割込みは発生しないので，0x00設定値を0x01に直して設定
	 */
	if (value == 0x00U) {
		value = 0x01U;
	}

	/* 差分タイマのタイマ値設定 */
	sil_wrh_mem((void *) TAAnCCR0(HwCounterId[coreId].dtim_id), value);

	/* 差分タイマの割込み許可 */
	HwcounterEnableInterrupt(HwCounterId[coreId].dtim_intno);
	/*
	 * カウント開始
	 */
	SetTimerStartTAA(HwCounterId[coreId].dtim_id);
}

/*
 *  ハードウェアカウンタの現在時間の取得
 */
TickType
get_hwcounter_tauj(uint8 n_c, uint8 ch_c, TickType maxval)
{
	uint16 coreId = current_peid() - 1U;
	return (GetCurrentTimeTAA(HwCounterId[coreId].ctim_id, MAIN_HW_COUNTER_maxval[coreId])/2);
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
cancel_hwcounter_tauj(uint8 n_d, uint8 ch_d)
{
	uint16 coreId = current_peid() - 1U;
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);
}

/*
 *  ハードウェアカウンタの強制割込み要求
 */
void
trigger_hwcounter_tauj(uint8 n_d, uint8 ch_d)
{
	uint16 coreId = current_peid() - 1U;

	/* 差分タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);

	/* 差分タイマの割込み許可 */
	HwcounterEnableInterrupt(HwCounterId[coreId].dtim_intno);

	sil_wrh_mem((void *) TAAnCCR0(HwCounterId[coreId].dtim_id), 1);
	sil_wrh_mem((void *) TAAnCCR1(HwCounterId[coreId].dtim_id), 0xFFFF);

	/*  差分タイマ開始 */
	SetTimerStartTAA(HwCounterId[coreId].dtim_id);
}

/*
 *  割込み要求のクリア
 */
void
int_clear_hwcounter_tauj(uint8 n_d, uint8 ch_d)
{
	uint16 coreId = current_peid() - 1U;
	/* 差分タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);
}

/*
 *  割込み要求のキャンセル
 *  ペンディングされている割込み要求をキャンセル
 */
void
int_cancel_hwcounter_tauj(uint8 n_d, uint8 ch_d)
{
	uint16 coreId = current_peid() - 1U;
	/* 差分タイマ停止 */
	SetTimerStopTAA(HwCounterId[coreId].dtim_id);

	/* 差分タイマの割込み要求のクリア */
	HwcounterClearInterrupt(HwCounterId[coreId].dtim_intno);
}

/*
 *  ハードウェアカウンタのインクリメント
 */
void
increment_hwcounter_tauj(uint8 n_d, uint8 ch_d)
{
	/* 未サポート */
	return;
}
/*
 *  =end ハードウェアカウンタのTAUJn依存部
 */

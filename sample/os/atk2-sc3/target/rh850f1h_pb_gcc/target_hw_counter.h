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
 *  $Id: target_hw_counter.h 35 2014-07-17 14:00:37Z ertl-honda $
 */

/*
 *		ハードウェアカウンタのターゲット依存定義（fl850f1l用）
 */

#ifndef TOPPERS_TARGET_HW_COUNTER_H
#define TOPPERS_TARGET_HW_COUNTER_H

#include "rh850_f1h.h"

/*
 *  使用するタイマーのユニット番号と差分タイマと現在値タイマのチャネル
 */
#ifdef HWC_USE_UNIT0

#define HWC_DTIM_UNIT  0  /* 0 or 1 */
#define HWC_DTIM_ID    0
#define HWC_CTIM_UNIT  0  /* 0 or 1 */
#define HWC_CTIM_ID    1

#elif defined(HWC_USE_UNIT1)

#define HWC_DTIM_UNIT  1  /* 0 or 1 */
#define HWC_DTIM_ID    0
#define HWC_CTIM_UNIT  1  /* 0 or 1 */
#define HWC_CTIM_ID    1

#endif /* HWC_USE_UNIT0 */

/*
 *  割込み番号
 */
#if (HWC_DTIM_UNIT == 0)
#define HWC_DTIM_INTNO  (TAUFJ0I0_INTNO + HWC_DTIM_ID)
#elif (HWC_DTIM_UNIT == 1)
#define HWC_DTIM_INTNO  (TAUFJ1I0_INTNO + HWC_DTIM_ID)
#else /*(HWC_DTIM_UNIT == 1) */
#error define HWC_UNIT  0 or 1.
#endif /* (HWC_DTIM_UNIT == 0) */

/*
 *  割込み優先度
 */
#if (HWC_DTIM_UNIT == 0)
#define HWC_DTIM_INTPRI  2
#elif (HWC_DTIM_UNIT == 1)
#define HWC_DTIM_INTPRI  1
#else /*(HWC_DTIM_UNIT == 1) */
#error define HWC_UNIT  0 or 1.
#endif /* (HWC_DTIM_UNIT == 0) */

/*
 *  タイマクロック周波数（Hz）（8MHz）
 */
#define TIMER_CLOCK_HZ		((uint32) 8000000)

/*
 *  TAUJ関連レジスタ
 */
#define TAUJ_BASE(n)	((uint32) (0xffe50000U + (n * 0x1000U)))
#define TAUJTPS(n)		(TAUJ_BASE(n) + 0x90U)
#define TAUJCDR(n, ch)	(TAUJ_BASE(n) + (ch * 0x04U))
#define TAUJCNT(n, ch)	(TAUJ_BASE(n) + 0x10U + (ch * 0x04U))
#define TAUJCMOR(n, ch)	(TAUJ_BASE(n) + 0x80U + (ch * 0x04U))
#define TAUJCMUR(n, ch)	(TAUJ_BASE(n) + 0x20U + (ch * 0x04U))
#define TAUJTS(n)		(TAUJ_BASE(n) + 0x54U)
#define TAUJTT(n)		(TAUJ_BASE(n) + 0x58U)

#define MCU_TAUJ_MASK_CK0				((uint16) 0xfff0)
#define MCU_TAUJ_CK0_0					((uint16) 0x0000) /* 分周なし */
#define MCU_TAUJ00_CMOR					((uint16) 0x0000)
#define MCU_TAUJ00_CMUR					((uint8) 0x01)

/*
 *  TAUJのユニット番号とチャネルから入力割込み番号への変換
 */
#define TAUJ_INTNO(n, ch)	(n == 0)? ((uint32) ((TAUFJ0I0_INTNO + ch))) : ((uint32) (TAUFJ1I0_INTNO + ch))

/*
 *  割込み要求のクリア
 */
LOCAL_INLINE void
HwcounterClearInterrupt(uint32 intno)
{
	/* 割込み制御レジスタ */
	uint32 eic_address = EIC_ADDRESS(intno);

	/* 割込み要求ビットのクリア */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) & ~EIRFn);
}

/*
 *  割込み禁止／許可設定
 */
LOCAL_INLINE void
HwcounterDisableInterrupt(uint32 intno)
{
	/* 割込み制御レジスタ */
	uint32 eic_address = EIC_ADDRESS(intno);

	/* 割込みマスクビットのセット */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) | EIMKn);
}

LOCAL_INLINE void
HwcounterEnableInterrupt(uint32 intno)
{
	/* 割込み制御レジスタ */
	uint32 eic_address = EIC_ADDRESS(intno);

	/* 割込みマスクビットのセット */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) & ~EIMKn);
}

/*
 *  TAUJn タイマの動作開始／停止処理
 */
LOCAL_INLINE void
SetTimerStartTAUJ(uint8 n, uint8 ch)
{
	/* タイマ開始処理 */
	sil_wrh_mem((void *) TAUJTS(n), (1 << ch));
}

LOCAL_INLINE void
SetTimerStopTAUJ(uint8 n, uint8 ch)
{
	/* タイマ停止処理 */
	sil_wrh_mem((void *) TAUJTT(n), (1 << ch));
}

/*
 *  TAUJnハードウェアカウンタ現在ティック値取得
 */
LOCAL_INLINE TickType
GetCurrentTimeTAUJ(uint8 n, uint8 ch, TickType maxval)
{
	TickType	count;
	TickType	curr_time = 0U;

	count = sil_rew_mem((void *) (TAUJCNT(n, ch)));

	/* ダウンカウンタの為，現在チック値に変換 */
	curr_time = maxval - count;
	curr_time = (curr_time % maxval);

	return(curr_time);
}


/* MAIN_HW_COUNTERの定義 */
extern void init_hwcounter_MAIN_HW_COUNTER(TickType maxval, TimeType nspertick);
extern void start_hwcounter_MAIN_HW_COUNTER(void);
extern void stop_hwcounter_MAIN_HW_COUNTER(void);
extern void set_hwcounter_MAIN_HW_COUNTER(TickType exprtick);
extern TickType get_hwcounter_MAIN_HW_COUNTER(void);
extern void cancel_hwcounter_MAIN_HW_COUNTER(void);
extern void trigger_hwcounter_MAIN_HW_COUNTER(void);
extern void int_clear_hwcounter_MAIN_HW_COUNTER(void);
extern void int_cancel_hwcounter_MAIN_HW_COUNTER(void);
extern void increment_hwcounter_MAIN_HW_COUNTER(void);

/*
 *  10msと一致するティック値(サンプルプログラム用)
 */
#define TICK_FOR_10MS	TIMER_CLOCK_HZ / 100

#endif /* TOPPERS_TARGET_HW_COUNTER_H */

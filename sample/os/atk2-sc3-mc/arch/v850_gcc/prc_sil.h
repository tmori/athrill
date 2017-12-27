/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
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
 *  $Id: prc_sil.h 117 2014-12-10 03:58:03Z t_ishikawa $
 */

/*
 *		メモリ及び周辺アドレスアクセスためのプロセッサ依存部（V850用）
 */
#include <prc_insn.h>

#ifndef TOPPERS_PRC_SIL_H
#define TOPPERS_PRC_SIL_H

#define PSW_ID_BITPTN	(1U << 5)

#define EIRFn		0x1000                      /* 割り込み要求ビット(bit12) */
#define EIMKn		0x0080                      /* 割り込み許可ビット(bit7) */

/*
 *  プロセッサのエンディアン
 */
#define SIL_ENDIAN_LITTLE

#ifndef TOPPERS_MACRO_ONLY

/*
 *  メモリ空間アクセス関数郡
 */

/*
 *  8ビット単位の読出し／書込み
 */
LOCAL_INLINE uint8
sil_reb_mem(void *mem)
{
	uint8 data;

	data = *((volatile uint8 *) mem);
	return(data);
}

LOCAL_INLINE void
sil_wrb_mem(void *mem, uint8 data)
{
	*((volatile uint8 *) mem) = data;
}

/*
 *  16ビット単位の読出し／書込み
 */
LOCAL_INLINE uint16
sil_reh_mem(void *mem)
{
	uint16 data;

	data = *((volatile uint16 *) mem);
	return(data);
}

LOCAL_INLINE void
sil_wrh_mem(void *mem, uint16 data)
{
	*((volatile uint16 *) mem) = data;
}

/*
 *  32ビット単位の読出し／書込み
 */
LOCAL_INLINE void
sil_wrw_mem(void *mem, uint32 data)
{
	*((volatile uint32 *) mem) = data;
}

LOCAL_INLINE uint32
sil_rew_mem(void *mem)
{
	uint32 data;

	data = *((volatile uint32 *) mem);
	return(data);
}

/*
 *  NMIを除くすべての割込みの禁止
 */
LOCAL_INLINE uint32
TOPPERS_disint(void)
{
	/* 現在のPSWを取得し，IDビットをセットする */
	volatile uint32 psw = current_psw();
	set_psw(psw | PSW_ID_BITPTN);

	return(psw);
}

/*
 *  割込み優先度マスク（内部表現）の現在値の設定
 */
LOCAL_INLINE void
TOPPERS_enaint(uint32 TOPPERS_psw)
{
	if ((TOPPERS_psw & PSW_ID_BITPTN) == 0u) {
		set_psw(current_psw() & ~PSW_ID_BITPTN);
	}
}
/*
 *  コアIDの取得
 */
LOCAL_INLINE uint32
sil_get_coreid(void)
{
	uint32 htcfg0_val;
	Asm("stsr sr0, %0, 2" : "=r" (htcfg0_val) :);
	return(((htcfg0_val >> 16) & 0x03) - 1);
}

/*
 *  SILのスピンロック用変数(prc_config.c)
 */
extern uint32 TOPPERS_spn_var;

/*
 *  割込み禁止を伴うスピンロックの取得
 */
LOCAL_INLINE uint32
TOPPERS_spn_lock(void)
{
	volatile uint32 psw;
	uint32			locked;

  retry:
	/* 割込み状態を取得した上，禁止状態へ */
	psw = TOPPERS_disint();

	/*
	 * lockedを事前にC言語で0にする方法もあるが，
	 * GHSのインラインアセンブラに不具合があり，
	 * r1にlockedもp_lockも割り付けるため，
	 * あえて両方のパスの最後で設定している．
	 */
	Asm("1: ldl.w [%1], r21 \n"
		"   cmp   r0, r21   \n"
		"   bnz   2f        \n"
		"   mov   1, r21    \n"
		"   stc.w r21, [%1] \n"
		"   cmp   r0, r21   \n"
		"   be    2f        \n"
		"   mov   1, %0     \n"
		"   br    3f        \n"
		"2:                 \n"
		"   mov   0, %0     \n"
		"3:                 \n"
		: "=r" (locked)
		: "r" (&TOPPERS_spn_var)
		: "cc", "r21", "memory");

	if (locked == 0) {
		/* スピンロックが取得できなかった場合 */
		/* 割込み状態を前の状態へ */
		TOPPERS_enaint(psw);
		goto retry;
	}
#if 0
	Asm("1: ldl.w [%0], r21 \n"
		"   cmp   r0, r21   \n"
		"   bnz   2f        \n"
		"   mov   1, r21    \n"
		"   stc.w r21, [%0] \n"
		"   cmp   r0, r21   \n"
		"   bnz   3f        \n"
		"2: snooze          \n"
		"   br    1b        \n"
		"3:                 \n" :: "r" (&TOPPERS_spn_var) : "cc", "r21");
#endif

	V850_MEMORY_CHANGED
	Asm("syncp");

	return(psw);
}

/*
 *  スピンロックの返却
 */
LOCAL_INLINE void
TOPPERS_spn_unlock(uint32 psw)
{
	TOPPERS_spn_var = 0;
	/* スピンロックの取得前の割込みの禁止状態へ */
	TOPPERS_enaint(psw);
	V850_MEMORY_CHANGED
}

/*
 *  全割込みロック状態の制御
 */
#define SIL_PRE_LOC		volatile uint32 TOPPERS_psw
#define SIL_LOC_INT()	TOPPERS_psw = TOPPERS_disint()
#define SIL_UNL_INT()	TOPPERS_enaint(TOPPERS_psw)
#define SIL_LOC_SPN()	(TOPPERS_psw = TOPPERS_spn_lock())
#define SIL_UNL_SPN()	(TOPPERS_spn_unlock(TOPPERS_psw))

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PRC_SIL_H */

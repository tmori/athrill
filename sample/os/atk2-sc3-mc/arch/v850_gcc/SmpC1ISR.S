/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
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
 *  $Id: SmpC1ISR.S 117 2014-12-10 03:58:03Z t_ishikawa $
 */

#define TOPPERS_MACRO_ONLY

#define UINT_C(val)	(val)
#define CAST(type, val)	(val)	/* 型キャストを行うマクロ */

#include "v850asm.inc"
#include "target_test.h"
#include "kernel/kernel_impl.h"
#include "target_timer.h"
#include "v850.h"

.global C1ISRMainSmpC1ISR
.global _C1ISRMainSmpC1ISR

C1ISRMainSmpC1ISR:
_C1ISRMainSmpC1ISR:

	/* レジスタの保存 */
	addi    -80, sp, sp
	st.w    ep, 12[sp]
	mov     sp, ep 
	sst.w   r1, 72[ep]
	sst.w   r6, 68[ep]
	sst.w   r7, 64[ep]
	sst.w   r8, 60[ep]
	sst.w   r9, 56[ep]
	sst.w	r10, 52[ep]
	sst.w	r11, 48[ep]
	sst.w	r12, 44[ep]
	sst.w   r13, 40[ep]
	sst.w   r14, 36[ep]
	sst.w   r15, 32[ep]
	sst.w   r16, 28[ep]
	sst.w   r17, 24[ep]
	sst.w   r18, 20[ep]
	sst.w   r19, 16[ep]
	sst.w   r31, 8[ep]

	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

    /* debug led4 on */
    Lea     P24, r6
    ld.h    0[r6], r7             /* P24 -> r7 */
    mov     ~0x0800, r8
    and     r7, r8
    st.h    r8, 0[r6]

	UNLOCK_ALL_INT      /* 全割込み解除 */

	/* 割込み発生前のcallevel_statをスタックに保存 */
	Lea     _callevel_stat, r6
	ld.h    0[r6], r7
	sst.w   r7, 76[ep]

	/* callevel_statにC1ISRをセット */

	Lea     TSYS_ISR1, r8
	or      r8, r7
	st.h    r7, 0[r6]

	/*
	 *  タイマ割込みをクリア
	 */
    Lea     ICTAUA0I(TAUA_CH7), r6
    ld.h    0[r6], r7
    mov     ~EIRFn, r8
    and      r8, r7
    st.h    r7, 0[r6]

/*	jarl	_c1isr_syslog, r31  FIXME */

	Lea     _callevel_stat, r6
	sld.w    76[ep], r7
	st.h    r7, 0[r6]

	LOCK_ALL_INT      /* 全割込み設定 */

   /* debug led4 off */
    Lea     P24, r6
    ld.h    0[r6], r7           /* P24 -> r7 */
    mov     0x0800, r8
    or     r7, r8
    st.h    r8, 0[r6]

	/* レジスタの復帰 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w   0[ep], r10
	ldsr    r10, eipsw
	sld.w   72[ep], r1
	sld.w   68[ep], r6
	sld.w   64[ep], r7
	sld.w   60[ep], r8
	sld.w   56[ep], r9
	sld.w   52[ep], r10
	sld.w   48[ep], r11
	sld.w   44[ep], r12
	sld.w   40[ep], r13
	sld.w   36[ep], r14
	sld.w   32[ep], r15
	sld.w   28[ep], r16
	sld.w   24[ep], r17
	sld.w   20[ep], r18
	sld.w   16[ep], r19
	sld.w    8[ep], r31
	addi    80, ep, sp
	sld.w   12[ep], r30
	eiret


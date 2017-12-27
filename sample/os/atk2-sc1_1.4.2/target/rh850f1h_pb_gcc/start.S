/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
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
 *  $Id: start.S 43 2014-07-19 15:33:29Z ertl-honda $
 */

/*
 *		カーネル対応のスタートアップモジュール（V850用）
 */

#define OMIT_INCLUDE_OS_CFG		/* Os_Cfg.hインクルード抑止 */

#include <v850asm.inc>

	.section .text
	.align  4
	.global __start
__start:
	di                                  /* 割り込み禁止 */
	/*
	 * 各種ポインタの初期化(SP/TP/EP/GP/CTBP)
	 */
	Lea     _ostkpt, r3
	ld.w    0[r3], r3
	Lea     __tp, tp
	Lea     __ep, ep
	Lea     __gp, gp
	Lea     __ctbp, r6
	ldsr    r6, 20

#ifdef TOPPERS_USE_HFLOAT
	/* 
	 *  FPUを有効に
	 */
	stsr psw, r12       /* load psw */
	mov  0x00010000,r13 /* PSW.CU0(PSW[16]) = 1 */
	or   r13, r12
	ldsr r12, psw       /* store psw */
#endif /* TOPPERS_USE_HFLOAT */

	/* permit local ram access */
	mov 0xfffee600, r20 -- PEG #0 base
	mov 0x0001, r21 -- PEGG1SP val / All SPID is permit
	mov 0xffffffff, r22 -- PEGG1MK val / All Address Masked
	mov 0x00000077, r23 -- PEGG1BA val / Enable, Read/Write permit, Non-Lock, Base=0

	st.h    r21, 0x0c[r20]   -- set PEGGSP
	st.w    r22, 0x80[r20]   -- set PEGG1MK
	st.w    r23, 0x84[r20]   -- set PEGG1BA

	-- Store completion barrier
	ld.w    0x84[r20], r0
	syncp

	/*
	 *  hardware_init_hook の呼出し（0 でない場合）
	 *
	 *  ターゲットハードウェアに依存して必要な初期化処理がある場合
	 *  は，hardware_init_hook という関数を用意する．
	 */

	Lea     hardware_init_hook, r1
	cmp     r0, r1
	be      hardware_init_hook_end
#ifdef __v850e3v5__
	jarl    [r1], lp
#else /* __v850e3v5__ */
	jarl    hardware_init_hook, lp
#endif /* __v850e3v5__ */

hardware_init_hook_end:
	/*
	 *  bssセクションのクリア
	 *   アラインされているので4ずつ行く
	 */
	Lea     __bss_start, ep
	Lea     __bss_end, r6
bss_clear_start:
	cmp     ep, r6
	bl      bss_clear_end
	sst.w   r0, 0[ep]
	add     4, ep
	br      bss_clear_start
bss_clear_end:
#ifndef OMIT_DATA_INIT
	/*
	 *  dataセクションの初期化（ROM化対応）
	 *
	 *  __idata_start から __idata_end までを，__data_start 以降に
	 *  コピーする．
	 */
	Lea     __idata_start, r6
	Lea     __idata_end,   r7
	Lea     __data_start,  r8
data_init_start:
	cmp     r6, r7
	bl      data_init_end
	ld.w    0[r6], r9
	st.w    r9, 0[r8]
	add     4, r6
	add     4, r8
	br      data_init_start
data_init_end:
#endif /* OMIT_DATA_INIT */
	/*
	 *  software_init_hook を呼出し（0 でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *  理がある場合は，software_init_hook という関数を用意すれば
	 *  よい．
	 */
	Lea     software_init_hook, r1
	cmp     r0, r1
	be      software_init_hook_end
#ifdef __v850e3v5__
	jarl    [r1], lp
#else /* __v850e3v5__ */
	jarl    software_init_hook, lp
#endif /* __v850e3v5__ */

software_init_hook_end:


 	jarl    _target_hardware_initialize, r31
	jarl    _main, r31
	halt
#if defined(PE2_ONLY) || defined(PE1_ONLY) 
	jarl    __non_use_pe_start, r31

	.section .non_use_pe_rom
	.align  4
	.global  __non_use_pe_start
 __non_use_pe_start:
	nop
#ifdef PE2_ONLY
	/* IBDxxx の初期化 */
	mov 0x00010002, r2

	/* RLIN30 */
	mov 0xFFFFB88C, r3
	st.w r2, 0[r3]

	/* RLIN31 */
	mov 0xFFFFB9E8, r3
	st.w r2, 0[r3]

	/* TAUJ0I0 */
	mov 0xFFFFB940, r3
	st.w r2, 0[r3]
	/* TAUJ1I0 */
	mov 0xFFFFBAA0, r3
	st.w r2, 0[r3]
#endif /* PE2_ONLY */

	/* permit local ram access */
	mov 0xfffee600, r20 -- PEG #0 base
	mov 0x0001, r21 -- PEGG1SP val / All SPID is permit
	mov 0xffffffff, r22 -- PEGG1MK val / All Address Masked
	mov 0x00000077, r23 -- PEGG1BA val / Enable, Read/Write permit, Non-Lock, Base=0

	st.h    r21, 0x0c[r20]   -- set PEGGSP
	st.w    r22, 0x80[r20]   -- set PEGG1MK
	st.w    r23, 0x84[r20]   -- set PEGG1BA

	-- Store completion barrier
	ld.w    0x84[r20], r0
	syncp

__non_use_pe_start_1:
	halt
	br   __non_use_pe_start_1

#endif /* defined(PE2_ONLY) || defined(PE1_ONLY) */

	.end
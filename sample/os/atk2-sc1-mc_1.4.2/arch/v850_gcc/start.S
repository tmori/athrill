/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: start.S 852 2017-12-21 07:30:44Z ertl-honda $
 */

/*
 *		カーネル対応のスタートアップモジュール（V850用）
 */

#define OMIT_INCLUDE_OS_CFG		/* Os_Cfg.hインクルード抑止 */

#define TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* 符号無し整数型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */

#include <v850asm.inc>
#include "kernel_impl.h"

	.section .text
	.align  4
	.global __start
__start:
	di                                  /* 割り込み禁止 */

	/*
	 * 各種ポインタの初期化(SP/TP/EP/GP/CTBP)
	 */
	GET_CID r4
	shl     2, r4
	Lea     _ostkpt_table, r3
	add     r3, r4
	ld.w    0[r4], r3
	Lea     __tp, tp
	Lea     __ep, ep
	Lea     __gp, gp
	Lea     __ctbp, r6
	ldsr    r6, 20

	/*
	 *  hardware_init_hook の呼出し（0 でない場合）
	 *
	 *  ターゲットハードウェアに依存して必要な初期化処理がある場合
	 *  は，hardware_init_hook という関数を用意する．
	 */
	Lea     hardware_init_hook, r1
	cmp     r0, r1
	be      hardware_init_hook_end
	jarl    hardware_init_hook, lp
hardware_init_hook_end:

__start_1:
	/*
	 *  マスタコア以外は初期化待ち
	 */
	GET_CID r6
	cmp     r0, r6
	bne     slave_start

	/*
	 *  bssセクションのクリア
	 *   アラインされているので4ずつ行く
	 */
	Lea     __bss_start, ep
	Lea     __bss_end, r7
bss_clear_start:
	cmp     r7, ep
	bge     bss_clear_end
	sst.w   r0, 0[ep]
	add     4, ep
	br      bss_clear_start
bss_clear_end:

	/*
	 *  他のコアは停止とする
	 */
	Lea     _core_state_table, r6
	st.w    r0, 4[r6]   /* for PE2 */

	/*
	 *  マスタコアを起動済みとする
	 */
	mov     MAGIC_START, r7
	st.w    r7, 0[r6]

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
	cmp     r7, r6
	bge     data_init_end
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
	jarl    software_init_hook, lp

software_init_hook_end:

__start_4:
	/* jarl    _target_hardware_initialize, r31 */

	/*
	 * メモリ初期化終了の通知
	 */
	mov  MEV_BASE, r20
	mov  MAGIC_START, r7
	st.w r7, 0x00[r20]

__start_5:
	jarl    _main, r31
	halt

	/*
	 *  マスタコア以外のメモリ初期化待ち及びStartCore待ちルーチン
	 */
slave_start:
	/*
	 *  メモリ初期化待ち
	 */
	mov  MEV_BASE, r20
	ld.w 0x00[r20], r6
	cmp  r0, r6
	be   slave_start

	/*
	 *  StartCore待ち
	 */
	GET_CID r6
	Lea     _core_state_table, r9
	shl     2, r6
	add     r6, r9
	mov     MAGIC_START, r7
slave_start_1:
	ld.w    0[r9], r8
	cmp     r7, r8
	bne     slave_start_1
	br      __start_5

	.end

/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2010 by Meika Sugimoto
 * 
 *  上記著作権者は，以下の (1)〜(4) の条件か，Free Software Foundation 
 *  によって公表されている GNU General Public License の Version 2 に記
 *  述されている条件を満たす場合に限り，本ソフトウェア（本ソフトウェア
 *  を改変したものを含む．以下同じ）を使用・複製・改変・再配布（以下，
 *  利用と呼ぶ）することを無償で許諾する．
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
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，その適用可能性も
 *  含めて，いかなる保証も行わない．また，本ソフトウェアの利用により直
 *  接的または間接的に生じたいかなる損害に関しても，その責任を負わない．
 * 
 */

/*
 *	V850/JG2固有処理
 */

#include "v850asm.inc"
#define	TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)		(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */
#include "kernel_impl.h"

	.section .text , "ax"
	.align 2

/*
 *	ハードウェア状態の初期化
 *
 *	WDT2の停止とPLLの安定待ち
 */
	.globl	hardware_init_hook
hardware_init_hook:
	/* WDT2の停止 */
	mov		WDTM2 , r10
	st.b	r0 , 0[r10]
	/* リングオシレータの停止 */
	mov		RCM , r10
	mov		1 , r11
	st.b	r11 , 0[r10]

wait_pll_lock:
	ld.b 0[r11] , r12
	cmp 1 , r12
	be wait_pll_lock
	/*
	 * システムウェイトコントロールレジスタ設定、0か1を
	 * 使用するメインクロックに応じて設定する
	 */
	movea	0x01 ,	r0 , r12
	mov		VSWC , r13
	st.b	r12 , 0[r13]
	/* クロック設定、PLLクロックをメインクロックに */
	movea	0x02 , r0 , r12
	mov		PLLCTL , r13
	ld.bu	0[r13] , r14
	or		r12 , r14
	st.b	r14 , 0[r13]
	nop
	nop
	nop
	nop
	nop
	jmp	lp

	.end

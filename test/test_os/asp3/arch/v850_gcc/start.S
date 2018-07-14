/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2000-2002 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 * 
 *  Copyright (C) 2005 by Freelines CO.,Ltd
 * 
 *  Copyright (C) 2010 by Meika Sugimoto
 *
 *  上記著作権者は，以下の (1)～(4) の条件か，Free Software Foundation
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

#define	TOPPERS_MACRO_ONLY
#include "kernel_impl.h"
#include "v850asm.inc"

	.section	".text" , "ax"
	.align	4

/*
 *	V850用スタートアップファイル
 */
	.global	hardware_init_hook
	
	.globl	_start
_start:
	di

	/*
	 * 各種ポインタの初期化(SP/TP/EP/GP/CTBP)
	 */
	Lea		__kernel_istkpt, r3
	ld.w	0[r3], r3

	Lea		__text , tp
	Lea		__ep , ep
	Lea 	__gp , gp
	Lea		__ctbp , r6
	ldsr	r6 , 20
	
	/*
	 *  hardware_init_hook の呼出し（0 でない場合）
	 *
	 *  ターゲットハードウェアに依存して必要な初期化処理がある場合
	 *  は，hardware_init_hook という関数を用意する．
	 */

	Lea		hardware_init_hook, r1
	cmp		r0, r1
	be		hardware_init_hook_end
	jarl	hardware_init_hook, lp
hardware_init_hook_end:
	
	/*
	 *  bssセクションのクリア
	 *   アラインされているので4ずつ行く
	 */
	Lea		__bss_start, ep
	Lea		__bss_end, r6
bss_clear_start:
	cmp		ep, r6
	be		bss_clear_end
	sst.w	r0, 0[ep]
	add		4, ep
	br		bss_clear_start
bss_clear_end:

	/*
	 *  dataセクションの初期化（ROM化対応）
	 *
	 *  __idata_start から __idata_end までを，__data_start 以降に
	 *  コピーする．
	 */
	Lea		__idata_start, r6
	Lea		__idata_end,   r7
	Lea		__data_start,  r8
data_init_start:
	cmp		r6, r7
	be		data_init_end
	ld.w	0[r6], r9
	st.w	r9, 0[r8]
	add		4, r6
	add		4, r8
	br		data_init_start
data_init_end:

	/*
	 *  software_init_hook を呼出し（0 でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *  理がある場合は，software_init_hook という関数を用意すれば
	 *  よい．
	 */

	Lea		software_init_hook, r1
	cmp		r0, r1
	be		software_init_hook_end
	jarl	software_init_hook, r31
software_init_hook_end:

	/*
	 *  カーネルを起動する．
	 */
	.globl	_sta_ker
	jarl	_sta_ker, r31
	halt

	.end

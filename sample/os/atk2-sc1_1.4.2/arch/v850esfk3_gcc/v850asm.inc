/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2000-2002 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
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
 *  @(#) $Id: v850asm.inc,v 1.8 2007/01/05 02:10:17 honda Exp $
 */

/*
 *  V850アセンブラ用いろいろ定義ファイル
 */
 #define TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* 符号無し整数型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */

#include "kernel_impl.h"

	/*
	 * ラベル宣言 Flabel
	 *
	 *   関数名として使うラベル割付を行う
	 */
	.macro	Flabel name
	.type	\name,@function
\name:
	.endm

#define FLABEL(name) Flabel name

#define AMARG(name)  \name

	/*
	 * ラベルロード Lea (Load Effective Address)
	 */
	 
	.macro	Lea name, destination
	movhi	hi(\name), r0, \destination
	movea	lo(\name), \destination, \destination
	.endm


	/*
	 * 関数宣言 Function
	 *
	 *   関数名として使うラベル割付と 諸設定を行う
	 */
	 
	.macro	Function name
	.text
	.align	4
	.globl	\name
	.type	\name,@function
\name:
	.endm

	/*
	 * ラベル宣言 Label
	 *
	 *   関数名として使うラベル割付を行う
	 */
	
	.macro	Label name
	.globl	\name
	.type	\name,@function
\name:
	.endm

	/*
	 * スタック操作 - プッシュ
	 */
	
	.macro	Push source
	add	-4, sp
	st.w	\source, 0[sp]
	.endm
	
	/*
	 * スタック操作 - ポップ
	 */
	
	.macro	Pop dest
	ld.w	0[sp],\dest
	add	4, sp
	.endm

	/*
	 *  全割込み禁止マクロ
	 */
	.macro LOCK_ALL_INT
	    di
	.endm

	/*
	 *  全割込み禁止解除マクロ
	 */
	.macro UNLOCK_ALL_INT
	    ei
	.endm


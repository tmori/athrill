/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
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
 *  $Id: prc_support.S 550 2015-12-30 11:39:36Z ertl-honda $
 */

/*
 *		ターゲット依存情報の定義
 */
#include <v850asm.inc>
#include <v850esfk3.h>
#include "offset.h"


.macro GET_ISPR reg
	Lea		INTC_ISPR, AMARG(reg)
	ld.b	0[AMARG(reg)],AMARG(reg)
	andi    255, AMARG(reg), AMARG(reg)
.endm


/*
 *  OS割込み禁止マクロ
 *  ネストの一番外側で呼び出されることを想定している
 */
.macro OUTER_LOCK_OS_INT   reg12, reg13

	jarl	_set_intpri_lock_os_int, lp

    /* nested_lock_os_int_cnt = 1 */
    Lea     _nested_lock_os_int_cnt, AMARG(reg12)
    mov     1, AMARG(reg13)
    st.b    AMARG(reg13), 0[AMARG(reg12)]

.endm

/*
 *  OS割込み禁止解除マクロ
 *  ネストの一番外側で呼び出されることを想定している
 */
.macro OUTER_UNLOCK_OS_INT  reg12, reg13

    /* nested_lock_os_int_cnt = 0 */
    Lea     _nested_lock_os_int_cnt, AMARG(reg12)
    mov     r0, AMARG(reg13)
    st.b    AMARG(reg13), 0[AMARG(reg12)]

	jarl	_set_intpri_unlock_os_int, lp
.endm

	/*
	 * 例外入り口でのレジスタの保存マクロ
	 */
.macro INT_EXC_SAVEREG
	addi    -80 , sp , sp
	st.w    r30, 12[sp]  /* r30(ep) */
	mov     sp, ep 
	sst.w   r1, 76[ep]
	sst.w   r5, 72[ep]
	sst.w   r6, 68[ep]
	sst.w   r7, 64[ep]
	sst.w   r8, 60[ep]
	sst.w   r9, 56[ep]
	sst.w   r10, 52[ep]
	sst.w   r11, 48[ep]
	sst.w   r12, 44[ep]
	sst.w   r13, 40[ep]
	sst.w   r14, 36[ep]
	sst.w   r15, 32[ep]
	sst.w   r16, 28[ep]
	sst.w   r17, 24[ep]
	sst.w   r18, 20[ep]
	sst.w   r19, 16[ep]
	sst.w   r31, 8[ep]
.endm

	/*
	 * 割込み入り口でのレジスタの保存マクロ
	 */
.macro INT_EXC_SAVEREG_ISR
	st.w    r30, 12[sp]  /* r30(ep) */
	mov     sp, ep 
	sst.w   r1, 76[ep]
	sst.w   r5, 72[ep]
	sst.w   r6, 68[ep]
	sst.w   r7, 64[ep]
	sst.w   r8, 60[ep]
	sst.w   r9, 56[ep]
	sst.w   r14, 36[ep]
	sst.w   r15, 32[ep]
	sst.w   r16, 28[ep]
	sst.w   r17, 24[ep]
	sst.w   r18, 20[ep]
	sst.w   r19, 16[ep]
	sst.w   r31, 8[ep]
.endm


	/*
	 * 割込み例外入り口でのレジスタの復帰マクロ
	 */
.macro INT_EXC_RESTOREREG
	sld.w   76[ep], r1
	sld.w   72[ep], r5
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
	sld.w   12[ep], r30 /* r30(ep) */
.endm



	/*
	 * <TRACE> 6.8 CPU例外ハンドラ
	 *  CPU例外エントリ
	 */
	.global _ei_exception_entry
FLABEL(_ei_exception_entry)
	/*
	 * Caller-Savedレジスタ保存
	 */
	INT_EXC_SAVEREG

	/* 
	 * EIPCとEIPSWを保存
	 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

	/*
	 *  SP，EIIC，EIPCをレジスタ退避
	 */
	mov     sp, r9                      /* sp をr9に退避 */
	stsr    ecr, r19                    /* 例外要因 */
	andi	255, r19, r19
	stsr    eipc, r18                   /* 例外発生PC */

	/*
	 * プロテクションフックはOS処理レベルで実行するためOS割込み禁止状態にする
	 * OS割込み禁止状態で例外が発生する可能性があるため，ネスト管理を行う．
	 * 例外は次の条件で発生する
	 *  ・ISR1実行状態で発生した場合
	 *  ・ISR1実行状態以外で発生した場合
	 *  ・OS割込み解除状態で発生した場合
	 *  ・OS割込み禁止状態で発生した場合
	 */

	/*
	 * ISR1処理中で発生したかどうかチェックする
	 * カテゴリ1で例外が発生：ei_exception_1へジャンプ
	 * r5は，C1ISRで発生したかどうかのフラグで今後使用する．
	 */
	GET_ISPR r5
	Lea     _ispr_isr1_mask, r7
	ld.b    0[r7], r7
	andi	255, r7, r7
	and     r7, r5
	cmp     r0, r5
	bne     ei_exception_1

	/*
	 *  OS割込み禁止ネストカウント++
	 *  ※ISR1実行状態以外で発生した場合
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     1, r13
	bne     ei_exception_1

	/*
	 * OS割込み禁止(OS割込み禁止ネストカウント=1)
	 * ※OS割込み解除状態で発生した場合
	 */
	jarl	_set_intpri_lock_os_int, lp

FLABEL(ei_exception_1)
	/*
	 * カーネル起動中かどうかチェックする
	 *  カーネル起動していない場合に起きたCPU例外は，無限ループへ
	 */
	Lea   _kernel_kerflg, r12          /* kerflgがFALSEなら無限ループ */
	ld.b  0[r12], r12

	/*
	 * 無限ループ(カーネル起動していない場合)
	 */
	cmp   r0, r12
	be    infinity_loop

	/*
	 * 割込み・例外ネストカウント++
	 */
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	mov     1, r14                      /*  割込み・例外のネスト回数のインクリメント */
	add     r14, r13
	st.w    r13, 0[r12]

	/*
	 * 割込み・例外ネストカウント > 1 の場合は既にスタックポインタは
	 * 非タスクコンテキストになっているため，以下の切替処理はスキップする
	 */
	cmp     1, r13
	bne     ei_exception_2

	/*
	 * ISR1実行中のCPU例外の場合は，以下の切替処理はスキップする
	 */
	cmp     r0, r5
	bne     ei_exception_2

	/*
	 * スタックを非タスクコンテキスト用のスタックに切り換える
	 */
	Lea     _kernel_ostkpt, r12
	ld.w    0[r12], r12
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */

FLABEL(ei_exception_2)
	/*
	 * 全割込み禁止解除
	 */
	UNLOCK_ALL_INT

#ifdef CFG_USE_PROTECTIONHOOK
	/* 
	 * CPU例外発生時の情報を退避する
	 * スタック領域を確保(4×3バイト)
	 */
	addi    -12, sp, sp
	mov     sp, ep

	/* 
	 * CPU例外発生時のSPを変数保存
	 */
	Lea     _v850_cpu_exp_sp, r12
	ld.w    0[r12], r13
	sst.w   r13, 0[ep]
	st.w    r9, 0[r12]                     /* r9：SP */

	/* 
	 * CPU例外発生時の例外コードを変数保存
	 */
	Lea     _v850_cpu_exp_no, r12
	ld.w    0[r12], r13
	sst.w   r13, 8[ep]
	st.w    r19, 0[r12]                     /* r19：EIIC */

	/* 
	 * CPU例外発生時のPCを変数保存
	 */
	Lea     _v850_cpu_exp_pc, r12
	ld.w    0[r12], r13
	sst.w   r13, 4[ep]
	mov     r18, r17                        /* r18：EIPC */
	add     -4, r17
	st.w    r17, 0[r12]

#endif /* CFG_USE_PROTECTIONHOOK */

	/*
	 * プロテクションフック関数呼び出し
	 */
	Lea     E_OS_PROTECTION_EXCEPTION, r6
	jarl    _kernel_call_protectionhk_main, r31

#ifdef CFG_USE_PROTECTIONHOOK

	/*
	 * CPU例外発生時のSPの変数復帰
	 */
	Lea     _v850_cpu_exp_sp, r12
	sld.w   0[ep], r13
	st.w    r13, 0[r12]

	/*
	 * CPU例外発生時の例外コードの変数復帰
	 */
	Lea     _v850_cpu_exp_no, r12
	sld.w   8[ep], r13
	st.w    r13, 0[r12]

	/*
	 * CPU例外発生時のPCの変数復帰
	 */
	Lea     _v850_cpu_exp_pc, r12
	sld.w   4[ep], r13
	st.w    r13, 0[r12]


	/* スタック領域を解放する */
	addi    12, sp, sp
	mov		sp, ep
#endif /* CFG_USE_PROTECTIONHOOK */

FLABEL(ei_ret_exc)
	/*
	 * 全割込み禁止
	 */
	LOCK_ALL_INT

	/*
	 *  割込み・例外ネストカウント--
	 */
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	mov     -1, r14
	add     r14, r13
	st.w    r13, 0[r12]

	/*
	 *  OS割込み禁止状態の解除
	 */
	/*
	 * ISR1実行状態で発生した場合は
	 * (1)スタックポインタの入れ替えをしない
	 * (2)OS割込み禁止状態の解除をしない
	 */
	cmp     r0, r5
	bne     ei_ret_exc_2

	/*
	 *  ネスト回数が1以上なら割込み元がISRなため例外発生元へ復帰
	 *  スタックポインタの入れ替えしない
	 */
	cmp     r0, r13
	bne     ei_ret_exc_1

	/*
	 *  スタックポインタをタスクに戻す
	 */
	ld.w    0[sp], sp

FLABEL(ei_ret_exc_1)
	/*
	 *  OS割込み禁止ネストカウント--
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     -1, r14
	add     r14, r13
	st.b    r13, 0[r12]

	/*
	 * OS割込み禁止ネストカウント＞０の場合は，OS割り込み禁止解除しない
	 */
	cmp     r0, r13
	bne     ei_ret_exc_2

	/*
	 * OS割込み禁止解除
	 */
	 jarl	_set_intpri_unlock_os_int, lp

FLABEL(ei_ret_exc_2)
	/*
	 * EIPCとEIPSWを復帰
	 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w    0[ep], r10
	ldsr    r10, eipsw

	/*
	 * Caller-Savedレジスタ復帰
	 */
	INT_EXC_RESTOREREG

	reti


	/*
	 *  カーネル起動していない場合に起きたCPU例外の無限ループ
	 */
FLABEL(infinity_loop)
	br infinity_loop

	/*
	 * <TRACE> 6.6.1 割込みハンドラの入口処理
	 *
	 *  C2ISR割り込みエントリ
	 * [本関数の呼び出し条件]
	 *  sp -= 80
	 *  sp[52] = r10
	 *  sp[48] = r11
	 *  sp[44] = r12
	 *  sp[40] = r13
	 *  r10 = 割込みハンドラアドレス
	 *  r11 = 割込み優先度(内部表現)
	 *  r12 = 割込みハンドラ番号
	 *  r13 = 割込み対象ISRCBアドレス
	 */
	.global _interrupt_isr2
FLABEL(_interrupt_isr2)
	/*
	 * Caller-Savedレジスタ保存
	 */
	INT_EXC_SAVEREG_ISR

	/* 
	 * EIPCとEIPSWを保存
	 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

	/* 
	 *  割込み発生前の割込みレベルを保存
	 */
	Lea     _current_iintpri, r6
	ld.b    0[r6], r19
	
	/* 
	 *  _current_iintpriを更新
	 */
	st.b    r11,0[r6]

	/*
	 *  割込み・例外ネストカウント++
	 */
	Lea     _except_nest_cnt, r14
	ld.w    0[r14], r6
	add     1,  r6
	st.w    r6, 0[r14]
	cmp     1,  r6
	bne     interrupt_1     /* ネストしている場合はスタック切替無し */

	/*
	 *  非タスクコンテキスト切替
	 */
	mov     sp , r14
	Lea     _kernel_ostkpt, sp
	ld.w    0[r3], r3
	add     -4, sp
	st.w    r14, 0[sp]

FLABEL(interrupt_1)
    /*
     *  割込み発生時の割込み優先度マスクをスタックに保存
     */
    add     -4, sp
    st.w    r19, 0[sp]

	/*
	 *  全割込み禁止解除
	 */
	UNLOCK_ALL_INT

	/*
	 *  _callevel_stat 保存
	 */
	Lea     _kernel_callevel_stat, r6
	ld.h    0[r6], r7
	Push    r7

	/*
	 *   _callevel_stat にISR2を設定
	 */
	mov     TCL_ISR2, r8
	or      r8, r7
	st.h    r7, 0[r6]

	/*
	 * p_runisr 保存
	 */
	Lea     _p_runisr, r6
	ld.w    0[r6], r7
	Push    r7

	/*
	 * p_runisr 設定
	 */
	Lea     _p_runisr, r8
	st.w    r13, 0[r8]

	/*
	 *  割込みハンドラ呼び出し
	 */
	Lea     end_int_handler, r31
	jmp     [r10]

FLABEL(end_int_handler)

	/*
	 * ISR2の不正終了チェック
	 */
	jarl    _kernel_exit_isr2, r31

	/*
	 * p_runisr の復帰
	 */
	Lea     _p_runisr, r7
	Pop     r6
	st.w    r6, 0[r7]

	/*
	 * _callevel_stat の復帰
	 */
	Lea     _kernel_callevel_stat, r7
	Pop     r6
	st.h    r6, 0[r7]

/*
 *
 *  割込みハンドラ出口処理
 */
FLABEL(ret_int)
	/*
	 * _except_nest_cntの操作とスタック切り替えの間はOS割込み禁止にする
	 * 必要がある
	 * この先，割込み先へのリターンか，遅延ディスパッチに分岐する．
	 * 割込み先へのリターンには全割込みが必要であり，遅延ディスパッチ時には
	 * ネスト管理のOS割込み禁止にする必要があるため，ここでは一旦全割込み禁
	 * 止とする
	 */

	/*
	 * 全割込み禁止
	 */
	LOCK_ALL_INT

	/*
	 *  割込み発生時の割込み優先度マスクをスタックから復帰
	 */
	ld.w    0[sp], r19
	add     4, sp
	Lea     _current_iintpri, r6
	st.b    r19, 0[r6]              /* current_iintpriを復帰 */

	/*
	 *  割込み・例外ネストカウント--
	 */
	Lea     _except_nest_cnt, r6
	ld.w    0[r6], r7
	add     -1, r7
	st.w    r7, 0[r6]

	/*
	 *  ネスト回数が0なら，割込み元がタスク(ret_int_1へ移動する)
	 */
	cmp     r0, r7
	be      ret_int_task

	/*
	 *  ISR2割込みからのリターン時は割込み元が
	 *  ISR2であるためそのままリターン
	 */
	br      ret_int_1

	/*
	 *  割込み元がタスクの場合
	 */
FLABEL(ret_int_task)
	/*
	 * タスクコンテキスト切替
	 */
	ld.w    0[sp], sp                       /* スタックポインタを戻す     */

	/* 割込み優先度マスクを変えるAPIがないためATK2では割込み優先度マスクの復帰処理は不要 */

	/*
	 *  割込みハンドラ実行時には，ISPRで優先度が自動的に上がるため，
	 *  PMRは変更していないが，割り込みハンドラ内で x_nested_unlock_os_int() を
	 *  呼び出すとPMRが書き換わる．ディスパッチが必要でない場合は，ret_int_1で
	 *  PMRを元の値に戻す．ディスパッチが必要な場合は，OS割り込み禁止とするため，
	 *  ここで，PMRを更新する必要はない．
	 */

	/*
	 *  ディスパッチの必要があるかチェック
	 */
	Lea     _kernel_p_runtsk, r6
	ld.w    0[r6], r6
	Lea     _kernel_p_schedtsk, r7
	ld.w    0[r7], r7
	cmp     r6, r7
	be      ret_int_1      /* ディスパッチの必要がなければ割込み元へ戻る(ret_int_1へ移動する) */

	/*
	 *  ここでは，戻り先がタスクであり，スタックは，タスクスタックに
	 *  スクラッチレジスタのみが保存された状態になっている．また，
	 *  全割込みを禁止した状態となっている
	 */

	/*
	 *  OS割込み禁止状態に移行する
	 *
	 *  この時点でOS割込み禁止状態とするのは，dispatcherを実行する際に
	 *  OS割込み禁止状態になっている必要があるためである
	 *
	 *  (モデル上の)割込み優先度マスクは0であるため，ISR1以外の割込みは全て禁止する
	 */
	OUTER_LOCK_OS_INT r12, r14

	/*
	 * 全割込み禁止解除
	 * ret_int で全割込み禁止状態としたため解除
	 */
	UNLOCK_ALL_INT

	/*
	 * Callee-Savedレジスタ保存
	 */
	addi    -40, sp, sp
	mov     sp, ep
	sst.w   r20, 36[ep]
	sst.w   r21, 32[ep]
	sst.w   r22, 28[ep]
	sst.w   r23, 24[ep]
	sst.w   r24, 20[ep]
	sst.w   r25, 16[ep]
	sst.w   r26, 12[ep]
	sst.w   r27, 8[ep]
	sst.w   r28, 4[ep]
	sst.w   r29, 0[ep]

	/* 
	 * スタックポインタを自タスク（p_runtsk）のTCBに保存する
	 */
	Lea     _kernel_p_runtsk, ep
	sld.w   0[ep], ep
	sst.w   sp, TCB_sp[ep]
	
	/*
	 * ret_int_rを，実行再開番地として自タスクのTCBに保存する
	 */
	Lea     ret_int_r, r6
	sst.w   r6, TCB_pc[ep]
	
	/*
	 * タスクディパッチする
	 * ※ISPRをクリアするために，retiを使用すること
	 */
	ldsr	r0, eipsw	/* 割込みは許可状態でよい */
	Lea		dispatcher, r10
	ldsr	r10, eipc
	reti

	/*
	 * <TRACE> 6.6.1 割込みハンドラの出口処理
	 *
	 *  割込み・例外でコンテキスト保存した場合の復帰ルーチン
	 */
FLABEL(ret_int_r)
	/*
	 * Callee-Savedレジスタ復帰
	 */
	mov     sp, ep
	sld.w   36[ep], r20
	sld.w   32[ep], r21
	sld.w   28[ep], r22
	sld.w   24[ep], r23
	sld.w   20[ep], r24
	sld.w   16[ep], r25
	sld.w   12[ep], r26
	sld.w   8[ep], r27
	sld.w   4[ep], r28
	sld.w   0[ep], r29
	addi    40, ep, sp

	/*
	 * 全割込み禁止
	 */
	LOCK_ALL_INT

	/*
	 * OS割り込み禁止解除
	 */
	OUTER_UNLOCK_OS_INT r12, r13
	
	/*
	 *  割込みからの復帰ルーチン(ret_int_1に移動する)
	 */
FLABEL(ret_int_1)
	/*
	 *  割込み優先度マスク(current_iintpri)に従って，割込みマスクを復帰する
	 */
	OUTER_UNLOCK_OS_INT  r7, r6
	
	/*
	 * EIPCとEIPSWを復帰
	 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w    0[ep], r10
	ldsr    r10, eipsw

	/*
	 * Caller-Savedレジスタ復帰
	 */
	INT_EXC_RESTOREREG

	reti


	/*
	 * <TRACE> 6.6.5 カーネル管理外の割込み
	 *
	 *  C1ISR割り込みエントリ
	 * [本関数の呼び出し条件]
	 *  sp -= 80
	 *  sp[52] = r10
	 *  sp[48] = r11
	 *  sp[44] = r12
	 *  sp[40] = r13
	 *  r10 = 割込みハンドラアドレス
	 *  r12 = 割込みハンドラ番号
	 */
	.global _interrupt_isr1
FLABEL(_interrupt_isr1)
	/* レジスタの保存 */
	INT_EXC_SAVEREG_ISR

	/* 
	 * EIPCとEIPSWを保存
	 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

	/*
	 * CIISRネストカウント++
	 */
	Lea     _c1isr_nest_cnt, r12
	ld.b    0[r12], r13
	mov     1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     1, r13
	bne     _interrupt_isr1_1

	/*
	 *  C1ISRコンテキスト切替
	 */
	mov     sp , r14
	Lea     _stkpt_c1isr, sp
	ld.w    0[r3], r3
	add     -4, sp
	st.w    r14, 0[sp]

FLABEL(_interrupt_isr1_1)
	/*
	 *  全割込み禁止解除
	 */
	UNLOCK_ALL_INT

	/*
	 *  _callevel_stat 保存
	 */
	Lea     _kernel_callevel_stat, r6
	ld.h    0[r6], r7
	Push    r7

	/*
	 *   _callevel_stat にISR1を設定
	 */
	Lea     TSYS_ISR1, r8
	or      r8, r7
	st.h    r7, 0[r6]

	/*
	 *  割込みハンドラ呼び出し
	 */
	Lea     end_c1isr_int_handler, r31
	jmp     [r10]

FLABEL(end_c1isr_int_handler)
	/*
	 * 全割込み禁止
	 */
	LOCK_ALL_INT

	/*
	 * _callevel_stat の復帰
	 */
	Lea     _kernel_callevel_stat, r7
	Pop     r6
	st.h    r6, 0[r7]


	/*
	 *  CIISRネストカウント--
	 */
	Lea     _c1isr_nest_cnt, r6
	ld.w    0[r6], r7
	add     -1, r7
	st.w    r7, 0[r6]
	cmp     r0, r7
	bne      _interrupt_isr1_2

	/*
	 * OSコンテキスト切替
	 */
	ld.w    0[sp], sp                       /* スタックポインタを戻す     */

FLABEL(_interrupt_isr1_2)
	/*
	 * EIPCとEIPSWを復帰
	 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w    0[ep], r10
	ldsr    r10, eipsw

	/*
	 * Caller-Savedレジスタ復帰
	 */
	INT_EXC_RESTOREREG

	reti



/*
 * <TRACE> (6-5-3-1) void dispatch(void)
 * タスクディスパッチ
 */
	.global _dispatch
FLABEL(_dispatch)
	/*
	 *  このルーチンは，タスクコンテキスト・OS割込み禁止状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる
	 */
	addi    -48, sp, sp
	st.w    r30,  4[sp] /* r30(ep) */
	mov     sp,  ep
	sst.w   r20, 44[ep]
	sst.w   r21, 40[ep]
	sst.w   r22, 36[ep]
	sst.w   r23, 32[ep]
	sst.w   r24, 28[ep]
	sst.w   r25, 24[ep]
	sst.w   r26, 20[ep]
	sst.w   r27, 16[ep]
	sst.w   r28, 12[ep]
	sst.w   r29, 8[ep]
	sst.w   r31, 0[ep]

	/* スタックと次の起動番地を保存 */
	Lea     _kernel_p_runtsk, ep
	sld.w   0[ep], ep
	sst.w   sp, TCB_sp[ep]
	Lea     dispatch_r, r10
	sst.w   r10, TCB_pc[ep]

	br      dispatcher

	/* 
	 * <TRACE> (6-5-3-1) void dispatch(void):スクラッチレジスタを除くすべてのレジスタをスタックから復帰する
	 * ディスパッチャの復帰ルーチン
	 */
FLABEL(dispatch_r)
	mov     sp, ep
	sld.w   44[ep], r20
	sld.w   40[ep], r21
	sld.w   36[ep], r22
	sld.w   32[ep], r23
	sld.w   28[ep], r24
	sld.w   24[ep], r25
	sld.w   20[ep], r26
	sld.w   16[ep], r27
	sld.w   12[ep], r28
	sld.w    8[ep], r29
	sld.w    0[ep], r31
	addi    48, ep, sp
	ld.w     4[ep], r30  /* r30(ep) */
	jmp     [lp]

/*
 * <TRACE> (6-5-4-1) void start_dispatch(void)
 *  ディスパッチャの動作開始
 */
	.global _start_dispatch
FLABEL(_start_dispatch)
	/*
	 *  このルーチンは，カーネル起動時に，すべての割込みを禁止した状態
	 * （全割込み禁止状態と同等）で呼び出される．また，割込みモード（非
	 *  タスクコンテキストと同等）で呼び出されることを想定している
	 *
	 *  dispatcherは，OS割込み禁止状態で呼び出す．
	 */
	/* OS割込み禁止状態 */
	
	OUTER_LOCK_OS_INT r12, r13
	/* 全割込み禁止解除状態へ */
	UNLOCK_ALL_INT
	br    dispatcher_0

	/*
	 * <TRACE> (6-5-5-1) void exit_and_dispatch(void)
	 *  現在のコンテキストを捨ててディスパッチ
	 */
	.global _exit_and_dispatch
FLABEL(_exit_and_dispatch)
	/* ディスパッチャ本体（dispatcher）へ */
	br	dispatcher

/*
 * <TRACE> 6.5.2 ディスパッチャ本体
 */
	.global dispacher
FLABEL(dispatcher)
	/*
	 *  PostTaskHookの呼び出し
	 */
#ifdef CFG_USE_POSTTASKHOOK
	Lea     _PostTaskHook, r20
	cmp     r20, r0
	be      dispatcher_0
FLABEL(posttaskhook_call)
	Lea     _kernel_call_posttaskhook, r6
	jarl    _stack_change_and_call_func_1, r31
#endif /* CFG_USE_POSTTASKHOOK */

FLABEL(dispatcher_0)
	/*
	 *  このルーチンは，タスクコンテキスト・OS割込み禁止状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる．実行再開番地へもこの状態のまま分岐する
	 */
#ifdef LOG_DSP_ENTER
	/*
	 * ディスパッチャ入口ログ出力
	 */
	Lea     _kernel_p_runtsk, r21
	ld.w    0[r21], r21
	cmp     r0, r21
	be      dispatcher_0_1
	mov     r21, r6                            /* 引数に_p_runtskを指定 */
	jarl    _log_dsp_enter, r31
FLABEL(dispatcher_0_1)
#endif	/* LOG_DSP_ENTER */

	/*
	 * 全割込み禁止解除(callevel_statにTSYS_DSALLINTが立っている場合)
	 */
	Lea     _kernel_callevel_stat, r21
	ld.h    0[r21], r22
	Lea     TSYS_DISALLINT, r20
	and     r22, r20
	be      dispatcher_1

	Lea     ~TSYS_DISALLINT, r20
	and     r22, r20
	st.h    r20, 0[r21]
	UNLOCK_ALL_INT                             /* 全割込み禁止解除状態へ */

	/*
	 *  例外（割込み/CPU例外）のネスト回数はstart_dispatchや，dispatcher_3:
	 *  から来た場合は'1'となっているため，タスクスタックに切り換え後，
	 *  '0'に初期化する．
	 */
FLABEL(dispatcher_1)
	/*
	 * 最高優先度タスク(p_schedtsk)を現在実行中タスク(p_runtsk)に設定
	 */
	Lea     _kernel_p_runtsk, r21
	Lea     _kernel_p_schedtsk, r20
	ld.w    0[r20], r22
	st.w    r22, 0[r21]
	/*
	 * dispatcher_3 に移動する(ディスパッチ先がない場合)
	 */
	cmp     r22, r0
	be      dispatcher_3

	/*
	 * 自タスク（p_runtsk）のTCBからスタックポインタを復帰する
	 */
	mov     r22, ep 
	sld.w   TCB_sp[ep], sp
	/*
	 * 例外（割込み/CPU例外）のネスト回数に０を設定
	 */
	Lea     _except_nest_cnt, r20
	st.w    r0, 0[r20]

#ifdef LOG_DSP_LEAVE
	/*
	 * ディスパッチャ出口ログ出力
	 */
	ld.w    0[r21], r6                          /* 引数に_p_runtskを指定 */
	jarl    _log_dsp_leave, r31
#endif	/* LOG_DSP_LEAVE */

#ifdef CFG_USE_PRETASKHOOK
	/*
	 * プレタスクフック実行
	 */
	Lea     _PreTaskHook, r20
	cmp     r20, r0
	be      dispatcher_2

FLABEL(pretaskhook_call)
	Lea     _kernel_call_pretaskhook, r6
	jarl    _stack_change_and_call_func_1, r31
#endif /* CFG_USE_PRETASKHOOK */

FLABEL(dispatcher_2)
	/*
	 * 自タスクのTCBから実行再開番地を復帰し，そこへ分岐する
	 */
	Lea     _kernel_p_runtsk, r20
	ld.w    0[r20], r22
	mov     r22, ep
	sld.w   TCB_pc[ep], r31
	jmp     [r31]

FLABEL(dispatcher_3)
	/*
	 * OS割込み禁止状態の解除と，非タスクコンテキスト実行状態への
	 * 準備をする
	 *
	 *  ここで非タスクコンテキストに切り替える（sp＝非タスクコンテキス
	 *  ト用スタックポインタ，except_nest_cnt＝1)のは，OS割込み禁止解
	 *  除後に発生する割込み処理にどのスタックを使うかという問題の解決
	 *  と，割込みハンドラ内でのタスクディスパッチの防止という2つの意
	 *  味がある
	 */
	Lea     _kernel_ostkpt, r20		/* 非タスクコンテキスト用のスタックの読み込みスタックポインタの入れ替え */
	ld.w    0[r20], r3

	/*
	 * 例外（割込み/CPU例外）のネスト回数に１を設定
	 */
	Lea     _except_nest_cnt, r20
	mov     1, r21
	st.w    r21, 0[r20]

	/*
	 *  OS割込み禁止解除状態へ
	 *  割込みを許可し，非タスクコンテキスト実行状態とし割込みを待つ
	 *
	 *  プロセッサを割込み待ちに移行させる処理と，割込み許可とは，不可
	 *  分に行なう必要がある
	 *  これを不可分に行なわない場合，割込みを許可した直後に割込
	 *  みが入り，その中でタスクが実行可能状態になると，実行すべきタス
	 *  クがあるにもかかわらずプロセッサが割込み待ちになってしまう．
	 *  不可分に行うため，diによりコアでの割り込みを禁止した状態で，
	 *  INTCの割り込み優先度マスクを下げてからhaltを呼び出す．
	 *  割り込みが発生した場合は，haltから復帰して次のeiでベクタに
	 *  ジャンプする．
	 *
	 *  割込み待ちの間は，p_runtskをNULL（＝0）に設定しなければならない
	 *  このように設定しないと，割込みハンドラからGetTaskIDを呼び出
	 *  した際の動作が仕様に合致しなくなる
	 */

	di
	OUTER_UNLOCK_OS_INT r12, r13
	halt
	/*
	 * HALT命令後には，NOP命令を5命令以上挿入する必要がある
	 * 4.3.3 スタンバイ機能の説明：注意1を参照のこと
	 */
	nop
	nop
	nop
	nop
	nop
	ei

	/*
	 *  OS割込み禁止状態へ
	 *  (モデル上の)割込み優先度マスクは0であるため，ISR1以外の割込みは全て禁止する
	 *  スタックは非タスクコンテキストのスタックを使用しているため，_except_nest_cntは，
	 *  '1'のままとする
	 */
	OUTER_LOCK_OS_INT r12, r13
	br      dispatcher_1






/*
 * <TRACE> (6-5-6-1) void activate_context(TCB *p_tcb)
 *  タスク開始時処理
 *
 *  dispatcherから呼び出される
 */
	.global _start_r
FLABEL(_start_r)
	/*
	 *  OS割込み禁止解除前に現在優先度(TCB.curpri)を実行優先度(TINIB.exepri)に
	 *  変更(内部リソースを獲得)
	 */
	Lea     _kernel_p_runtsk, ep
	ld.w    0[ep], ep
	ld.w    TCB_p_tinib[ep], r22
	ld.b    TINIB_exepri[r22], r21			/* TINIB.exepri -> TCB.curpri */
	st.b    r21, TCB_curpri[ep]

	/*
	 *  OS割込み禁止解除状態へ
	 *  (モデル上の)割込み優先度マスクは0であるため，有効な割込みは全て許可する
	 */

	OUTER_UNLOCK_OS_INT r12, r13

	/*
	 *  タスクが不正終了した場合は保護処理を行うよう
	 *  戻りアドレスにexit_taskを設定
	 */
	Lea     _kernel_exit_task, r31

	ld.w    TCB_p_tinib[ep], r22
	ld.w    TINIB_task[r22], r10
	jmp     [r10]

	/*
	 * <TRACE> (6-9-1) void call_errorhook(StatusType ercd, OSServiceIdType svcid)
	 * <TRACE> (6-9-2) void call_shutdownhook(StatusType ercd)
	 */
	.global _stack_change_and_call_func_1 
	.global _stack_change_and_call_func_2
FLABEL(_stack_change_and_call_func_1)
FLABEL(_stack_change_and_call_func_2)

	push    lp
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	add     1, r13
	st.w    r13, 0[r12]
	cmp     1, r13
	bne     _stack_change_and_call_func_1_1

	/* スタックポインタの保存と入れ替え */
	Lea     _kernel_ostkpt, r12
	ld.w    0[r12], r12
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */
    
FLABEL(_stack_change_and_call_func_1_1)
	Lea     stack_change_and_call_func_r, lp  /* 戻りアドレス */
	mov     r6, r12
	mov     r7, r6
	mov     r8, r7
	jmp     [r12]

FLABEL(stack_change_and_call_func_r)
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	add     -1, r13
	st.w    r13, 0[r12]
	cmp     r0, r13
	bne     stack_change_and_call_func_r_1

	ld.w    0[sp], sp                    /* スタックポインタの復帰     */
FLABEL(stack_change_and_call_func_r_1)
	pop     lp
	jmp     [lp]


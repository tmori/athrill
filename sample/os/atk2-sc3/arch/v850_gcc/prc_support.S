/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2016 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
 *  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: prc_support.S 189 2015-06-26 01:54:57Z t_ishikawa $
 */

/*
 *		ターゲット依存情報の定義
 */
#include <v850asm.inc>
#include "offset.h"
#include "Os_Cfg_asm.inc"

#ifdef USE_ASMCONFIG_INC
#include "asm_config.inc"
#endif /* USE_ASMCONFIG_INC */

#ifdef __v850e2v3__

/*
 *  V850E2用の割込みコントローラ操作ルーチン
 */
.macro GET_ISPR reg
	mov		ISPR_H, AMARG(reg)
	ld.h	0[AMARG(reg)],AMARG(reg)
.endm

.macro CLEAR_ISPR reg1, reg2
	mov		ISPC_H,AMARG(reg1)
	mov		0xffff,AMARG(reg2)
	st.h	AMARG(reg2),0[AMARG(reg1)]
	mov		ISPR_H, AMARG(reg1)
	st.h	r0,0[AMARG(reg1)]
.endm

.macro SET_PMR reg_val, reg_tmp
	mov     PMR, AMARG(reg_tmp)
	st.h    AMARG(reg_val), 0[AMARG(reg_tmp)]  /* set interrupt level */
	syncm
.endm

#endif /* __v850e2v3__ */

#ifdef __v850e3v5__

/*
 *  V850E3V5用の割込みコントローラ操作ルーチン
 */
.macro GET_ISPR reg
	stsr 10, AMARG(reg), 2
.endm

.macro SET_ISPR val, reg 
	movea AMARG(val),r0,AMARG(reg)
	ldsr AMARG(reg), 10, 2
	syncp
.endm

.macro SET_INTCFG val, reg 
	movea AMARG(val),r0,AMARG(reg)
	ldsr AMARG(reg), 13, 2
	syncp
.endm

.macro CLEAR_ISPR reg1, reg2
	SET_INTCFG 1, AMARG(reg1) /* ISPR を書き換え可能に */
	SET_ISPR   0, AMARG(reg1) /* ISPR のクリア */
	SET_INTCFG 0, AMARG(reg1) /* ISPR を書き換え禁止に(自動更新に) */
.endm

.macro GET_PMR reg
	stsr 11, AMARG(reg), 2
.endm

.macro SET_PMR reg_val, reg_tmp
	stsr psw, AMARG(reg_tmp)
	di
	ldsr AMARG(reg_val), 11, 2
	ldsr AMARG(reg_tmp), psw
	syncp
.endm

.macro GET_ICSR reg
	stsr sr12, AMARG(reg), 2
.endm

.macro SET_ICSR val, reg 
	movea AMARG(val),r0,AMARG(reg)
	ldsr \reg, sr12, 2
.endm

#endif /* !__v850e3v5__ */


/*
 *  OS割込み禁止マクロ
 *  ネストの一番外側で呼び出されることを想定している
 */
.macro OUTER_LOCK_OS_INT   reg12, reg13

    /* pmr_isr2_mask -> PMR */
    Lea     _pmr_isr2_mask, AMARG(reg13)
    ld.h    0[AMARG(reg13)], AMARG(reg13)
    SET_PMR AMARG(reg13), AMARG(reg12)

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

    /* pmr_setting_tbl[current_iintpri] -> PMR */
    Lea     _current_iintpri, AMARG(reg12)
    ld.bu   0[AMARG(reg12)], AMARG(reg12)
    shl     1, AMARG(reg12)

    Lea     _pmr_setting_tbl, AMARG(reg13)
    add     AMARG(reg12), AMARG(reg13)
    ld.h    0[AMARG(reg13)], AMARG(reg13)
    SET_PMR AMARG(reg13), AMARG(reg12)  /* set interrupt level */

.endm

/* =begin modified for SC3 */
/*
 *  OS割込み禁止マクロ
 *  ネストあり
 */
.macro NESTED_LOCK_OS_INT   reg12, reg13
    //.macrolocal increment_nest_lock

    /* nested_lock_os_int_cnt == 0 ? */
    Lea     _nested_lock_os_int_cnt, AMARG(reg13)
    ld.b    0[AMARG(reg13)], AMARG(reg12)
    cmp     r0, AMARG(reg12)
    bne     increment_nest_lock

    /* pmr_isr2_mask -> PMR */
    Lea     _pmr_isr2_mask, AMARG(reg13)
    ld.h    0[AMARG(reg13)], AMARG(reg13)
    SET_PMR AMARG(reg13), AMARG(reg12)  /* set interrupt level */
    Lea     _nested_lock_os_int_cnt, AMARG(reg13)
    mov     r0, AMARG(reg12)                /* nested_lock_os_int_cnt = 0 */

FLABEL(increment_nest_lock)
    /* nested_lock_os_int_cnt += 1 */
    add     1, AMARG(reg12)
    st.b    AMARG(reg12), 0[AMARG(reg13)]

.endm

/*
 *  OS割込み禁止解除マクロ
 *  ネストあり
 */
.macro NESTED_UNLOCK_OS_INT  reg12, reg13
    .macrolocal nested_unlock_exit

    /* nested_lock_os_int_cnt -= 1 */
    Lea     _nested_lock_os_int_cnt, AMARG(reg12)
    ld.b    0[AMARG(reg12)], AMARG(reg13)
    add     -1, AMARG(reg13)
    st.b    AMARG(reg13), 0[AMARG(reg12)]
    cmp     r0, AMARG(reg13)
    bne     nested_unlock_exit

    /* pmr_setting_tbl[current_iintpri] -> PMR */
    Lea     _current_iintpri, AMARG(reg12)
    ld.bu   0[AMARG(reg12)], AMARG(reg12)
    shl     1, AMARG(reg12)

    Lea     _pmr_setting_tbl, AMARG(reg13)
    add     AMARG(reg12), AMARG(reg13)
    ld.h    0[AMARG(reg13)], AMARG(reg13)
    SET_PMR AMARG(reg13), AMARG(reg12)  /* set interrupt level */

FLABEL(nested_unlock_exit)
.endm

/* =end modified for SC3 */

	/* 
	 * 割込み/例外の先頭で実行する処理 
	*/
.macro PRE_INT_EXC
#ifdef _RH850G3M_
	/* 一部中断型例外ハンドラでのみ必要だが一律実行する */
	syncp
#endif /* _RH850G3M_ */
.endm

	/*
	 * 割込み例外入り口でのレジスタの保存マクロ
	 */
.macro INT_EXC_SAVEREG workreg
#ifndef TOPPERS_USE_PUSHSP
#ifdef TOPPERS_USE_HFLOAT
	addi    -88 , sp , sp
#else /* !TOPPERS_USE_HFLOAT */
	addi    -84 , sp , sp
#endif /* TOPPERS_USE_HFLOAT */
	st.w    r30, 12[sp]  /* r30(ep) */
    /*
     * 割込み発生前のr2はeiwrに保存されている
     * ここで，r2はすでにワークレジスタとして
     * 使用中であるため，上書きしない
     * r2に入っている割込み番号を破壊しないように
     */
    stsr    AMARG(workreg), r30    /* r2->r30に復帰 */
	st.w    r30, 76[sp]
	mov     sp, ep 
	sst.w   r1, 80[ep]
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
#ifdef TOPPERS_USE_HFLOAT
	stsr    fpsr, r19     /* load FPSR */
	sst.w   r19, 84[ep]
#endif /* TOPPERS_USE_HFLOAT */

#else /* TOPPERS_USE_PUSHSP */
	pushsp  r1-r1
    /*
     * 割込み発生前のr2はeiwrに保存されている
     * ここで，r2はすでにワークレジスタとして
     * 使用中であるため，上書きしない
     * r2に入っている割込み番号を破壊しないように
     */
    stsr    AMARG(workreg), r1    /* r2->r1に復帰 */
	pushsp  r1-r1
#ifdef TOPPERS_USE_HFLOAT
	stsr    fpsr, r1     /* load FPSR */
	pushsp  r1-r1
#endif /* TOPPERS_USE_HFLOAT */
	pushsp  r5-r19
	pushsp  r30-r31
	addi    -8, sp , sp
	mov     sp, ep 
#endif /* TOPPERS_USE_PUSHSP */
.endm

	/*
	 * 割込み例外入り口でのレジスタの復帰マクロ
	 */
.macro INT_EXC_RESTOREREG workreg
#ifndef TOPPERS_USE_PUSHSP
#ifdef TOPPERS_USE_HFLOAT
	sld.w   84[ep], r19
	ldsr    r19, fpsr     /* store FPSR */
#endif /* TOPPERS_USE_HFLOAT */
	sld.w   80[ep], r1
	sld.w   76[ep], r2
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
#ifdef TOPPERS_USE_HFLOAT
	addi    88, ep, sp
#else /* !TOPPERS_USE_HFLOAT */
	addi    84, ep, sp
#endif /* TOPPERS_USE_HFLOAT */
	sld.w   12[ep], r30 /* r30(ep) */
    ldsr    r2, AMARG(workreg)    /* r2を退避 */

#else /* TOPPERS_USE_PUSHSP */
	addi    8, sp , sp
	popsp   r30-r31
	popsp   r5-r19
#ifdef TOPPERS_USE_HFLOAT
	popsp   r1-r1
	ldsr    r1, fpsr      /* store FPSR */
#endif /* TOPPERS_USE_HFLOAT */
    /*
     * 割込み発生前のr2はeiwrに復帰
     */
	popsp   r1-r1
    ldsr    r1, AMARG(workreg)     /* r2->eiwrに復帰 */
	popsp   r1-r1
#endif /* TOPPERS_USE_PUSHSP */
.endm


#ifdef __v850e3v5__
	/* 
	 * V850E3V5用ベクタ
	 * ここにRBASEが設定されることを想定（ここが電源投入時の開始アドレス）
	 * (EIレベル割込みはテーブル参照方式を使用するため本ベクタは使用しない)
	 */
	.section .reset.text
	.global __reset
__reset:
	jr __start
	.rept 6
	nop
	.endr
__syserr: /* 0x0010 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__hvtrap: /* 0x0020 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__fetrap: /* 0x0030 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__trap0:  /* 0x0040 */
	jr _ei_exception_entry
	.rept 6
	nop
	.endr
__trap1:  /* 0x0050 */
	jr _ei_exception_entry
	.rept 6
	nop
	.endr
__rie:    /* 0x0060 */
	jr _ei_exception_entry
	.rept 6
	nop
	.endr
__fppfpi: /* 0x0070 */
	jr _ei_exception_entry
	.rept 6
	nop
	.endr
__ucpop:  /* 0x080 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__mip:    /* 0x0090 */
	jr _fe_exception_entry_memory
	.rept 6
	nop
	.endr
__pie:   /* 0x00a0 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__debug:  /* 0x00b0 */
	jr __debug
	.rept 6
	nop
	.endr
__mae:    /* 0x00c0 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__rfu:    /* 0x00d0 */
	jr __rfu
	.rept 6
	nop
	.endr
__fenmi:  /* 0x00e0 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__feint:  /* 0x00f0 */
	jr _fe_exception_entry_cpu
	.rept 6
	nop
	.endr
__eiintn0:  /*（優先度0） 0x0100 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn1:  /*（優先度1） 0x0110 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn2:  /*（優先度2） 0x0120 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn3:  /*（優先度3） 0x0130 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn4:  /*（優先度4） 0x0140 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn5:  /*（優先度5） 0x0150 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn6:  /*（優先度6） 0x0160 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn7:  /*（優先度7） 0x0170 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn8:  /*（優先度8） 0x0180 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn9:  /*（優先度9） 0x0190 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn10: /*（優先度10） 0x01a0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn11: /*（優先度11） 0x01b0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn12: /*（優先度12） 0x01c0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn13: /*（優先度13） 0x01d0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn14: /*（優先度14） 0x01e0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr
__eiintn15: /*（優先度15） 0x01f0 */
	jr _default_int_handler
	.rept 6
	nop
	.endr

#endif /* __v850e3v5__ */

.macro FE_EXCEPTION_ENTRY
    //.macrolocal _fe_exception_macro
	/* 割込み/例外の先頭で実行する処理 */
	PRE_INT_EXC

    ldsr    r2, fewr                /* エラーコードを保存 */
    stsr    fepsw, r2
    shl     PSW_SV, r2              /* r2 << PSW_SV により例外発生時のモード取得 */
    mov     r0, r2
    bnc     _fe_exception_macro          /* 特権からの例外ならばスキップ */
    /*
     *  ユーザタスクからの例外
     */
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    st.w    sp, TCB_usp[r2]         /* uspをTCBに保存 */
    ld.w    TCB_p_tinib[r2], r2
    ld.w    TINIB_sstk_bottom[r2], sp         /* sspを復帰 */
    ori     0x1000, r0, r2             /* r2 = 0x1000 */
//FLABEL(_fe_exception_macro)
.endm


    .section .text_kernel
	/*
	 *  例外エントリ（feretでリターンする例外用）
     *  ここで，CY = FEPSW.IMP，r2 = error code
     *  r2は例外エントリでスタックに保存済み
     *  r2の最上位ビットに，例外元のコンテキストが特権かどうかを保存する
	 */
	.global _fe_exception_entry_cpu
FLABEL(_fe_exception_entry_cpu)
    FE_EXCEPTION_ENTRY
    ori     E_OS_PROTECTION_EXCEPTION, r2, r2
    br      fe_exception_0

	.global _fe_exception_entry
FLABEL(_fe_exception_entry_memory)
    FE_EXCEPTION_ENTRY
    ori     E_OS_PROTECTION_MEMORY, r2, r2

FLABEL(fe_exception_0)
    /* スクラッチレジスタを保存 */
    INT_EXC_SAVEREG fewr

	/* 多重割込みのため，現在のFEPCとFEPSWを保存 */
	stsr    fepc, r6
	sst.w   r6, 4[ep]
	stsr    fepsw, r6
	sst.w   r6, 0[ep]

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
	 * カテゴリ1の割込み以外で例外が発生したかの判断する
	 * カテゴリ1で例外が発生：fe_exception_1へジャンプ
	 */
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     fe_exception_1

	/*
	 *  ISR1実行状態以外で発生した場合
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     1, r13
	bne     fe_exception_1

	/*
	 * OS割込み解除状態で発生した場合
	 * OS割込み禁止状態とする
	 */
	Lea     _pmr_isr2_mask, r7
	ld.h    0[r7], r7
	SET_PMR r7, r6  /* pmr_isr2_mask -> PMR */

FLABEL(fe_exception_1)
	/*
	 *  割込み発生時に変更されるので割込み解除前に取得
	 */
	stsr    feic, r19                   /* 例外要因 */
	stsr    fepc, r18                   /* 例外発生PC */

	/* 
	 * C1ISRを受け付けるため，NPbit クリア 
	 */
	stsr    psw, r12
	mov     ~0x0080, r13
	and     r12, r13
	ldsr    r13, psw

FLABEL(fe_exception_2)
	/*
	 *  カーネル起動していない場合に起きたCPU例外は，無限ループへ
	 */
	Lea   _kerflg, r12          /* kerflgがFALSEなら無限ループ */
	ld.b  0[r12], r12
	cmp   r0, r12
FLABEL(infinity_loop_fe)
	be    infinity_loop_fe

	/*
	 * 多重割込みか判定
	 */
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	mov     1, r14                      /*  割込み・例外のネスト回数のインクリメント */
	add     r14, r13
	st.w    r13, 0[r12]

	mov     sp, r9                      /* sp をr9に退避 */

	cmp     1, r13
	bne     fe_exception_3              /* _nested_lock_os_int_cnt > 1 -> fe_exception_3 */

	/*
	 * C1ISR実行状態で発生した場合はスタックポインタの入れ替えしない
	 */
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     fe_exception_3

	/* スタックポインタの保存と入れ替え */
	Lea     __ostkpt, r12
	ld.w    0[r12], r12
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */

FLABEL(fe_exception_3)
    /* FEPSW.ID == 1の場合は全割込み禁止状態なので解除しない */
	stsr    fepsw, r12
    mov     0x20, r13
    tst     r12, r13
    bne     fe_exception_4
	UNLOCK_ALL_INT

FLABEL(fe_exception_4)
#ifdef CFG_USE_PROTECTIONHOOK
	/* v850_cpu_exp_*を退避するスペースを確保 */
	addi    -12, sp, sp

	/* v850_cpu_exp_spの退避と設定 */
	Lea     _v850_cpu_exp_sp, r12
	st.w    r9, 0[r12]
	st.w    ep, 0[sp]
	mov     sp, ep

	/* v850_cpu_exp_noの退避と設定 */
	Lea     _v850_cpu_exp_no, r12
	ld.w    0[r12], r13
	sst.w   r13, 8[ep]
	st.w    r19, 0[r12]                     /* r19 例外要因 */

	/* v850_cpu_exp_pcの退避と設定 */
	Lea     _v850_cpu_exp_pc, r12
	ld.w    0[r12], r13
	sst.w   r13, 4[ep]
	mov     r18, r17                        /* r18:fepc */
	add     -4, r17
	st.w    r17, 0[r12]

#endif /* CFG_USE_PROTECTIONHOOK */

	/*
	 * CPU例外が発生した場合，OSはE_OS_PROTECTION_EXCEPTIONをパラメータとして
	 * プロテクションフックを呼び出す
	 */
    /* =begin modified for SC3 */
    andi    0xefff, r2, r6 /* エラーコードを第1引数に(r6=r2 & ~0x1000) */
    shr     12, r2         /* 例外の発生元がユーザモードならr2=1 */
    xori    0x0001, r2, r2     /* r2 = r2==1? 0: 1 */
    Lea     _pre_protection_supervised, r12
    st.b    r2, 0[r12]
    /* =end modified for SC3 */
	jarl    _call_protectionhk_main, r31

#ifdef CFG_USE_PROTECTIONHOOK
	mov     sp, ep

	/* v850_cpu_exp_noを復帰 */
	Lea     _v850_cpu_exp_no, r12
	sld.w   8[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_pcを復帰 */
	Lea     _v850_cpu_exp_pc, r12
	sld.w   4[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_sp退避 */
	Lea     _v850_cpu_exp_sp, r12
	sld.w   0[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_*を退避したスペースを戻す */
	addi    12, sp, sp
#endif /* CFG_USE_PROTECTIONHOOK */

FLABEL(fe_ret_exc)
	LOCK_ALL_INT

	/*
	 * プロテクションフックはOS割込み禁止状態で実行する
	 */

	/*
	 *  割込み・例外のネスト回数のデクリメント
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
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     ret_exc_2

	/*
	 *  ネスト回数が1以上なら割込み元がISRなため例外発生元へ復帰
	 *  スタックポインタの入れ替えしない
	 */
	cmp     r0, r13
	bne     ret_exc_1

	/*
	 *  戻り先がタスクなためスタックを戻す
	 */
	ld.w    0[sp], sp

FLABEL(ret_exc_1)
	/*
	 *  ISR1実行状態以外で発生した場合，OS割込み禁止状態の解除
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     -1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     r0, r13
	bne     ret_exc_2

	/* OS割込みを解除 */
	/* pmr_setting_tbl[current_iintpri] -> PMR */
	Lea     _current_iintpri, r12
	ld.bu   0[r12], r12
	shl     1, r12

	Lea     _pmr_setting_tbl, r13
	add     r12, r13
	ld.h    0[r13], r13
	SET_PMR r13, r12     /* set interrupt level */

	/*
	 *  例外発生元へ復帰
	 */
FLABEL(ret_exc_2)
	/*
	 * レジスタを復帰
	 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, fepc
	sld.w   0[ep], r10
	ldsr    r10, fepsw

    /* スクラッチレジスタの復帰 */
    INT_EXC_RESTOREREG fewr

	stsr    fepsw, r2   /* r2 = fepsw */
    shl     PSW_SV, r2         /* r2 << PSW_SV により例外発生時のモード取得 */
    bnc     return_from_fe_exception    /* 特権からの例外ならばスキップ */
FLABEL(fe_exception_to_utask)
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    ld.w    TCB_usp[r2], sp         /* uspを復帰 */
FLABEL(return_from_fe_exception)
    stsr    fewr, r2
	feret

	/*
	 *  例外エントリ（eiretでリターンする例外用）
     *  ここで，CY = EIPSW.IMP，r2 = error code
     *  r2はOS側で予約する（割込み・例外の出入口の
     *  ワークレジスタとして使用する）ため，
     *  スタックに保存しない
	 */
	.global _ei_exception_entry
FLABEL(_ei_exception_entry)
	/* 割込み/例外の先頭で実行する処理 */
	PRE_INT_EXC

    stsr    eipsw, r2
    shl     PSW_SV, r2              /* r2 << PSW_SV により例外発生時のモード取得 */
    mov     r0, r2                  /* mov命令ではステータスレジスタは変化しない */
    bnc     ei_exception_0          /* 特権からの例外ならばスキップ */
    /*
     *  ユーザタスクからの例外
     */
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    st.w    sp, TCB_usp[r2]         /* uspを保存 */
    ld.w    TCB_p_tinib[r2], r2
    ld.w    TINIB_sstk_bottom[r2], sp         /* sspを復帰 */
    ori     0x1000, r0, r2          /* r2 = 0x1000 */
FLABEL(ei_exception_0)
    /*
     *  EI例外による要因はE_OS_PROTECTION_EXCEPTIONのみである
     */
    ori E_OS_PROTECTION_EXCEPTION, r2, r2 /* r2 |= E_OS_PROTECTION_EXCEPTION */

    /* スクラッチレジスタの保存 */
    INT_EXC_SAVEREG eiwr

	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

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
	 * カテゴリ1の割り込み以外で例外が発生したかの判断する
	 * カテゴリ1で例外が発生：ei_exception_1へジャンプ
	 */
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     ei_exception_1

	/*
	 *  ISR1実行状態以外で発生した場合
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     1, r13
	bne     ei_exception_1

	/*
	 * OS割込み解除状態で発生した場合
	 * OS割込み禁止状態とする
	 */
	Lea     _pmr_isr2_mask, r7
	ld.h    0[r7], r7
	SET_PMR r7, r6                   /* pmr_isr2_mask -> PMR */

FLABEL(ei_exception_1)
	/*
	 *  割込み発生時に変更されるので割込み解除前に取得
	 */
	stsr    eiic, r19                   /* 例外要因 */
	stsr    eipc, r18                   /* 例外発生PC */


FLABEL(ei_exception_2)
	/*
	 *  カーネル起動していない場合に起きたCPU例外は，無限ループへ
	 */
	Lea   _kerflg, r12          /* kerflgがFALSEなら無限ループ */
	ld.b  0[r12], r12
	cmp   r0, r12
FLABEL(infinity_loop_ei)
	be    infinity_loop_ei

	/*
	 * 多重割込みか判定
	 */
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	mov     1, r14                      /*  割込み・例外のネスト回数のインクリメント */
	add     r14, r13
	st.w    r13, 0[r12]

	mov     sp, r9                      /* sp をr9に退避 */

	cmp     1, r13
	bne     ei_exception_3             /* _nested_lock_os_int_cnt > 1 -> ei_exception_3 */

	/*
	 * ISR1実行状態で発生した場合はスタックポインタの入れ替えしない
	 */
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     ei_exception_3

	/* スタックポインタの保存と入れ替え */
	Lea     __ostkpt, r12
	ld.w    0[r12], r12
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */

FLABEL(ei_exception_3)
    /* EIPSW.ID == 1の場合はここにこないため，条件判定は不要 */
	UNLOCK_ALL_INT

#ifdef CFG_USE_PROTECTIONHOOK
	/* v850_cpu_exp_*を退避するスペースを確保 */
	addi    -12, sp, sp

	/* v850_cpu_exp_spの退避と設定 */
	Lea     _v850_cpu_exp_sp, r12
	st.w    r9, 0[r12]
	st.w    ep, 0[sp]
	mov     sp, ep

	/* v850_cpu_exp_noの退避と設定 */
	Lea     _v850_cpu_exp_no, r12
	ld.w    0[r12], r13
	sst.w   r13, 8[ep]
	st.w    r19, 0[r12]                     /* r19 例外要因 */

	/* v850_cpu_exp_pcの退避と設定 */
	Lea     _v850_cpu_exp_pc, r12
	ld.w    0[r12], r13
	sst.w   r13, 4[ep]
	mov     r18, r17                        /* r18:fepc */
	add     -4, r17
	st.w    r17, 0[r12]

#endif /* CFG_USE_PROTECTIONHOOK */

	/*
	 * CPU例外が発生した場合，OSはE_OS_PROTECTION_EXCEPTIONをパラメータとして
	 * プロテクションフックを呼び出す
	 */
    /* =begin modified for SC3 */
    andi    0xefff, r2, r6 /* エラーコードを第1引数に(r6=r2 & ~0x1000) */
    shr     12, r2         /* 例外の発生元が特権モードならr2=1 */
    xori    0x0001, r2, r2     /* r2 = r2==1? 0: 1 */
    Lea     _pre_protection_supervised, r12
    st.b    r2, 0[r12]
    /* =end modified for SC3 */
	jarl    _call_protectionhk_main, r31

#ifdef CFG_USE_PROTECTIONHOOK
	mov     sp, ep

	/* v850_cpu_exp_noを復帰 */
	Lea     _v850_cpu_exp_no, r12
	sld.w   8[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_pcを復帰 */
	Lea     _v850_cpu_exp_pc, r12
	sld.w   4[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_sp退避 */
	Lea     _v850_cpu_exp_sp, r12
	sld.w   0[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_*を退避したスペースを戻す */
	addi    12, sp, sp
#endif /* CFG_USE_PROTECTIONHOOK */

FLABEL(ei_ret_exc)
	LOCK_ALL_INT

	/*
	 * プロテクションフックはOS割込み禁止状態で実行する
	 */

	/*
	 *  割込み・例外のネスト回数のデクリメント
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
	GET_ISPR r6
	Lea     _pmr_isr1_mask, r7
	ld.h    0[r7], r7
	and     r7, r6
	cmp     r0, r6
	bne     ei_ret_exc_2

	/*
	 *  ネスト回数が1以上なら割込み元がISRなため例外発生元へ復帰
	 *  スタックポインタの入れ替えしない
	 */
	cmp     r0, r13
	bne     ei_ret_exc_1

	/*
	 *  戻り先がタスクなためスタックを戻す
	 */
	ld.w    0[sp], sp

FLABEL(ei_ret_exc_1)
	/*
	 *  ISR1実行状態以外で発生した場合，OS割込み禁止状態の解除
	 */
	Lea     _nested_lock_os_int_cnt, r12
	ld.b    0[r12], r13
	mov     -1, r14
	add     r14, r13
	st.b    r13, 0[r12]
	cmp     r0, r13
	bne     ei_ret_exc_2

	/* OS割込みを解除 */
	/* pmr_setting_tbl[current_iintpri] -> PMR */
	Lea     _current_iintpri, r12
	ld.bu   0[r12], r12
	shl     1, r12

	Lea     _pmr_setting_tbl, r13
	add     r12, r13
	ld.h    0[r13], r13
	SET_PMR r13, r12             /* set interrupt level */

	/*
	 *  例外発生元へ復帰
	 */
FLABEL(ei_ret_exc_2)
	/*
	 * レジスタを復帰
	 */
	mov     sp, ep
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w   0[ep], r10
	ldsr    r10, eipsw

    /* スクラッチレジスタの復帰 */
    INT_EXC_RESTOREREG eiwr

	stsr    eipsw, r2   /* r2 = eipsw */
    shl     PSW_SV, r2  /* r2 << PSW_SV により例外発生時のモード取得 */
    bnc     return_from_ei_exception    /* 特権からの例外ならばスキップ */
FLABEL(ei_exception_to_utask)
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    ld.w    TCB_usp[r2], sp         /* uspを復帰 */
FLABEL(return_from_ei_exception)
    stsr    eiwr, r2
	eiret

	/*
	 *  カーネル起動していない場合に起きたCPU例外の無限ループ
	 */
FLABEL(infinity_loop)
	br infinity_loop

	/*
	 *  割り込みエントリ
	 *  V850E2
	 *   ベクターテーブルから Os_Lcfg.c に生成される各割込みの割込みエントリ
	 *   からジャンプし，r2に割込み番号が保存された状態で実行される
	 *  V850E3V5
	 *   ベクターテーブルから直接実行されるため，スタックの確保やレジスタの保存は
	 *   特に行われていない状態で実行される
	 */
	.global _interrupt
FLABEL(_interrupt)
	/* 割込み/例外の先頭で実行する処理 */
	PRE_INT_EXC

#ifdef __v850e3v5__
    ldsr    r2, eiwr
#endif /* __v850e3v5__ */
	/*
	 *  残りのレジスタ退避， EPでのアクセスに切り替えて行う
     * ここで，CY = EIPSW.IMP，r2[31:16] = intno
     *  r2はOS側で予約する（割込み・例外の出入口の
     *  ワークレジスタとして使用する）ため，
     *  スタックに保存しない
	 */
    stsr    eipsw, r2
    shl     PSW_SV, r2              /* r2 << PSW_SV により例外発生時のモード取得 */
    bnc     interrupt_0             /* 特権からの割込みならばスキップ */
    /*
     *  ユーザタスクへの割込み
     */
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    st.w    sp, TCB_usp[r2]         /* uspを保存 */
    ld.w    TCB_p_tinib[r2], r2
    ld.w    TINIB_sstk_bottom[r2], sp         /* sspを復帰 */
FLABEL(interrupt_0)
#ifdef __v850e3v5__
	/* 割込み要因の特定 */
	stsr	eiic,   r2
	/* EIC割込みは0x1000 から */
	addi   -0x1000, r2, r2
#endif /* __v850e3v5__ */

    /* スクラッチレジスタの保存 */
    INT_EXC_SAVEREG eiwr

	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]

	mov     r2, r12

	/* 割込み優先度を取得(r11) */
	GET_ISPR r13
	sch1r	r13, r11   
	add		-1,r11

	/* 
	 *  _current_iintpri の更新 
	 *  割り込み処理中はISPRが自動的に設定されているため，PMRは設定する必要がない．
	 */
	Lea     _current_iintpri, r6
	ld.bu   0[r6], r19
	st.b    r11,0[r6]

	/*
	 *  多重割込みか判定
	 */
	Lea     _except_nest_cnt, r13
	ld.w    0[r13], r6

	/*
	 *  割込み・例外のネスト回数のインクリメント
	 */
	add     1,  r6
	st.w    r6, 0[r13]
	cmp     1,  r6
	bne     interrupt_1     /* ネストしている場合はスタック切替無し */

	/*
	 *  タスクに割込んだ場合
	 *  スタックポインタの保存と入れ替え
	 */
	mov     sp , r13
	Lea     __ostkpt, sp
	ld.w    0[r3], r3
	add     -4, sp
	st.w    r13, 0[sp]

FLABEL(interrupt_1)
    /*
     *  割込み発生時の割込み優先度マスクをスタックに保存
     */
    add     -4, sp
    st.w    r19, 0[sp]

	UNLOCK_ALL_INT

#ifdef CFG_USE_STACKMONITORING
    /* 
     *  割込み番号の保存
     *  スタックオーバフローのプロテクションフックで破壊される可能性がある
     */
    push    r12 
	/*
	 *  タスクスタックのオーバフローチェック
	 */

	/*
	 * 多重割込みの場合はタスクスタックのスタックモニタリングを行わない
	 */
	cmp     1,  r6
	bne     int_nested

	/*
	 *  スタックポインタチェック方式
	 */
	Lea     _p_runtsk, r6                   /* 管理ブロックの先頭アドレス取得 */
	ld.w    0[r6], r6
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
    /* =begin modified for SC3 */
    /* システムスタックモニタリング */
	ld.w    TINIB_sstksz[r6], r7            /* タスクシステムスタックサイズ取得 */
	ld.w    TINIB_sstk_bottom[r6], r6       /* タスクシステムスタック終端アドレス取得 */
    sub     r7, r6
    mov     r0, r9
	cmp     r6, r13                         /* スタックポインタ <= スタックの先頭の場合NG  */
    Lea     stack_monitoring_exit, lp     
    /* =end modified for SC3 */
	bnh      stack_monitoring_error_isr

	/*
	 * マジックナンバーチェック方式
	 */
    ld.w    0[r6], r6
    mov     STACK_MAGIC_NUMBER, r7
	/*
	 *  マジックナンバー未破壊なら，割込み処理に飛ぶ
	 *  タスクに割込んだ場合，割込みスタックをチェックしない
	 */
	cmp     r7, r6
	be      stack_monitoring_exit
    /* =begin modified for SC3 */
    Lea     stack_monitoring_exit, lp     
    /* =end modified for SC3 */
	br      stack_monitoring_error_isr

	/* 
	 * 多重割込みの場合 
	 */
FLABEL(int_nested)
	/*
	 *  割込みスタックのオーバフローチェック
	 */

	/*
	 *  スタック残量チェック方式
	 */
	Lea     _isr_p_isrcb_tbl, r7
	mov     r12, r11
	shl     2, r11                          /* 割込み番号を4倍してオフセットを生成  */
	add     r11, r7
    ld.w    0[r7], r7                       /* r7 = &isrcb_table[x] */
    /* =begin modified for SC3 */
	ld.w    ISRCB_p_isrinib[r7], r7         
	ld.w    ISRINIB_p_intinib[r7], r7
    mov     1, r9
    /* =end modified for SC3 */
	ld.w    INTINIB_remain_stksz[r7], r7    /* 割込み番号に対応したスタックサイズの取得 */
	Lea     __ostk, r6                      /* 割込みスタックの先頭アドレス取得 */
	ld.w    0[r6], r6
	add     r6, r7                          /* 先頭アドレス＋ISRの使用するスタックサイズ */
	cmp     r7, sp                          /* スタックポインタ <= 先頭アドレス＋ISRの使用するスタックサイズの場合OK  */
    /* =begin modified for SC3 */
    Lea     stack_monitoring_exit, lp     
    /* =end modified for SC3 */
	bnh     stack_monitoring_error_isr

	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6                       /* このアドレスの示す先にマジックナンバーが入っている */
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
    /* =begin modified for SC3 */
    Lea     stack_monitoring_exit, lp     
    /* =end modified for SC3 */
	bne     stack_monitoring_error_isr

FLABEL(stack_monitoring_exit)
    pop     r12 
#endif /* CFG_USE_STACKMONITORING */

	/*
	 *  _callevel_stat 保存
	 */
	Lea     _callevel_stat, r6
	ld.h    0[r6], r7
	Push    r7

	/*
	 *  _callevel_stat 設定
	 */
	mov     TCL_ISR2, r8
	or      r8, r7
	st.h    r7, 0[r6]

    /* =begin added for SC3 */
	/*
	 *  run_trusted 保存
	 */
    Lea     _run_trusted, r6
    ld.b    0[r6], r7
    Push    r7

	/*
	 *  run_trusted 設定
	 */
	mov     TRUE, r8
    st.b    r8, 0[r6]

	/*
	 *  p_runosap 保存
	 */
    Lea     _p_runosap, r6
    ld.w    0[r6], r7
	Push    r7
    /* =end added for SC3 */

	/*
	 * p_runisr 保存
	 */
	Lea     _p_runisr, r6
	ld.w    0[r6], r7
	Push    r7

	/*
	 * p_runisr 設定
	 */
	Lea     _isr_p_isrcb_tbl, r7
	shl     2, r12                      /* 割込み番号を4倍してオフセットを生成  */
	add     r12, r7
	Lea     _p_runisr, r8
	ld.w    0[r7], r7
	st.w    r7, 0[r8]

	/*
	 *  割込みハンドラのアドレスを読み込む
	 */
	Lea     _isr_tbl, r6
	add     r12, r6

    /* =begin modified for SC3 */
    /*
     *  p_runosap 設定
     */
    cmp     r7, r0                      /* r7 = p_runisr == NULL ? */
    be      int_handler_call            /* p_runisr が NULL でなければスキップ */
    ld.w    ISRCB_p_isrinib[r7], ep     /* ep = p_runisr->p_isrinib */
    sld.w   ISRINIB_p_osapcb[ep], r8    /* r8 = p_runisr->p_isrinib->p_osapcb */
    Lea     _p_runosap, ep
    sst.w   r8, 0[ep]                   /* p_runosap = p_runisr->p_isrinib->p_osapcb */

FLABEL(int_handler_call)
    /* =end modified for SC3 */
	/*
	 *  割込みハンドラ呼び出し
	 */
	Lea     end_int_handler, r31
	ld.w    0[r6], r6
	jmp     [r6]

	/*
	 *  スタックオーバフロー時の処理
	 */
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  スタックオーバフロー時プロテクションフックを呼び出し
	 *  スタックを更に壊さないため，割込みスタックの初期値を使用する
     *  ISRの出入口から相対ジャンプできるようにここに置く
     *  stack_monitoring_errorへの踏み台
	 */
FLABEL(stack_monitoring_error_isr)
    br  stack_monitoring_error
#endif /* CFG_USE_STACKMONITORING */

FLABEL(end_int_handler)

#ifdef CFG_USE_STACKMONITORING
	/*
	 *  割込みスタックのオーバフローチェック
	 *  割込みから戻った時，スタックポインタも戻ったはずなので，
	 *  マジックナンバーチェック方式のみ実施
	 */

	/*
	 * マジックナンバーチェック方式
	 */
	Lea     __ostk, r6                       /* 割込みスタックの先頭アドレス取得 */
	ld.w    0[r6], r6
	ld.w    0[r6], r6                       /* このアドレスの示す先にマジックナンバーが入っている */
    /* =begin modified for SC3 */
    mov     1, r9
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
    Lea     stack_monitoring_exit2, lp     
	bne     stack_monitoring_error_isr

FLABEL(stack_monitoring_exit2)
    /* =end modified for SC3 */
#endif /* CFG_USE_STACKMONITORING */

	/*
	 * ISR2の不正終了チェック
	 */
	jarl    _exit_isr2, r31

	/*
	 * p_runisr の復帰
	 */
	Lea     _p_runisr, r7
	Pop     r6
	st.w    r6, 0[r7]

    /* =begin modified for SC3 */
	/*
	 *  p_runosap 復帰
	 */
    Lea     _p_runosap, r7
	Pop     r6
    st.w    r6, 0[r7]

	/*
	 *  run_trusted 復帰
	 */
    Lea     _run_trusted, r7
	Pop     r6
    st.b    r6, 0[r7]
    /* =end modified for SC3 */


	/*
	 * _callevel_stat の復帰
	 */
	Lea     _callevel_stat, r7
	Pop     r6
	st.h    r6, 0[r7]

/*
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

	LOCK_ALL_INT

    /*
     *  割込み発生時の割込み優先度マスクをスタックから復帰
     */
    ld.w    0[sp], r19
    add     4, sp
	Lea     _current_iintpri, r6
	st.b    r19, 0[r6]              /* current_iintpriを復帰 */

	/*
	 *  割込み・例外のネスト回数のデクリメント
	 */
	Lea     _except_nest_cnt, r6
	ld.w    0[r6], r7
	add     -1, r7
	st.w    r7, 0[r6]

	/*
	 *  ネスト回数が0なら，割込み元がタスク
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
	ld.w    0[sp], sp                       /* スタックポインタを戻す     */

	/*  
     *  割込み優先度マスクを変えるAPIがないため，
     *  ATK2では割込み優先度マスクの復帰処理は不要
     */
	/*
	 *  割込みハンドラ実行時には，ISPRで優先度が自動的に上がるため，
	 *  PMRは変更していないが，割り込みハンドラ内で x_nested_unlock_os_int() を
	 *  呼び出すとPMRが書き換わる．ディスパッチが必要でない場合は，ret_int_1で
	 *  PMRを元の値に戻す．ディスパッチが必要な場合は，OS割り込み禁止とするため，
	 *  ここで，PMRを更新する必要はない．
	 */

	/*
	 *  戻り先がタスクの場合，ディスパッチの必要があるかチェック
	 */
	Lea     _p_runtsk, r6
	ld.w    0[r6], r6
	Lea     _p_schedtsk, r7
	ld.w    0[r7], r7
	cmp     r6, r7
	be      ret_int_1      /* ディスパッチの必要がなければ割込み元へ戻る */

	/*
	 *  ここでは，戻り先がタスクであり，スタックは，タスクスタックに
	 *  スクラッチレジスタのみが保存された状態になっている．また，
	 *  全割込みを禁止した状態となっている
	 */

	/* 
	 *  優先度マスクのクリア 
	 *  eiretを実行しないため，eiretによる割込みコントローラの優先度マスク
	 *  (インサービスプライオリティ)のクリアが行われないため，代わりにISPC
	 *  を用いてクリアする．
	 */
	CLEAR_ISPR r10, r12

	/*
	 *  OS割込み禁止状態に移行する
	 *
	 *  この時点でOS割込み禁止状態とするのは，dispatcherを実行する際に
	 *  OS割込み禁止状態になっている必要があるためである
	 *
	 *  (モデル上の)割込み優先度マスクは0であるため，ISR1以外の割込みは全て禁止する
	 */

	OUTER_LOCK_OS_INT r12, r13
	/* ret_int で全割込み禁止状態としたため解除 */
	UNLOCK_ALL_INT

	/*
	 * コンテキストの退避 
	 */
#ifndef TOPPERS_USE_PUSHSP
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
#else /* TOPPERS_USE_PUSHSP */
	pushsp  r20-r29
#endif /* TOPPERS_USE_PUSHSP */

	/* スタックと次の起動番地を保存 */
	Lea     _p_runtsk, ep
	sld.w   0[ep], ep
    /* =begin modified for SC3 */
	sst.w   sp, TCB_ssp[ep]
    /* =end modified for SC3 */
	Lea     ret_int_r, r6
	sst.w   r6, TCB_pc[ep]
	br      dispatcher

	/*
	 *  割込み・例外でコンテキスト保存した場合の復帰ルーチン
	 */
FLABEL(ret_int_r)
	/*
	 * コンテキストの復帰
	 */
#ifndef TOPPERS_USE_PUSHSP
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
#else /* TOPPERS_USE_PUSHSP */
	popsp   r20-r29
#endif /* TOPPERS_USE_PUSHSP */

	LOCK_ALL_INT

	OUTER_UNLOCK_OS_INT r12, r13
	/*
	 *  割込みからの復帰ルーチン
	 */
FLABEL(ret_int_1)
	/*
	 * レジスタを復帰
	 */
	mov     sp, ep

	/*  
     *  割込み優先度マスクを変えるAPIがないため，
     *  ATK2では割込み優先度マスクの復帰処理は不要
     */
	/*
	 *  PMRを元に戻す．
	 *  割込みハンドラ実行時には，ISPRで優先度が自動的に上がるため，
	 *  PMRは変更していないが，割り込みハンドラ内で x_nested_unlock_os_int() を
	 *  呼び出すとPMRが書き換わるためここで元の値に戻す．
	 *  OUTER_UNLOCK_OS_INT は nested_lock_os_int_cnt を0にするが，
	 *  ここでは既に0であるため問題ない．
	 */
	OUTER_UNLOCK_OS_INT  r1, r6
	sld.w    4[ep], r10
	ldsr    r10, eipc
	sld.w   0[ep], r10
	ldsr    r10, eipsw

    /* スクラッチレジスタを復帰 */
    INT_EXC_RESTOREREG eiwr

	stsr    eipsw, r2           /* r2 = eipsw */
    shl     PSW_SV, r2          /* r2 << PSW_SV により例外発生時のモード取得 */
    bnc     return_from_int     /* 特権からの割込みならばスキップ */
FLABEL(int_to_utask)
    Lea     _p_runtsk, r2
    ld.w    0[r2], r2
    ld.w    TCB_usp[r2], sp     /* uspを復帰 */
FLABEL(return_from_int)
    stsr    eiwr, r2
	eiret

/*
 * タスクディスパッチ
 */
	.global _dispatch
FLABEL(_dispatch)
	/*
	 *  このルーチンは，タスクコンテキスト・OS割込み禁止状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる
	 */
#ifndef TOPPERS_USE_PUSHSP
	addi    -48, sp, sp
	st.w    ep,  0[sp]
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
	sst.w   r31, 4[ep]
#else /* TOPPERS_USE_PUSHSP */
	pushsp  r20-r31
#endif /* TOPPERS_USE_PUSHSP */
	/* スタックと次の起動番地を保存 */
	Lea     _p_runtsk, ep
	sld.w   0[ep], ep
    /* =begin modified for SC3 */
	sst.w   sp, TCB_ssp[ep]
    /* =end modified for SC3 */
	Lea     dispatch_r, r10
	sst.w   r10, TCB_pc[ep]
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  実行中タスクのタスクスタックのオーバフローチェック
	 */
	/*
	 *  スタックポインタチェック方式
	 */
	Lea     _p_runtsk, r6                   /* 管理ブロックの先頭アドレス取得 */
	ld.w    0[r6], r6
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
    /* =begin modified for SC3 */
    /* システムスタックモニタリング */
	ld.w    TINIB_sstksz[r6], r7            /* タスクシステムスタックサイズ取得 */
	ld.w    TINIB_sstk_bottom[r6], r6       /* タスクシステムスタック終端アドレス取得 */
    sub     r7, r6
    mov     1, r9
	cmp     r6, sp                          /* スタックポインタ <= スタックの先頭の場合NG  */
    Lea     dispatcher, lp
    /* =end modified for SC3 */
	bnh      stack_monitoring_error
	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
    /* =begin modified for SC3 */
    Lea     dispatcher, lp
    /* =end modified for SC3 */
	bne     stack_monitoring_error
#endif /* CFG_USE_STACKMONITORING */
	br      dispatcher

	/* 
	 * ディスパッチャの復帰ルーチン 
	 */
FLABEL(dispatch_r)
#ifndef TOPPERS_USE_PUSHSP
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
	sld.w    4[ep], r31
	addi    48, ep, sp
	ld.w    0[sp], ep
#else /* TOPPERS_USE_PUSHSP */
	popsp   r20-r31
#endif /* TOPPERS_USE_PUSHSP */
	jmp     [lp]

	/*
	 *  スタックオーバフロー時の処理
	 */
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  スタックオーバフロー時プロテクションフックを呼び出し
	 *  スタックを更に壊さないため，割込みスタックの初期値を使用する
     *  r9 = ostkptの初期値に切り替えるか?
	 */
FLABEL(stack_monitoring_error)
	/*
	 *  r9==0の場合でも，既に_ostkptを使っているので，
	 *  except_nest_cntをインクリメントしても問題はない
	 */
	Lea     _except_nest_cnt, r12               /* _ostkptを使う場合はexcept_nest_cntをインクリメントする */
	ld.w    0[r12], r13
	add     1, r13
	st.w    r13, 0[r12]

    /* =begin modified for SC3 */
    cmp     r0, r9
    be      stack_monitoring_error_1
    /* =end modified for SC3 */

	Lea    __ostkpt, r12                        /* スタックを更に壊さないため，割込みスタックの初期値を使用する */
	ld.w    0[r12], r12
	mov     r12, sp

    /* =begin modified for SC3 */
FLABEL(stack_monitoring_error_1)
	/*
	 *  プロテクションフックはOS割込み禁止状態で呼び出す
	 */
	NESTED_LOCK_OS_INT r12, r13
    /* =end modified for SC3 */

	mov     E_OS_STACKFAULT, r6                 /* プロテクションフックの引数を設定 */
    /* =begin modified for SC3 */
 	Lea     _pre_protection_supervised, r12
    mov     0x01, r13
 	st.b    r13, 0[r12]     /* pre_protection_supervised = 0x01 */
	jr      _call_protectionhk_main        /* プロテクションフックを呼び出し */
    /* ここには来ない */
FLABEL(stack_monitoring_error_loop)
    jr   stack_monitoring_error_loop
    /* =end modified for SC3 */
#endif /* CFG_USE_STACKMONITORING */


/*
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

    /* =begin modified for SC3 */
	.global _exit_and_dispatch_nohook
	/*
	 *  タスク強制終了の場合はPostTaskHookを呼出さない
	 *  その場合のエントリである
	 */
_exit_and_dispatch_nohook:

	br    dispatcher_0
    /* =end modified for SC3 */

	/*
	 *  現在のコンテキストを捨ててディスパッチ
	 */
	.global _exit_and_dispatch
FLABEL(_exit_and_dispatch)
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  実行中のタスクスタックのオーバフローチェック
	 */
	/*
	 *  スタックポインタチェック方式
	 */
	Lea     _p_runtsk, r6                   /* 管理ブロックの先頭アドレス取得 */
	ld.w    0[r6], r6
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
    /* =begin modified for SC3 */
    /* システムスタックモニタリング */
	ld.w    TINIB_sstksz[r6], r7            /* タスクシステムスタックサイズ取得 */
	ld.w    TINIB_sstk_bottom[r6], r6       /* タスクシステムスタック終端アドレス取得 */
    sub     r7, r6
    mov     1, r9
	cmp     r6, sp                          /* スタックポインタ <= スタックの先頭の場合NG  */
    Lea     dispatcher, lp
    /* =end modified for SC3 */
	bnh      stack_monitoring_error
	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
    /* =begin modified for SC3 */
    Lea     dispatcher, lp
    /* =end modified for SC3 */
	bne     stack_monitoring_error
#endif /* CFG_USE_STACKMONITORING */
	/* ディスパッチャ本体（dispatcher）へ */

FLABEL(dispatcher)
	/*
	 *  PostTaskHookの呼び出し
	 */
#ifdef CFG_USE_POSTTASKHOOK
FLABEL(posttaskhook_call)
	Lea     _call_posttaskhook, r6
	jarl    _stack_change_and_call_func_1, r31
#endif /* CFG_USE_POSTTASKHOOK */

FLABEL(dispatcher_0)
	/*
	 *  このルーチンは，タスクコンテキスト・OS割込み禁止状態・ディスパッチ
	 *  許可状態・（モデル上の）割込み優先度マスク全解除状態で呼び出さ
	 *  れる．実行再開番地へもこの状態のまま分岐する
	 */
#ifdef LOG_DSP_ENTER
	Lea     _p_runtsk, r21
	ld.w    0[r21], r21
	cmp     r0, r21
	be      dispatcher_0_1
	mov     r21, r6                            /* 引数に_p_runtskを指定 */
	jarl    _log_dsp_enter, r31
FLABEL(dispatcher_0_1)
#endif	/* LOG_DSP_ENTER */

	Lea     _callevel_stat, r21
	ld.h    0[r21], r22
	mov     TSYS_DISALLINT, r20
	and     r22, r20
	be      dispatcher_1

	mov     ~TSYS_DISALLINT, r20
	and     r22, r20
	st.h    r20, 0[r21]
	UNLOCK_ALL_INT                             /* 全割込み禁止解除状態へ */

	/*
	 *  例外（割込み/CPU例外）のネスト回数はstart_dispatchや，dispatcher_3:
	 *  から来た場合は'1'となっているため，タスクスタックに切り換え後，
	 *  '0'に初期化する．
	 */
FLABEL(dispatcher_1)
	Lea     _p_runtsk, r21
	Lea     _p_schedtsk, r20
	ld.w    0[r20], r22
	st.w    r22, 0[r21]
	cmp     r22, r0
	be      dispatcher_3

	mov     r22, ep 
    /* =begin modified for SC3 */
	sld.w   TCB_ssp[ep], sp
    /* =end modified for SC3 */
	Lea     _except_nest_cnt, r20
	st.w    r0, 0[r20]

    /* =begin modified for SC3 */
    sld.w   TCB_p_tinib[ep], ep
    mov     ep, r25                     /* r25 = p_tinib */
    sld.w   TINIB_p_osapcb[ep], r26     /* r26 = p_osapcb */
    Lea     _p_runosap, ep              /* ep = &p_runosap */
    sst.w   r26, 0[ep]                  /* p_runosap = p_osapcb */
    ld.w    OSAPCB_p_osapinib[r26], r29 /* r29 = p_osapinib */
    ld.bu   OSAPINIB_osap_trusted[r29], r24 /* r24 = trusted? */
    Lea     _run_trusted, ep
    sst.b   r24, 0[ep]                      /* run_trusted = trusted? */
    cmp     r0, r24                         /* 信頼ならばスキップ */
    bne     dispatcher_5

#ifdef __v850e2v3__
    ori     0x1001, r0, r27
    ldsr    r27, bsel
    /*
     *  ユーザスタック領域
     */
    mov     r25, ep
    sld.w   TINIB_start_ustk[ep], r27   /* 先頭番地 */
    ori     0x01, r27, r27
    sld.w   TINIB_limit_ustk[ep], r28   /* 保護属性とサイズ */
    ori     0x06, r28, r28
    ldsr    r27, dpa0l
    ldsr    r28, dpa0u
#else /* defined(__v850e3v5__) */
    /*
     *  ユーザスタック領域
     */
    mov     r25, ep
    sld.w   TINIB_start_ustk[ep], r27   /* 先頭番地 */
    sld.w   TINIB_limit_ustk[ep], r28   /* 保護属性とサイズ */
    ldsr    r27, 0, 6                   /* MPLA0 */
    ldsr    r28, 1, 6                   /* MPUA0 */

#endif /* __v850e2v3__ */

   	/*
	 *  実行ドメインと保護ドメイン(MPUの設定)が同じならば，
     *  ドメイン毎のコード領域・データ領域をMPUに設定し直す必要はない
     */
    Lea     _p_ctxosap, ep      /* ep = &p_ctxosap */	
    sld.w   0[ep], r27          /* r27 = p_ctxosap */
	cmp     r27, r26            /* p_ctxosap == p_runosap ? */
	be      dispatcher_5        /* 同じならdispatcher_5へ */
    nop
	sst.w   r26, 0[ep]          /* p_ctxosap = p_runosap : 保護ドメインを切替え */
#ifdef __v850e2v3__
    ldsr    r26, vmtid          /* メモリ保護違反タスク識別子をp_runosapとする */
    mov     r29, ep             /* ep = p_osapinib */

    /*
     *  自保護ドメイン専用のrom領域
     */
    sld.w   OSAPINIB_start_text[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_text[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, ipa0l
    ldsr    r28, ipa0u
    /*
     *  自保護ドメイン専用のrosdata領域
     */
    sld.w   OSAPINIB_start_rosdata[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_rosdata[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, ipa1l
    ldsr    r28, ipa1u
    /* 
     * 自保護ドメイン専用のRWX領域
     */
    sld.w   OSAPINIB_start_ram[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_ram[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, dpa1l
    ldsr    r28, dpa1u
    /* 
     * 自保護ドメイン専用のRWX領域(sdata)
     */
    sld.w   OSAPINIB_start_sram[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_sram[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, dpa2l
    ldsr    r28, dpa2u
    /*
     * 共有リード/専用ライト領域の専用領域
     */
    sld.w   OSAPINIB_start_srpw[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_srpw[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, dpa3l
    ldsr    r28, dpa3u
     /*
     * 共有リード/専用ライト領域の専用領域(sdata)
     */
    sld.w   OSAPINIB_start_ssrpw[ep], r27  /* 先頭番地 */
    sld.w   OSAPINIB_limit_ssrpw[ep], r28  /* 保護属性とサイズ */
    ldsr    r27, dpa4l
    ldsr    r28, dpa4u

#else /* defined(__v850e3v5__) */
    /*
     * reg1: p_osapinib
     * reg2: tmp1
     * reg3: tmp2
     * reg4: tmp3
     * reg5: tmp4
     * reg6: tmp5
     */
    DISPATCHER_MPU_SETTING r29, r24, r25, r26, r27, r28
#endif /* __v850e2v3__ */

FLABEL(dispatcher_5)
#ifdef __v850e2v3__
    ldsr    r0, bsel    /* bselを戻す */
#endif /* __v850e2v3__ */
    /* =end modified for SC3 */

#ifdef LOG_DSP_LEAVE
	ld.w    0[r21], r6                          /* 引数に_p_runtskを指定 */
	jarl    _log_dsp_leave, r31
#endif	/* LOG_DSP_LEAVE */

#ifdef CFG_USE_PRETASKHOOK
FLABEL(pretaskhook_call)
	Lea     _call_pretaskhook, r6
	jarl    _stack_change_and_call_func_1, r31
#endif /* CFG_USE_PRETASKHOOK */

FLABEL(dispatcher_2)
	Lea     _p_runtsk, r20
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
	Lea     __ostkpt, r20		/* 非タスクコンテキスト用のスタックの読み込みスタックポインタの入れ替え */
	ld.w    0[r20], r3

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

#if 0
	nop
	nop
	nop
	nop
#else
	halt
#endif
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
 *  タスク開始時処理
 *
 *  dispatcherから呼び出される
 */
    /* =begin modified for SC3 */
	.global _start_stask_r
FLABEL(_start_stask_r)
    Lea		_p_runtsk, ep                   /* ep = p_runtsk */
	sld.w	0[ep], ep
	/*
	 *  タスクが不正終了した場合は保護処理を行うよう
	 *  戻りアドレスにexit_taskを設定
	 */
	Lea     __kernel_exit_task, r31
    mov     r0, r6
    br      _start_r

	.global _start_utask_r
FLABEL(_start_utask_r)
#ifndef __v850e3v5__
    ldsr    r0, bsel
#endif
    Lea		_p_runtsk, ep                   /* ep = p_runtsk */
	sld.w	0[ep], ep
    clr1    0, TCB_priv_mode[ep]            /* 特権モード実行を抜ける */
    sld.w   TCB_p_tinib[ep], r20            /* ep = p_runtsk->p_tinib */
    ld.w    TINIB_ustk_bottom[r20], r20     /* r20 = ユーザスタックポインタの初期値 */
    mov     r20, sp
	/*
	 *  タスクが不正終了した場合は保護処理を行うよう
	 *  戻りアドレスにexit_utaskを設定
	 */
    Lea     _exit_utask, lp                 /* lp = _exit_utask */
    stsr    psw, r10

    /* 
     *  全割込み禁止状態へ
     *  タスクコンテキストへのジャンプで解除される
     */
    di

    /* 
	 * 非特権モードで動作するよう準備
     * 保護機能有効（非信頼へ），割込み許可
     * PP,NPV,DMP,IMP = 1, ID = 0
     */
#ifdef __v850e2v3__
    movhi   0x000f, r0, r7                  
#else /* defined(__v850e3v5__) */
    movhi   0x4000, r0, r7  
#endif /* __v850e2v3__ */
    or      r7, r10         /* psw |= (PP|NPV|DMP|IMP) */
	ldsr    r10, eipsw      /* eipsw = psw */
    mov     1, r6

	.global _start_r
FLABEL(_start_r)
	ld.w    TCB_p_tinib[ep], r22
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  SC3ではタスク強制終了があるので，タスク起動毎にマジックナンバー
	 *  を初期化する
	 */
	ld.w    TINIB_sstk_bottom[r22], r21
    ld.w    TINIB_sstksz[r22], r10
    sub     r10, r21
	mov     STACK_MAGIC_NUMBER, r10
	st.w	r10, 0[r21]
#endif /* CFG_USE_STACKMONITORING */

	/*
	 *  OS割込み禁止解除前に現在優先度(TCB.curpri)を実行優先度(TINIB.exepri)に
	 *  変更(内部リソースを獲得)
	 */
	ld.b    TINIB_exepri[r22], r21			/* TINIB.exepri -> TCB.curpri */
	st.b    r21, TCB_curpri[ep]

	/*
	 *  OS割込み禁止解除状態へ
	 *  (モデル上の)割込み優先度マスクは0であるため，有効な割込みは全て許可する
	 */

	OUTER_UNLOCK_OS_INT r12, r13

	ld.w    TCB_p_tinib[ep], r22
	ld.w    TINIB_task[r22], r10
    cmp     r0, r6
    bne     _start_r_1              /* r6 != 0 ならば _start_r_1 へ*/
	jmp     [r10]

FLABEL(_start_r_1)
	ldsr    r10, eipc               /* EIPC = pc */
    eiret
    /* =end modified for SC3 */

    /*
     *  ext_tskをsvc命令で呼び出す
     */
    .section .text_shared
    .align 4
    .globl _exit_utask
FLABEL(_exit_utask)
    mov     TFN_EXIT_TASK, r11
    syscall NO_SVC
    nop

    .section .text_kernel
	.global _stack_change_and_call_func_1 
	.global _stack_change_and_call_func_2
FLABEL(_stack_change_and_call_func_1)
FLABEL(_stack_change_and_call_func_2)

	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	add     1, r13
	st.w    r13, 0[r12]
	cmp     1, r13
	bne     _stack_change_and_call_func_1_1

	/* スタックポインタの保存と入れ替え */
	Lea     __ostkpt, r12
	ld.w    0[r12], r12
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */
    

FLABEL(_stack_change_and_call_func_1_1)
	push    lp
	Lea     stack_change_and_call_func_r, lp  /* 戻りアドレス */
	mov     r6, r12
	mov     r7, r6
	mov     r8, r7
	jmp     [r12]

FLABEL(stack_change_and_call_func_r)
	pop     lp
	Lea     _except_nest_cnt, r12
	ld.w    0[r12], r13
	add     -1, r13
	st.w    r13, 0[r12]
	cmp     r0, r13
	bne     stack_change_and_call_func_r_1

	ld.w    0[sp], sp                    /* スタックポインタの復帰     */
FLABEL(stack_change_and_call_func_r_1)
	jmp     [lp]


#ifdef ENABLE_RETURN_MAIN
/*
 *  AKTSP用のmain()へのリターンコード
 */
	.global _return_main
FLABEL(_return_main)
	Lea   __ostkpt, sp
	ld.w  0[sp],sp
	jarl  _main, r31
#endif /* ENABLE_RETURN_MAIN */

    /* =begin modified for SC3 */
/*
 *  SYSCALLテーブル
 *  syscall命令は、scbp + scbp[x] の番地にジャンプする
 */
    .globl _syscall_table
    .section .text_kernel
    .align 4
FLABEL(_syscall_table)
    .word (_syscall_error_handler - _syscall_table)
    .word (_svc_entry - _syscall_table)

    .globl _syscall_error_handler
    .globl _svc_entry
/*
 *  SYSCALLエラーハンドラ
 */
FLABEL(_syscall_error_handler)
    br  _syscall_error_handler
    nop

/*
 *  サービスコールの出入口
 *  PP,NPV,DMP,IMP = 0, ID,EP = 1
 *  eipc = pc, eipsw = psw : eiする前に保存
 *    r10 : 返り値
 *    r11：関数コード
 *    r6 - r9 : 引数（4つまで）
 *    r20 : 第5引数
 *    スクラッチレジスタ保存済み
 *    lp保存済み
 */
FLABEL(_svc_entry)
    /*
     *  呼出し元が特権モードか？
     *  eipswをチェックする
     */
#ifndef __v850e3v5__
    ldsr    r0, bsel
#endif
    stsr    eipsw, r13
    shl     PSW_SV, r13         /* r13 << PSW_SV により例外発生時のモード取得 */
    bnc     svc_entry_2         /* 特権モードからの呼出しならばスキップ */

    /*
     *  タスクコンテキストからの呼出しの場合
     */
    /*
     *  ユーザスタックとシステムスタックを入れ替える
     *  タスクの状態を特権モード実行中に移行する
     *  戻り先が非特権モードで非信頼フック実行中でなければ，
     *  タスク実行中であるはずなので，p_runtskがNULLかどうかの
     *  チェックは必要ないはず
     */
    Lea     _p_runtsk, ep
    sld.w   0[ep], ep
    set1    0, TCB_priv_mode[ep]    /* p_runtsk->priv_mode |= 0x01 */
    sst.w   sp, TCB_usp[ep]         /* uspを保存 */
    sld.w   TCB_p_tinib[ep], ep
    sld.w   TINIB_sstk_bottom[ep], sp         /* sspを復帰 */

FLABEL(svc_entry_2)
    /*
     *  eipcとeipsw（syscall呼出し元のpcとpsw）を保存して
     *  割込みからリターン
     *  +--------------+ <- new sp
     *  | eipsw        |
     *  +--------------+
     *  | eipc         |
     *  +--------------+ <- old sp
     */
    addi    -8, sp, sp
    mov     sp, ep
    stsr    eipc, r12
    sst.w   r12, 4[ep]
    stsr    eipsw, r12
    sst.w   r12, 0[ep]
    /*
     *  eipswの保護関連ビットを0にする
     */
#ifdef __v850e2v3__
    movhi   0x000f, r0, r13
#else /* defined(__v850e3v5__) */
    movhi   0x4000, r0, r13
#endif /* __v850e2v3__ */
    not     r13, r13
    and     r13, r12
    ldsr    r12, eipsw          /* PP,NPV,DMP,IMP = 0 */
    Lea     svc_entry_3, r12
    ldsr    r12, eipc
    eiret

FLABEL(svc_entry_3)
    /*
     *  機能コードのチェック
     */
    mov     TMAX_SVCID, r12
    cmp     r12, r11            /* 機能コードの上限チェック(TMAX_SVCID < r11) */
    bgt     svc_entry_error     /* 上限をオーバしているならばエラー */

    /*
     *  サービスコールのテーブルを引き，
     *  サービスコールの本体を呼ぶ
     */
    Lea     _svc_table, r12
    shl     2, r11                  /* r11 *= 4 */
    add     r11, r12                /* r12 = &svc_table[r11] */
	ld.w    0[r12], r12             /* テーブルを引く */
    Lea     svc_entry_r, lp         /* 戻り番地を設定 */
	jmp     [r12]                   /* サービスコール処理ルーチンを呼ぶ */
    nop

FLABEL(svc_entry_error)
    jarl    _no_support_service, lp /* no_support_service を呼ぶ */

FLABEL(svc_entry_r)
    /*
     *  この時点でのpswをr11に保存する
     *  全割込み禁止で更新した時への対応
     */
    stsr    psw, r11

    /*
     *  全割込み禁止かつEP状態へ
     *  ここでEP状態とするのは，ISRからシステムサービスが呼ばれた
     *  場合に，eiretによってISPRが更新されるのを防ぐためである
     */
    ori     0x60, r11, r12   /* psw |= EP|ID */
    ldsr    r12, psw

    /*
     *  eipcとeipsw（syscall呼出し元のpcとpsw）を復帰
     *  ここでは，割込み処理中でない（サービスコール呼出し前に
     *  eiretを発行している）が，eiretを利用してリターンする
     *  +--------------+ <- old sp
     *  | eipc         |
     *  +--------------+
     *  | eipsw        |
     *  +--------------+ <- new sp
     */
    mov     sp, ep
    sld.w   4[ep], r15
    sld.w   0[ep], r16
    addi    8, sp, sp

    /*
	 *  スタックを戻すかチェック
	 */
    /*
     *  呼出し元が特権モードか？
     *  eipswをチェックする
     */
#ifdef __v850e2v3__
    ldsr    r0, bsel
    movhi   0x000f, r0, r12
#else /* defined(__v850e3v5__) */
    movhi   0x4000, r0, r12
#endif /* __v850e2v3__ */
    tst     r16, r12
    be      svc_prv_r           /* 特権モードからの呼出しならばスキップ */

    Lea     _p_runtsk, ep
    sld.w   0[ep], ep
    tst1    0, TCB_priv_mode[ep]    /* uspを保存したか？ */
    be      svc_prv_r               /* uspを保存していないならばスキップ */

FLABEL(svc_usr_r)
    /* 
     *  特権モード実行を抜ける
     *  sstkとustkを入れ替える
     *  戻り先がタスクコンテキストならば，タスク実行中であるはず
     *  なので，p_runtskがNULLかどうかのチェックは必要ないはず
     */
    clr1    0, TCB_priv_mode[ep]    /* p_runtsk->priv_mode &= ~0x01 */
    sld.w   TCB_usp[ep], sp         /* uspを復帰 */
    
FLABEL(svc_prv_r)
    /*
     *  システムサービスでセットされた割込み優先度マスクを反映する
     */
    mov     0x20, r12
    and     r12, r11    /* PSW.ID以外を0に */
    not     r12, r12    /* r12 = ~0x20 */
    and     r12, r16    /* eipsw &= ~0x20 */
    or      r11, r16    /* eipsw |= r11 */
    ldsr    r16, eipsw
    ldsr    r15, eipc
    eiret               /* 呼出し元へリターン */
    nop

	/*
	 *  スタックチェック関数
	 *   入力 r6  使用するスタック量
	 *   使用するスタック量が残っていない場合はエラー
	 */
    .global _trustedfunc_stack_check
FLABEL(_trustedfunc_stack_check)
    Lea     _callevel_stat, ep
    sld.h   0[ep], r10
    addi    TCL_ISR2, r0, r11     
    tst     r11, r10
    be      trustedfunc_stack_check_1       /* TCL_TASKの場合はスキップ */

    /*
     *  _ostkを使用中
     */
    Lea     __ostk, ep
    sld.w   0[ep], r11
    br      trustedfunc_stack_check_2

FLABEL(trustedfunc_stack_check_1)
    /*
     *  タスクのシステムスタックを使用中
     */
    Lea     _p_runtsk, ep
    sld.w   0[ep], ep
    sld.w   TCB_p_tinib[ep], ep
    sld.w   TINIB_sstk_bottom[ep], r11
    sld.w   TINIB_sstksz[ep], r12
    sub     r12, r11
FLABEL(trustedfunc_stack_check_2)
    /*
     *  スタックの残量をチェック
     *  r11 = スタックの先頭
     */
    subr    sp, r6                      /* r6 = ssp - tf_stksz */
    bl      trustedfunc_stack_check_3   /* ssp < tfstksz ならばエラー処理へ */
    cmp     r11, r6                     /* r11 < r6 ？ */
    bnh     trustedfunc_stack_check_3   /* そうでなければエラー処理へ */
    addi    E_OK, r0, r10
    jmp     [lp]

FLABEL(trustedfunc_stack_check_3)
    addi    E_OS_STACKINSUFFICIENT, r0, r10
    jmp     [lp]

    /* =end modified for SC3 */


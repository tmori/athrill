/* 
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
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
 *  $Id: prc_support.S 845 2017-09-25 04:33:56Z ertl-honda $
 */

/*
 *		ターゲット依存情報の定義
 */
#include <v850asm.inc>
#include "offset.h"

#ifdef USE_ASMCONFIG_INC
#include "asm_config.inc"
#endif /* USE_ASMCONFIG_INC */

/*
 *  V850E2用の割込みコントローラ操作ルーチン
 */
.macro GET_ISPR reg
	Lea		INTC_ISPR, AMARG(reg)
	ld.b	0[AMARG(reg)],AMARG(reg)
	andi    255, AMARG(reg), AMARG(reg)
.endm

/*
 *  自コアのCCBの取得
 */
.macro MYCCB reg12, reg13
    GET_CID AMARG(reg12)
    Lea     _p_ccb_table, AMARG(reg13)
    shl     2, AMARG(reg12)
    add     AMARG(reg12), AMARG(reg13)
    ld.w    0[AMARG(reg13)], AMARG(reg12)
.endm

/*
 *  OS割込み禁止マクロ
 *  ネストの一番外側で呼び出されることを想定している
 */
.macro OUTER_LOCK_OS_INT  reg12, reg13, reg_myccb
	jarl	_set_intpri_lock_os_int, lp

    /* nested_lock_os_int_cnt = 1 */
    mov     1, AMARG(reg13)
    st.b    AMARG(reg13), CCB_nested_lock_os_int_cnt[AMARG(reg_myccb)]
.endm

/*
 *  OS割込み禁止解除マクロ
 *  ネストの一番外側で呼び出されることを想定している
 */
.macro OUTER_UNLOCK_OS_INT  reg12, reg13, reg_myccb

    /* nested_lock_os_int_cnt = 0 */
    st.b    r0, CCB_nested_lock_os_int_cnt[AMARG(reg_myccb)]

	jarl	_set_intpri_unlock_os_int, lp

.endm

/*
 *  非タスクコンテキスト用のスタックの取得
 */
.macro MYOSTKPT reg12, reg13
     GET_CID AMARG(reg12)
     shl     2, AMARG(reg12)
     Lea     _ostkpt_table, AMARG(reg13)
     add     AMARG(reg12), AMARG(reg13)
     ld.w    0[AMARG(reg13)], AMARG(reg12)
.endm

	/* 
	 * 割込み/例外の先頭で実行する処理
	 */
.macro PRE_INT_EXC
	nop
.endm

	/*
	 * 割込み例外入り口でのレジスタの保存マクロ
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

.macro FE_SAVEREG
	INT_EXC_SAVEREG
	stsr    fepc, r6
	sst.w   r6, 4[ep]
	stsr    fepsw, r6
	sst.w   r6, 0[ep]
.endm

.macro FE_RESTOREREG
	sld.w    4[ep], r6
	ldsr    r6, fepc
	sld.w    0[ep], r6
	ldsr    r6, fepsw
	INT_EXC_RESTOREREG
.endm

.macro EI_SAVEREG
	INT_EXC_SAVEREG
	stsr    eipc, r6
	sst.w   r6, 4[ep]
	stsr    eipsw, r6
	sst.w   r6, 0[ep]
.endm

.macro EI_RESTOREREG
	sld.w    4[ep], r6
	ldsr    r6, eipc
	sld.w    0[ep], r6
	ldsr    r6, eipsw
	INT_EXC_RESTOREREG
.endm

	/*
	 *  例外エントリ（eiretでリターンする例外用）
	 */
	.global _ei_exception_entry
FLABEL(_ei_exception_entry)
	/* 割込み/例外の先頭で実行する処理 */
	PRE_INT_EXC

	/* 
	 * レジスタの保 (EIPC/EIPSWも保存)
	 */
	EI_SAVEREG

	MYCCB   r16, r12

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
	ld.b    CCB_nested_lock_os_int_cnt[r16], r13
	mov     1, r14
	add     r14, r13
	st.b    r13, CCB_nested_lock_os_int_cnt[r16]
	cmp     1, r13
	bne     ei_exception_1

	/*
	 * OS割込み禁止(OS割込み禁止ネストカウント=1)
	 * ※OS割込み解除状態で発生した場合
	 */
	jarl	_set_intpri_lock_os_int, lp

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
	ld.b  CCB_kerflg[r16], r12  /* kerflgがFALSEなら無限ループ */
	cmp   r0, r12
	be    infinity_loop

	/*
	 * 多重割込みか判定
	 */
	ld.w    CCB_except_nest_cnt[r16], r13
	mov     1, r14                      /*  割込み・例外のネスト回数のインクリメント */
	add     r14, r13
	st.w    r13, CCB_except_nest_cnt[r16]

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
	MYOSTKPT r12, r17
	add     -4, r12
	st.w    sp, 0[r12]                      /* スタックポインタの保存 */
	mov     r12, sp                         /* スタックポインタの切り替え */

FLABEL(ei_exception_3)
	UNLOCK_ALL_INT

#ifdef CFG_USE_PROTECTIONHOOK
	/* v850_cpu_exp_*を退避するスペースを確保 */
	addi    -12, sp, sp

	/* オフセットの生成 */
	GET_CID r11
	shl     2,  r11

	/* v850_cpu_exp_spの退避と設定 */
	Lea     _v850_cpu_exp_sp_table, r12
	add     r11, r12
	st.w    r9, 0[r12]
	st.w    ep, 0[sp]
	mov     sp, ep

	/* v850_cpu_exp_noの退避と設定 */
	Lea     _v850_cpu_exp_no_table, r12
	add     r11, r12
	ld.w    0[r12], r13
	sst.w   r13, 8[ep]
	st.w    r19, 0[r12]                     /* r19 例外要因 */

	/* v850_cpu_exp_pcの退避と設定 */
	Lea     _v850_cpu_exp_pc_table, r12
	add     r11, r12
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
	mov     E_OS_PROTECTION_EXCEPTION, r6
	jarl    _call_protectionhk_main, r31

#ifdef CFG_USE_PROTECTIONHOOK
	mov     sp, ep

	/* オフセットの生成 */
	GET_CID r11
	shl     2,  r11

	/* v850_cpu_exp_noを復帰 */
	Lea     _v850_cpu_exp_no_table, r12
	add     r11, r12
	sld.w   8[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_pcを復帰 */
	Lea     _v850_cpu_exp_pc_table, r12
	add     r11, r12
	sld.w   4[ep], r13
	st.w    r13, 0[r12]

	/* v850_cpu_exp_sp退避 */
	Lea     _v850_cpu_exp_sp_table, r12
	add     r11, r12
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
	MYCCB   r16, r12

	/*
	 *  割込み・例外のネスト回数のデクリメント
	 */
	ld.w    CCB_except_nest_cnt[r16], r13
	mov     -1, r14
	add     r14, r13
	st.w    r13, CCB_except_nest_cnt[r16]

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
	ld.b    CCB_nested_lock_os_int_cnt[r16], r13
	mov     -1, r14
	add     r14, r13
	st.b    r13, CCB_nested_lock_os_int_cnt[r16]
	cmp     r0, r13
	bne     ei_ret_exc_2

	/*
	 * OS割込み禁止解除
	 */
	 jarl	_set_intpri_unlock_os_int, lp

	/*
	 *  例外発生元へ復帰
	 */
FLABEL(ei_ret_exc_2)
	/*
	 * レジスタを復帰(EIPC/EIPSWも復帰)
	 */
	EI_RESTOREREG

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
	.global _interrupt_isr2
FLABEL(_interrupt_isr2)
	/* 割込み/例外の先頭で実行する処理 */
	PRE_INT_EXC

	/* 
	 * レジスタの保 (EIPC/EIPSWも保存)
	 */
	EI_SAVEREG

	/* 割込み優先度を取得(r11) */
	GET_ISPR r13
	sch1r    r13, r11
	add      -1,r11

	/* 自コアのCCBを取得 */
	MYCCB    r16, r19

	/* 
	 *  _current_iintpri の更新
	 *  割り込み処理中はISPRが自動的に設定されているため，PMRは設定する必要がない．
	 */
	ld.bu   CCB_current_iintpri[r16], r19
	st.b    r11,CCB_current_iintpri[r16]

	/*
	 *  多重割込みか判定
	 */
	ld.w    CCB_except_nest_cnt[r16], r6

	/*
	 *  割込み・例外のネスト回数のインクリメント
	 */
	add     1,  r6
	st.w    r6, CCB_except_nest_cnt[r16]
	cmp     1,  r6
	bne     interrupt_1     /* ネストしている場合はスタック切替無し */

	/*
	 *  タスクに割込んだ場合
	 *  スタックポインタの保存と入れ替え
	 */
	mov     sp , r13
	MYOSTKPT sp, r14
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
	ld.w    CCB_p_runtsk[r16], r6           /* 管理ブロックの先頭アドレス取得 */
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
	ld.w    TINIB_stk[r6], r6               /* タスクスタック先頭アドレス取得 */
	cmp     r6, r13                         /* スタックポインタ <= スタックの先頭の場合NG  */
	bnh     _stack_monitoring_error_isr

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
	br      _stack_monitoring_error_isr

	/* 
	 * 多重割込みの場合
	 */
FLABEL(int_nested)
	/*
	 *  割込みスタックのオーバフローチェック
	 */

	/*
	 *  ICIの場合はC2ISR用の
	 */
	GET_CID r11
	shl     2, r11
	Lea     _target_ici_intno_table, r6
	add     r11, r6
	ld.w    0[r6], r6
	andi    0xffff, r6, r6  /* コア指定をマスク */
	cmp     r12, r6
	be      ici_nested

	/*
	 *  スタック残量チェック方式
	 */
	Lea     _isr_p_isrcb_table, r6          /* テーブルのアドレスの取得 */
	add     r11, r6
	ld.w    0[r6], r7

	mov     r12, r6
	shl     2, r6                           /* 割込み番号を4倍してオフセットを生成  */
	add     r6, r7
	ld.w    0[r7],r7                        /* 該当するisrcb_tableのアドレスを取得 */
	ld.w    ISRCB_p_isrinib[r7], r7         /* 割込み番号に対応したISRCBのアドレス取得 */
	ld.w    ISRINIB_p_intinib[r7], r7       /* 割込み番号に対応したISRINIBのアドレス取得 */
	ld.w    INTINIB_remain_stksz[r7], r7    /* 割込み番号に対応したスタックサイズの取得 */
	br      cmp_stksz

FLABEL(ici_nested)
	/*
	 *  ICISRの使用するスタックサイズの取得
	 */    
	Lea     _ici_remain_stksz, r6
	add     r11, r6
	ld.w    0[r6], r7

FLABEL(cmp_stksz)
	Lea     _ostk_table, r6                 /* 割込みスタックの先頭アドレス取得 */
	add     r11, r6
	ld.w    0[r6], r6
	add     r6, r7                          /* 先頭アドレス＋ISRの使用するスタックサイズ */
	cmp     r7, sp                          /* スタックポインタ <= 先頭アドレス＋ISRの使用するスタックサイズの場合NG  */
	bnh     _stack_monitoring_error_isr

	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6                       /* このアドレスの示す先にマジックナンバーが入っている */
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
	bne     _stack_monitoring_error_isr

FLABEL(stack_monitoring_exit)
#endif /* CFG_USE_STACKMONITORING */

	/*
	 *  ディスパッチのためシステム用意したコア間割込みであれば，
	 *  ディスパッチャを呼び出す．
	 *      
	 *  割込み要因をr4に入れている，後続処理でも使用する
	 *  r3(my_ccb)の値はret_int_ici_dispatchにも使用する
	 */
#ifdef USE_ICI_DIS_HANDER_BYPASS
	GET_CID r11
	shl     2, r11
	Lea     _target_ici_intno_table, r6
	add     r11, r6
	ld.w    0[r6], r6
	andi    0xffff, r6, r6  /* コア指定をマスク */
	cmp     r12, r6
	bne     not_ici           /* コア間割込みで無ければ抜ける */

	ld.w    CCB_ici_request_map[r16], r6  /* ユーザ定義コア間割込みの要求であるかを判定 */
	cmp     r0, r6
	be      ret_int_ici_dispatch 

FLABEL(not_ici)
#endif /* USE_ICI_DIS_HANDER_BYPASS */

	/*
	 *  _callevel_stat 保存
	 */
	ld.h    CCB_callevel_stat[r16], r7
	Push    r7

	/*
	 *  _callevel_stat 設定
	 */
	mov     TCL_ISR2, r8
	or      r8, r7
	st.h    r7, CCB_callevel_stat[r16]

	/*
	 *  p_currentosap 設定
	 */
	ld.w    CCB_p_currentosap[r16], r7
	Push    r7

	/*
	 * p_runisr 設定
	 */
	ld.w    CCB_p_runisr[r16], r7
	Push    r7

	/*
	 * p_runisr 設定
	 */
	GET_CID r11
	shl     2, r11
	Lea     _isr_p_isrcb_table, r6          /* テーブルのアドレスの取得 */
	add     r11, r6
	ld.w    0[r6], r7
	shl     2, r12                          /* 割込み番号を4倍してオフセットを生成  */
	add     r12, r7
	ld.w    0[r7], r7
	st.w    r7, CCB_p_runisr[r16]

	/*
	 *  割込みハンドラのアドレスを読み込む(r6)
	 */
	Lea     _isr_table, r6
	add     r11, r6
	ld.w    0[r6], r6
	add     r12, r6

	/*
	 *  p_currentosap 設定
	 */
	cmp   r0, r7
	be    int_handler_call           /* p_runisr が NULL でなければ設定 */
	ld.w  ISRCB_p_isrinib[r7],r7     /* r5 <- ISR 初期化ブロックのアドレス */
	ld.w  ISRINIB_p_osapinib[r7],r7  /* r5 <- ISR が所属するOSアプリケーション */
	st.w  r7, CCB_p_currentosap[r16]     /* 実行中のISRが所属するOSアプリケーションを設定*/

	/*
	 *  割込みハンドラ呼び出し
	 */
FLABEL(int_handler_call)
	Lea     end_int_handler, r31
	ld.w    0[r6], r6
	jmp     [r6]

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
	GET_CID r11
	shl     2, r11
	Lea     _ostk_table, r6                 /* 割込みスタックの先頭アドレス取得 */
	add     r11, r6
	ld.w    0[r6], r6
	ld.w    0[r6], r6                       /* このアドレスの示す先にマジックナンバーが入っている */
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
	bne     _stack_monitoring_error_isr
#endif /* CFG_USE_STACKMONITORING */

	/*
	 * ISR2の不正終了チェック
	 */
	MYCCB   r16, r7
	ld.w    CCB_p_runisr[r16], r7
	cmp     r0, r7
	be      end_int_handler_1 
	jarl    _exit_isr2, r31

FLABEL(end_int_handler_1)
	MYCCB   r16, r6

	/*
	 * p_runisr の復帰
	 */
	Pop     r6
	st.w    r6, CCB_p_runisr[r16]

	/*
	 *  p_currentosap の復帰
	 */
	Pop     r6
	st.w    r6, CCB_p_currentosap[r16]

	/*
	 * _callevel_stat の復帰
	 */
	Pop     r6
	st.h    r6, CCB_callevel_stat[r16]

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
	st.b    r19, CCB_current_iintpri[r16]              /* current_iintpriを復帰 */

	/*
	 *  割込み・例外のネスト回数のデクリメント
	 */
	ld.w    CCB_except_nest_cnt[r16], r7
	add     -1, r7
	st.w    r7, CCB_except_nest_cnt[r16]

	/*
	 *  ネスト回数が1以上なら割込み元のISR2へ戻る
	 */
	cmp     r0, r7
	bne     ret_int_1

	/*
	 *  割込み元がタスクの場合
	 */
FLABEL(ret_int_task)
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
	 *  戻り先がタスクの場合，ディスパッチの必要があるかチェック
	 */
	ld.w    CCB_p_runtsk[r16], r6
	ld.w    CCB_p_schedtsk[r16], r7
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
	 *
	 *  G3M/G3KではINTCFG/INSPRの更新はdi状態で呼び出す必要がある．
	 *  ここは全割込み禁止状態で呼び出されるため問題ない．
	 */
	/* TODO CLEAR_ISPR r10, r12 */

	/*
	 *  OS割込み禁止状態に移行する
	 *
	 *  この時点でOS割込み禁止状態とするのは，dispatcherを実行する際に
	 *  OS割込み禁止状態になっている必要があるためである
	 *
	 *  (モデル上の)割込み優先度マスクは0であるため，ISR1以外の割込みは全て禁止する
	 */

	OUTER_LOCK_OS_INT r12, r13, r16
	/* ret_int で全割込み禁止状態としたため解除 */
	UNLOCK_ALL_INT

	/*
	 * コンテキストの退避
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

	/* スタックと次の起動番地を保存 */
	ld.w    CCB_p_runtsk[r16], ep
	sst.w   sp, TCB_sp[ep]
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
	 *  割込み・例外でコンテキスト保存した場合の復帰ルーチン
	 */
FLABEL(ret_int_r)
	/*
	 * コンテキストの復帰
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

	LOCK_ALL_INT

	/*
	 *  割込みからの復帰ルーチン
	 */
FLABEL(ret_int_1)

	/*
	 * EIPCとEIPSWを復帰
	 */
	mov     sp, ep

	/*
	 *  割込み優先度マスク(current_iintpri)に従って，割込みマスクを復帰する
	 */
	OUTER_UNLOCK_OS_INT  r7, r6, r16

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
 *  ディスパッチ要求用のプロセッサ間割込みのバイパス処理
 *
 *  共通部で用意されている ici_handler を呼び出さずにディスパッチャを
 *  呼び出す．
 *  r16(my_ccb)の値が入っていることを想定している
 */    
#ifdef USE_ICI_DIS_HANDER_BYPASS
FLABEL(ret_int_ici_dispatch)
	Lea   _shutdown_reqflg, r10      /* シャットダウン要求かのチェック */
	ld.w  0[r10], r10
	cmp   1, r10
	be    call_shtdown_os

	/*
	 *  ディスパッチ要求
	 */
	st.w  r0, CCB_ici_disreqflg[r16] /* ディスパッチ要求フラグのクリア */
	br    ret_int

FLABEL(call_shtdown_os)
	mov   E_OS_SHUTDOWN_OTHER_CORE, r6   /* 引数 */
	jarl  _internal_shutdownallcores, r31
	halt                                 /* ここには戻って来ない */
#endif /* USE_ICI_DIS_HANDER_BYPASS */


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

	MYCCB   r16, r10

	/* スタックと次の起動番地を保存 */
	ld.w    CCB_p_runtsk[r16], ep
	sst.w   sp, TCB_sp[ep]
	Lea     dispatch_r, r10
	sst.w   r10, TCB_pc[ep]
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  実行中タスクのタスクスタックのオーバフローチェック
	 */
	/*
	 *  スタックポインタチェック方式
	 */
	ld.w    CCB_p_runtsk[r16], r6           /* 管理ブロックの先頭アドレス取得 */
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
	ld.w    TINIB_stk[r6], r6               /* タスクスタック先頭アドレス取得 */
	cmp     r6, sp                          /* スタックポインタ <= スタックの先頭の場合NG  */
	bnh      stack_monitoring_error
	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
	bne     stack_monitoring_error
#endif /* CFG_USE_STACKMONITORING */
	br      dispatcher

	/* 
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
	MYCCB   r16, r12
	/* OS割込み禁止状態 */
	OUTER_LOCK_OS_INT r12, r13, r16
	/* 全割込み禁止解除状態へ */
	UNLOCK_ALL_INT
	br    dispatcher_0

	/*
	 *  現在のコンテキストを捨ててディスパッチ
	 */
	.global _exit_and_dispatch
FLABEL(_exit_and_dispatch)
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  実行中のタスクスタックのオーバフローチェック
	 */
	MYCCB   r16, r6

	/*
	 *  スタックポインタチェック方式
	 */
	ld.w    CCB_p_runtsk[r16], r6           /* 管理ブロックの先頭アドレス取得 */
	ld.w    TCB_p_tinib[r6], r6             /* タスク初期化ブロック先頭アドレス取得 */
	ld.w    TINIB_stk[r6], r6               /* タスクスタック先頭アドレス取得 */
	cmp     r6, sp                          /* スタックポインタ <= スタックの先頭の場合NG  */
	bnh      stack_monitoring_error
	/*
	 * マジックナンバーチェック方式
	 */
	ld.w    0[r6], r6
	mov     STACK_MAGIC_NUMBER, r7
	cmp     r7, r6
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
	MYCCB   r16, r21
	ld.w    CCB_p_runtsk[r16], r21
	cmp     r0, r21
	be      dispatcher_0_1
	mov     r21, r6                            /* 引数に_p_runtskを指定 */
	jarl    _log_dsp_enter, r31
FLABEL(dispatcher_0_1)
#endif	/* LOG_DSP_ENTER */

	MYCCB   r16, r22
	ld.h    CCB_callevel_stat[r16], r22
	mov     TSYS_DISALLINT, r20
	and     r22, r20
	be      dispatcher_1

	mov     ~TSYS_DISALLINT, r20
	and     r22, r20
	st.h    r20, CCB_callevel_stat[r16]
	UNLOCK_ALL_INT                             /* 全割込み禁止解除状態へ */

	/*
	 *  例外（割込み/CPU例外）のネスト回数はstart_dispatchや，dispatcher_3:
	 *  から来た場合は'1'となっているため，タスクスタックに切り換え後，
	 *  '0'に初期化する．r16に自コアのCCBを入れた状態で呼び出すこと．
	 */
FLABEL(dispatcher_1)
	ld.w    CCB_p_schedtsk[r16], r22
	st.w    r22, CCB_p_runtsk[r16]
	cmp     r22, r0
	be      dispatcher_3

	mov     r22, ep 
	sld.w   TCB_sp[ep], sp
	st.w    r0, CCB_except_nest_cnt[r16]

	ld.w    TCB_p_tinib[r22], ep
	ld.w    TINIB_p_osapinib[ep], r6
	st.w    r6, CCB_p_currentosap[r16]              /* p_currentosap ← 次に実行するタスクのOSアプリケーション */

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
	MYCCB   r16, r22
	ld.w    CCB_p_runtsk[r16], r22
	mov     r22, ep
	sld.w   TCB_pc[ep], r31
	jmp     [r31]

FLABEL(dispatcher_3)
	st.w    r0, CCB_p_currentosap[r16]		/* p_currentosap を NULL にする */
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
	MYOSTKPT sp, r20			/* 非タスクコンテキスト用のスタックの読み込みスタックポインタの入れ替え */

	mov     TCL_ISR2, r20		/* _callevel_stat をISR2に */
	st.h    r20, CCB_callevel_stat[r16]

	mov     1, r21
	st.w    r21, CCB_except_nest_cnt[r16]

	/*
	 *  OS割込み禁止解除状態へ
	 *  割込みを許可し，非タスクコンテキスト実行状態とし割込みを待つ
	 *
	 *  プロセッサを割込み待ちに移行させる処理と，割込み許可とは，不可
	 *  分に行なう必要がある
	 *  これを不可分に行なわない場合，割込みを許可した直後に割込
	 *  みが入り，その中でタスクが実行可能状態になると，実行すべきタス
	 *  クがあるにもかかわらずプロセッサが割込み待ちになってしまう
	 *
	 *  割込み待ちの間は，p_runtskをNULL（＝0）に設定しなければならない
	 *  このように設定しないと，割込みハンドラからGetTaskIDを呼び出
	 *  した際の動作が仕様に合致しなくなる
	 */
	di
	OUTER_UNLOCK_OS_INT r12, r13, r16
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
	OUTER_LOCK_OS_INT r12, r13, r16
	mov     TCL_TASK, r20
	st.h    r20, CCB_callevel_stat[r16]
	br      dispatcher_1

	/*
	 *  スタックオーバフロー時の処理
	 */
#ifdef CFG_USE_STACKMONITORING
	/*
	 *  スタックオーバフロー時プロテクションフックを呼び出し
	 *  スタックを更に壊さないため，割込みスタックの初期値を使用する
	 */
    .global _stack_monitoring_error_isr
FLABEL(_stack_monitoring_error_isr)
	MYCCB   r16, r12
	OUTER_LOCK_OS_INT r12, r13, r16             /* ISRから来た場合はOS割込み禁止になっていない */

FLABEL(stack_monitoring_error)
	ld.w    CCB_except_nest_cnt[r16], r13       /* _ostkptを使う場合はexcept_nest_cntをインクリメントする */
	add     1, r13
	st.w    r13, 0[r12]

	MYOSTKPT sp, r12                          /* スタックを更に壊さないため，割込みスタックの初期値を使用する */

	mov     E_OS_STACKFAULT, r6                 /* プロテクションフックの引数を設定 */
	jarl    _call_protectionhk_main, r31        /* プロテクションフックを呼び出し */
	/*
	 * プロテクションフックからは戻ってこない
	 */
	halt                                        /* ここには来ないはず */
#endif /* CFG_USE_STACKMONITORING */

/*
 *  タスク開始時処理
 *
 *  dispatcherから呼び出される
 *  r16に自コアのCCBが入っていることを想定
 */
	.global _start_r
FLABEL(_start_r)
	/*
	 *  OS割込み禁止解除前に現在優先度(TCB.curpri)を実行優先度(TINIB.exepri)に
	 *  変更(内部リソースを獲得)
	 */
	ld.w    CCB_p_runtsk[r16], ep
	sld.w   TCB_p_tinib[ep], r22
	ld.b    TINIB_exepri[r22], r21			/* TINIB.exepri -> TCB.curpri */
	sst.b   r21, TCB_curpri[ep]

	/*
	 *  OS割込み禁止解除状態へ
	 *  (モデル上の)割込み優先度マスクは0であるため，有効な割込みは全て許可する
	 */

	OUTER_UNLOCK_OS_INT r12, r13, r16

	/*
	 *  タスクが不正終了した場合は保護処理を行うよう
	 *  戻りアドレスにexit_taskを設定
	 */
	Lea     _exit_task, r31

	ld.w    TCB_p_tinib[ep], r22
	ld.w    TINIB_task[r22], r10
	jmp     [r10]

	.global _stack_change_and_call_func_1 
	.global _stack_change_and_call_func_2
FLABEL(_stack_change_and_call_func_1)
FLABEL(_stack_change_and_call_func_2)

	push    lp

	MYCCB   r16, r13

	ld.w    CCB_except_nest_cnt[r16], r13
	add     1, r13
	st.w    r13, CCB_except_nest_cnt[r16]
	cmp     1, r13
	bne     _stack_change_and_call_func_1_1

	/* スタックポインタの保存と入れ替え */
	MYOSTKPT r12, r11
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
	MYCCB   r16, r13
	ld.w    CCB_except_nest_cnt[r16], r13
	add     -1, r13
	st.w    r13, CCB_except_nest_cnt[r16]
	cmp     r0, r13
	bne     stack_change_and_call_func_r_1

	ld.w    0[sp], sp                    /* スタックポインタの復帰     */
FLABEL(stack_change_and_call_func_r_1)
	pop     lp
	jmp     [lp]




	/*
	 *  フックの呼び出し
	 *  フックの強制終了を備える
	 */
	.global _call_trusted_hook
FLABEL(_call_trusted_hook)

#ifndef TOPPERS_USE_PUSHSP
	addi    -60, sp, sp
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
#else /* TOPPERS_USE_PUSHSP */
	/* TODO pushsp  r20-r31 */
#endif /* TOPPERS_USE_PUSHSP */

	MYCCB   r16, r20

	ld.h  CCB_callevel_stat[r16], r20
	sst.h r20, 48[ep]
	ld.w  CCB_except_nest_cnt[r16], r20
	sst.w r20, 52[ep]
	ld.w  CCB_trusted_hook_savedsp[r16], r20
	sst.w r20, 56[ep]
	st.w  ep,  CCB_trusted_hook_savedsp[r16]

	Lea   trusted_hook_ret, lp  /* 戻りアドレス */
	mov   r6, r12  /* 引数の積み直し */
	mov   r7, r6
	jmp   [r12]

	/*
	 *  フックの強制終了
	 */
	.global _exit_trusted_shutdown_hook
FLABEL(_exit_trusted_shutdown_hook)
	MYCCB   r16, r20
	ld.w  CCB_trusted_hook_savedsp[r16], sp
	ld.h  48[sp], r20
	st.h  r20, CCB_callevel_stat[r16]
	ld.w  52[sp], r20
	st.w  r20, CCB_except_nest_cnt[r16]
	ld.w  56[sp], r20
	st.w  r20, CCB_trusted_hook_savedsp[r16]

FLABEL(trusted_hook_ret)
	MYCCB r16, r20
	st.w  r16, CCB_trusted_hook_savedsp[r16]
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
	sld.w    0[ep], r31
	addi    60, ep, sp
	ld.w     4[sp], r30  /* r30(ep) */
#else /* TOPPERS_USE_PUSHSP */
	/* TODO popsp  r20-r31 */
#endif /* TOPPERS_USE_PUSHSP */

	jmp     [lp]



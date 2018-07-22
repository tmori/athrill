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
 *  Copyright (C) 2010-2011,2013 by Meika Sugimoto
 * 
 *  上記著作権者は，以下の (1)~(4) の条件か，Free Software Foundation 
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
 *	プロセッサ依存モジュール アセンブリ言語部（V850用）
 */

#include "v850asm.inc"

#define	TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)		(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */
#define TOPPERS_LABEL_ASM
#include "kernel_impl.h"
#include "offset.h"

/* 外部参照宣言 */
	.globl	_set_intpri
	.globl	_call_texrtn
	.globl	_ipmflg
	
/*
 *	ディスパッチの開始
 *
 *	割込み禁止状態，lock_flag = trueでここへはやってくるが，
 *	割込みマスクレベルは0であるため，ここでCPUロック状態に
 *	設定して最初のディスパッチに移る．
 *	
 */
	.globl	_start_dispatch
_start_dispatch:
	/* 割込みマスクレベルをCPUロック状態の値に設定 */
	mov		INTPRI_LOCK , r6
	jarl	_set_intpri , lp
	/* 割込み禁止状態を解除 */
	ei
	/* ディスパッチャ本体へ */
	br		dispatcher

/*
 *  タスクディスパッチャ
 */
 	.globl	_dispatch
 	.globl	_exit_and_dispatch
_dispatch:
	/*
	 * コンテキストの退避 
	 */
	addi	-60, sp, sp
	st.w	ep , 0[sp]
	mov		sp, ep
	sst.w	r2,  56[ep]
	sst.w	r10, 52[ep]
	sst.w	r11, 48[ep]
	sst.w	r20, 44[ep]
	sst.w	r21, 40[ep]
	sst.w	r22, 36[ep]
	sst.w	r23, 32[ep]
	sst.w	r24, 28[ep]
	sst.w	r25, 24[ep]
	sst.w	r26, 20[ep]
	sst.w	r27, 16[ep]
	sst.w	r28, 12[ep]
	sst.w	r29,  8[ep]
	sst.w	r31,  4[ep]
	
	/* スタックと次の起動番地を保存 */
	Lea		_p_runtsk, ep
	sld.w	0[ep], ep
	sst.w	sp, TCB_sp[ep]
	Lea		dispatch_r, r10
	sst.w	r10, TCB_pc[ep]
	
	/*
	 *  タスクディスパッチャ
	 *  
	 *  ここはCPUロック状態，ディスパッチ前の処理単位が
	 *  コンテキスト退避済みある状態で来る必要がある.
	 */
_exit_and_dispatch:
dispatcher:
	Lea		_p_runtsk, r21
#ifdef LOG_DSP_ENTER
	ld.w		0[r21] , r6		/* 引数にp_runtskを指定 */
	jarl	_log_dsp_enter , lp
#endif	/* LOG_DSP_ENTER */
	Lea		_p_schedtsk, r20
	ld.w	0[r20], r22
	st.w	r22, 0[r21]
	cmp		r22 , r0
	be		pre_idle
	/* スタックと次の起動番地を復帰 */
	mov		r22, ep
	sld.w	TCB_pc[ep], r20
	sld.w	TCB_sp[ep], sp
#ifdef LOG_DSP_LEAVE
	ld.w	0[r21] , r6	/* 引数にp_runtskを指定 */
	jarl	_log_dsp_leave , lp
#endif	/* LOG_DSP_LEAVE */
	jmp		r20


	/*
	 * アイドルループ
	 */
pre_idle:
	/* 割り込みスタックに切り替え */
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	/* CPUロック状態の解除 */
	Lea		_lock_flag , r10
	st.w	r0 , 0[r10]
	di
	mov		INTPRI_ENAALL , r6
	jarl	_set_intpri , lp
	/* intnestを1にする */
	mov		1, r10
	Lea		_intnest, r11
	st.b	r10, 0[r11]
	/* reqflgのアドレスをロード */
	Lea		_reqflg , r20
idle_loop:	
	/* 割り込み待ち */
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


	/* 割り込み待ち */
	ei
	nop
	di
	
	/* reqflgのチェック */
	ld.w	0[r20], r10
	cmp		r10 , r0
	be		idle_loop
	
	/* reqflgのクリア */
	st.w	r0 , 0[r20]
	
	/* intnestをクリアする */
	Lea		_intnest, r10
	st.b	r0, 0[r10]
	/* CPUロック状態へ移行 */
	di
	mov		INTPRI_LOCK , r6
	jarl	_set_intpri , lp
	ei
	/* ディスパッチャへ */
	br	dispatcher
	
	/* ディスパッチャの復帰ルーチン */
dispatch_r:
	Lea		_p_runtsk, r10
	ld.w	0[r10], r10
	ld.w	TCB_enatex[r10], r11
	mov		(1 << TCB_enatex_bit) , r12
	tst		r11 , r12
	bz		ret_to_task
	ld.w	TCB_texptn[r10], r11
	cmp		r0, r11
	be		ret_to_task
	Lea		_ipmflg , r10
	ld.w	0[r10] , r10
	cmp		r0 , r10
	be		ret_to_task

	/* タスク例外の呼び出し */
	jarl	_call_texrtn, lp
ret_to_task:
	mov		sp, ep
	sld.w	56[ep], r2
	sld.w	52[ep], r10
	sld.w	48[ep], r11
	sld.w	44[ep], r20
	sld.w	40[ep], r21
	sld.w	36[ep], r22
	sld.w	32[ep], r23
	sld.w	28[ep], r24
	sld.w	24[ep], r25
	sld.w	20[ep], r26
	sld.w	16[ep], r27
	sld.w	12[ep], r28
	sld.w	 8[ep], r29
	sld.w	 4[ep], r31
	addi	60, ep, sp
	ld.w	0[sp], ep
	jmp		lp

/*
 *  タスク起動時処理
 */
 	.globl _start_r
_start_r:
	/* 割込みマスクレベルをCPUアンロック状態の値に設定 */
	di
	mov		INTPRI_ENAALL , r6
	jarl	_set_intpri , lp
	Lea		_lock_flag , r6
	st.w	r0 , 0[r6]
	
	Lea		_p_runtsk, ep
	ld.w	0[ep], ep
	ld.w	TCB_p_tinib[ep], ep
	ld.w	TINIB_exinf[ep], r6
	ld.w	TINIB_task[ep], r10
	Lea		_ext_tsk, lp
	
	/* タスク本体へ */
	ei		/* 割込み禁止状態を解除 */
	jmp		r10

/*
 *  割込みハンドラ
 *
 *  この時点でスタックは80byte分確保してあり，レジスタに割込みハンドラ情報が
 *  格納されている．
 *
 *  - r10：割込みハンドラアドレス
 *  - r11：割り込み優先度(内部表現)
 *  - r12：割込みハンドラ番号
 *
 *  スタックの構成は以下のとおり．
 *
 *    |----------------|<- 割込み発生時のSP
 * 76 | current_intpri |
 *    |----------------|
 * 72 |       r1       |
 *    |----------------|
 * 68 |       r6       |
 *    |----------------|
 * 64 |       r7       |
 *    |----------------|
 * 60 |       r8       |
 *    |----------------|
 * 56 |       r9       |
 *    |----------------|
 * 52 |      r10       |
 *    |----------------|
 * 48 |      r11       |
 *    |----------------|
 * 44 |      r12       |
 *    |----------------|
 * 40 |      r13       |
 *    |----------------|
 * 36 |      r14       |
 *    |----------------|
 * 32 |      r15       |
 *    |----------------|
 * 28 |      r16       |
 *    |----------------|
 * 24 |      r17       |
 *    |----------------|
 * 20 |      r18       |
 *    |----------------|
 * 16 |      r19       |
 *    |----------------|
 * 12 |      r30       |
 *    |----------------|
 *  8 |      r31       |
 *    |----------------|
 *  4 |      EIPC      |
 *    |----------------|
 *  0 |     EIPSW      |
 *    |----------------|<- 割込み発生時のSP - 80
 */
	.globl interrupt
interrupt:
	/*
	 *  残りのレジスタ退避， EPでのアクセスに切り替えて行う
	 *  (r10-r12は退避済み)
	 */
	st.w	ep,   12[sp]
	mov		sp ,  ep 
	sst.w	r1 ,  72[ep]
	sst.w	r6 ,  68[ep]
	sst.w	r7 ,  64[ep]
	sst.w	r8 ,  60[ep]
	sst.w	r9 ,  56[ep]
	sst.w	r14 , 36[ep]
	sst.w	r15 , 32[ep]
	sst.w	r16 , 28[ep]
	sst.w	r17 , 24[ep]
	sst.w	r18 , 20[ep]
	sst.w	r19 , 16[ep]
	sst.w	r31 ,  8[ep]
	/* 多重割込みのため，現在のEIPCとEIPSWを保存 */
	stsr	eipc,   r6
	sst.w	r6 ,   4[ep]
	stsr	eipsw,  r6
	sst.w	r6 ,   0[ep]
	
	/* ロックフラグを解除する */
	Lea		_lock_flag , r7
	st.w	r0 , 0[r7]
	
	/* 割込み発生前の割込みレベルを保存，発生中の割込みレベル(r11)に書き換え */
	Lea		_current_intpri , r13
	ld.bu	0[r13] , r14
	sst.w	r14 , 76[ep]
	st.b	r11 , 0[r13]
	
	/* 多重割り込みチェックとネストカウンタをインクリメント */
	Lea		_intnest, r13
	ld.bu	0[r13], r6
	add		1, r6
	st.b	r6, 0[r13]
	cmp		1 , r6
	bne		call_int_handler
save_tsksp:
	/* 初回割込みであればスタックをカーネルスタックへ切り替え */
	mov		sp , r13
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	add		-4 , sp
	st.w	r13 , 0[sp]
call_int_handler:
#ifdef LOG_INH_ENTER
	/* 割込みハンドラアドレス，割込み優先度，割込み番号を退避 */
	add		-8 , sp
	st.w	r10 , 0[sp]
	st.w	r12 , 4[sp]
	mov		r12 , r6
	jarl	_log_inh_enter , lp
	ld.w	0[sp] , r10
	ld.w	4[sp] , r12
	add		8 , sp
#endif	/* LOG_INH_ENTER */
#ifdef LOG_INH_LEAVE
	add		-4 , sp
	st.w	r12 , 0[sp]
#endif	/* LOG_INH_LEAVE */
	Lea		end_int_handler, lp
	/* 割り込み許可 */
	ei
	/* ハンドラ起動 */
	jmp		r10
end_int_handler:
	/* 割り込み禁止 */
	di
#ifdef LOG_INH_LEAVE
	ld.w	0[sp] , r6
	add		4 , sp
	jarl	_log_inh_leave , lp
#endif	/* LOG_INH_LEAVE */
	/* CPUロック状態をアンロック状態に戻す */
	Lea		_lock_flag , r10
	st.w	r0 , 0[r10]
	/* intnestをデクリメント */
	Lea		_intnest, r11
	ld.bu	0[r11], r10
	add		-1 , r10
	st.b	r10, 0[r11]
	cmp		r0 , r10
	be		return_to_task
return_from_int:
	/*
	 * レジスタ復帰 
	 */
	mov		sp , ep	/* EPへ切り替え */
	Lea		_current_intpri , r10
	sld.w	76[ep] , r6
	st.b	r6 , 0[r10]
	di
	jarl	_set_intpri , lp
	/* CPUロックフラグの解除 */
	Lea		_lock_flag , r6
	st.w	r0 , 0[r6]
	sld.w	 4[ep] , r10
	ldsr	r10 , eipc
	sld.w    0[ep] , r10
	ldsr    r10 , eipsw
	sld.w	72[ep] , r1
	sld.w	68[ep] , r6
	sld.w	64[ep] , r7
	sld.w	60[ep] , r8
	sld.w	56[ep] , r9
	sld.w	52[ep] , r10
	sld.w	48[ep] , r11
	sld.w	44[ep] , r12
	sld.w	40[ep] , r13
	sld.w	36[ep] , r14
	sld.w	32[ep] , r15
	sld.w	28[ep] , r16
	sld.w	24[ep] , r17
	sld.w	20[ep] , r18
	sld.w	16[ep] , r19
	sld.w	 8[ep] , r31
	addi	80 , ep, sp
	sld.w	12[ep] , r30

	reti	/* 割込み元へ */

return_to_task:
	/* スタックの復帰 */
	ld.w	0[sp], sp
	
	/* 遅延ディスパッチ及びタスク例外処理の処理要求があるかチェック */
	Lea		_reqflg, r10
	ld.w	0[r10], r11
	cmp		r0, r11
	be		return_from_int
	st.w	r0, 0[r10]			/* reqflgのクリア */
	
	/* ディスパッチ可能状態かチェック */
	Lea		_dspflg , r10
	ld.w	0[r10], r10
	cmp		r0, r10
	bz		check_texrtn_cond
	
	/* 割込み全解除状態フラグのチェック */
	Lea		_ipmflg , r10
	ld.w	0[r10], r10
	cmp		r0, r10
	bz		check_texrtn_cond
	
	/* タスク切り替えの必要があるかチェック */
	Lea		_p_schedtsk, r10
	ld.w	0[r10], r10
	Lea		_p_runtsk, r11
	ld.w	0[r11], r11
	cmp		r10,r11
	bne		ret_int
	br		check_texrtn_cond_1
	
check_texrtn_cond:
	/* 実行中タスクをロード */
	Lea		_p_runtsk, r11
	ld.w	0[r11], r11

check_texrtn_cond_1:
	/* タスク例外が発生しているかのチェックとタスク例外処理ルーチンの実行 */
	ld.w	TCB_enatex[r11] , r13
	mov		(1 << TCB_enatex_bit) , r12
	tst		r12 , r13
	bz		return_from_int
	ld.w	TCB_texptn[r11] , r11
	cmp		r0, r11
	bz		return_from_int
	jarl	_call_texrtn , lp
	br		return_from_int

/*
 *  さらに非スクラッチレジスタを退避する
 *
 *    |------------------|<- 割込み発生時のSP - 80
 * 40 |        r2        |
 *    |------------------|
 * 36 |       r20        |
 *    |------------------|
 * 32 |       r21        |
 *    |------------------|
 * 28 |       r22        |
 *    |------------------|
 * 24 |       r23        |
 *    |------------------|
 * 20 |       r24        |
 *    |------------------|
 * 16 |       r25        |
 *    |------------------|
 * 12 |       r26        |
 *    |------------------|
 *  8 |       r27        |
 *    |------------------|
 *  4 |       r28        |
 *    |------------------|
 *  0 |       r29        |
 *    |------------------|<- 割込み発生時のSP - 80 - 44
 */      
	.globl	ret_int
ret_int:
	addi	-44, sp, sp
	mov		sp , ep
	sst.w	r2 ,  40[ep]
	sst.w	r20 , 36[ep]
	sst.w	r21 , 32[ep]
	sst.w	r22 , 28[ep]
	sst.w	r23 , 24[ep]
	sst.w	r24 , 20[ep]
	sst.w	r25 , 16[ep]
	sst.w	r26 , 12[ep]
	sst.w	r27 ,  8[ep]
	sst.w	r28 ,  4[ep]
	sst.w	r29 , 0[ep]
	
	/* TCBに復帰先の情報を設定 */
	Lea		_p_runtsk, r10
	ld.w	0[r10], r10
	st.w	sp , TCB_sp[r10]
	Lea		ret_int_r, r11
	st.w	r11 , TCB_pc[r10]
	
	/* CPUロック状態に設定 */
	mov		INTPRI_LOCK , r6
	jarl	_set_intpri , lp
	Lea		_current_intpri , r6	/* 現在の仮想上の割込み優先度を0に */
	st.b	r0 , 0[r6]
	mov		1 , r7
	Lea		_lock_flag , r6			/* ロックフラグをtrueに */
	st.w	r7 , 0[r6]
	
	/* ディスパッチャへ */
	ldsr	r0, eipsw	/* 割込みは許可状態でよい */
	Lea		dispatcher, r10
	ldsr	r10, eipc
	reti

	.globl ret_int_r
ret_int_r:
	/* 非スクラッチレジスタの復帰 */
	mov		sp, ep
	sld.w	40[ep], r2
	sld.w	36[ep], r20
	sld.w	32[ep], r21
	sld.w	28[ep], r22
	sld.w	24[ep], r23
	sld.w	20[ep], r24
	sld.w	16[ep], r25
	sld.w	12[ep], r26
	sld.w	 8[ep], r27
	sld.w	 4[ep], r28
	sld.w	 0[ep], r29
	addi	44 , ep, sp
	
	/* おおもとの復帰処理へ飛ばす */
	br		check_texrtn_cond

/*
 *  CPU例外処理
 *
 *  CPU例外処理発生時にp_excinfに格納する情報は以下の通り．
 *  ・offset:0 CPU例外発生時が発生したPC
 *  ・offset:4 CPU例外発生時のSP
 *  ・offset:8 CPU例外発生時のPSW
 *  ・offset:12 例外コード
 */

	.globl	exception
exception:
	/*
	 *  残りのレジスタ退避， EPでのアクセスに切り替えて行う
	 *  (r10-r12は退避済み)
	 */
	st.w	ep,   16[sp]
	mov		sp ,  ep 
	sst.w	r1 ,  72[ep]
	sst.w	r6 ,  68[ep]
	sst.w	r7 ,  64[ep]
	sst.w	r8 ,  60[ep]
	sst.w	r9 ,  56[ep]
	sst.w	r14 , 36[ep]
	sst.w	r15 , 32[ep]
	sst.w	r16 , 28[ep]
	sst.w	r17 , 24[ep]
	sst.w	r18 , 20[ep]
	sst.w	r19 , 12[ep]
	sst.w	r31 ,  8[ep]
	/* 多重例外のため，現在のEIPCとEIPSWを保存 */
	stsr	eipc,   r6
	sst.w	r6 ,   4[ep]
	stsr	eipsw,  r6
	sst.w	r6 ,   0[ep]
	/* 割込み発生前の割込みレベルを保存 */
	Lea		_current_intpri , r13
	ld.bu	0[r13] , r14
	sst.w	r14 , 76[ep]
	
	/* 多重割込み/例外チェックとネストカウンタをインクリメント */
	Lea		_intnest, r13
	ld.bu	0[r13], r6
	add		1, r6
	st.b	r6, 0[r13]
	cmp		1 , r6
	bne		set_excinf
	
exc_save_tsksp:
	/* 初回割込みであればスタックをカーネルスタックへ切り替え */
	mov		sp , r13
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	add		-4 , sp
	st.w	r13 , 0[sp]
	
set_excinf:
	/* CPU例外情報をスタックに格納 */
	addi	-20 , sp , sp
	mov		sp , r6
	stsr	0 , r13				/* CPU例外発生時のEIPC */
	st.w	r13 , 0[r6]
	stsr	1 , r13				/* CPU例外発生時のEIPSW */
	st.w	r13 , 8[r6]
	stsr	4 , r13
	mov		0x0000ffff , r14	/* CPU例外発生時のEICC(ECRの下位16bit) */
	and		r14 , r13
	st.w	r13 , 12[r6]
	st.w	r11 , 4[r6]			/* CPU例外発生時のsp */
	Lea		_lock_flag , r14
	ld.w	0[r14] , r13
	st.w	r13 , 16[r6]		/* CPU例外発生時のロックフラグ */
	
	/* CPU例外発生時にCPUロック状態ならアンロックする */
	cmp 	r0 , r13
	be		call_exc_handler
	/*
	 *  TIPM_ENAALLで設定すると割込みハンドラで発生したCPU例外で
	 *  優先度が逆転するため，引数は現在の割込み優先度マスクを用いる
	 */
	/* st.w	r0 , 0[r14] */
	add		-8 , sp
	st.w	r10 , 4[sp]
	st.w	r6 , 0[sp]
	Lea		_current_intpri , r6
	ld.bu	0[r6] , r6
	jarl	_set_intpri , lp
	ld.w	4[sp] , r10
	ld.w	0[sp] , r6
	add		8 , sp
call_exc_handler:
#ifdef LOG_EXC_ENTER
	/* CPU例外ハンドラアドレス，例外発生時のSP，CPU例外番号を退避 */
	add		-12 , sp
	st.w	r10 , 0[sp]
	st.w	r6 , 4[sp]
	st.w	r12 , 8[sp]
	mov		r12 , r6
	jarl	_log_exc_enter , lp
	ld.w	0[sp] , r10
	ld.w	4[sp] , r6
	ld.w	8[sp] , r12
	add		12 , sp
#endif	/* LOG_EXC_ENTER */
#ifdef LOG_EXC_LEAVE
	add		-4 , sp
	st.w	r12 , 0[sp]
#endif	/* LOG_EXC_LEAVE */
	Lea		end_exc_handler , lp
	/* ハンドラ起動 */
	ei
	jmp		r10
end_exc_handler:
	di
#ifdef LOG_EXC_LEAVE
	ld.w	0[sp] , r6
	add		4 , sp
	jarl	_log_exc_leave , lp
#endif	/* LOG_EXC_LEAVE */
	
	/*
	 *  CPUロック状態をCPU例外発生前の状態に戻す
	 */
	ld.w	16[sp] , r10
	Lea		_lock_flag , r11
	st.w	r10 , 0[r11]
	cmp		r0 , r10
	bz		end_exc_handler_1
	mov		INTPRI_LOCK , r6
	jarl	_set_intpri , lp
	
end_exc_handler_1:
	/* CPU例外発生時の状態を格納した領域を破棄 */
	addi	20 , sp , sp
	/* intnestをデクリメント */
	Lea		_intnest, r11
	ld.bu	0[r11], r10
	add		-1 , r10
	st.b	r10, 0[r11]
	cmp		r0 , r10	
	/* 以降は共通処理 */
	be		return_to_task_from_exc
	br		return_from_exc


return_to_task_from_exc:
	/* スタックの復帰 */
	ld.w	0[sp], sp
	
	/* 遅延ディスパッチ及びタスク例外処理の処理要求があるかチェック */
	Lea		_reqflg, r10
	ld.w	0[r10], r11
	cmp		r0, r11
	be		return_from_exc
	st.w	r0, 0[r10]			/* reqflgのクリア */
	
	/* ディスパッチ可能状態かチェック */
	Lea		_dspflg , r10
	ld.w	0[r10], r10
	cmp		r0, r10
	bz		check_texrtn_cond
	
	/* 割込み全解除状態フラグのチェック */
	Lea		_ipmflg , r10
	ld.w	0[r10], r10
	cmp		r0, r10
	bz		check_texrtn_cond
	
	/* タスク切り替えの必要があるかチェック */
	Lea		_p_schedtsk, r10
	ld.w	0[r10], r10
	Lea		_p_runtsk, r11
	ld.w	0[r11], r11
	cmp		r10,r11
	bne		ret_int
	br		check_texrtn_cond_1

return_from_exc:
	/*
	 * レジスタ復帰 
	 */
	mov		sp , ep	/* EPへ切り替え */
	Lea		_current_intpri , r10
	sld.w	76[ep] , r6
	st.b	r6 , 0[r10]
	di
	jarl	_set_intpri , lp
	/* CPUロックフラグの解除 */
	sld.w	 4[ep] , r10
	ldsr	r10 , eipc
	sld.w    0[ep] , r10
	ldsr    r10 , eipsw
	sld.w	72[ep] , r1
	sld.w	68[ep] , r6
	sld.w	64[ep] , r7
	sld.w	60[ep] , r8
	sld.w	56[ep] , r9
	sld.w	52[ep] , r10
	sld.w	48[ep] , r11
	sld.w	44[ep] , r12
	sld.w	40[ep] , r13
	sld.w	36[ep] , r14
	sld.w	32[ep] , r15
	sld.w	28[ep] , r16
	sld.w	24[ep] , r17
	sld.w	20[ep] , r18
	sld.w	16[ep] , r19
	sld.w	 8[ep] , r31
	addi	80 , ep, sp
	sld.w	12[ep] , r30

	reti	/* 割込み元へ */

/*
 *  カーネルの終了処理の呼出し
 */
	.globl _call_exit_kernel
_call_exit_kernel:
	/* 非タスクコンテキストに切り替え */
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	jarl	_exit_kernel , lp
	/* 戻ってくることはないが，万が一戻ってきた場合のため無限ループ */
	di
_call_exit_kernel_loop:
	br		_call_exit_kernel_loop



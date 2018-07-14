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

/*
 *	プロセッサ依存モジュール アセンブリ言語部（V850用）
 */


#define TOPPERS_MACRO_ONLY
#define TOPPERS_ASM_MACRO
#define UINT_C(val)			(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)		(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val)		(val)		/* 型キャストを行うマクロ */
#include "kernel_impl.h"
#include "offset.h"
#include "v850asm.inc"


/* 外部参照宣言 */
	.globl	_set_intpri
	
/*
 *	ディスパッチの開始
 *
 * void
 * start_dispatch(void)
 * {
 * 	タスクコンテキストに切り換える
 * 	スタックをIDが1のタスクのスタック領域に切り換える
 * 	CPUロック状態・割込み優先度マスク全解除状態にする
 * 	dispatcher_0に分岐する
 * }
 */
	.globl	_start_dispatch
_start_dispatch:
	/*
	 * タスクコンテキストに切り替える
	 * intnestを0にする
	 */
	Lea		_intnest, r17
	st.b	r0, 0[r17]

	/*
	 * スタックをIDが1のタスクのスタック領域に切り換える
	 */
	 /* tcb_table[0].p_tinib → ep */
	Lea		__kernel_tcb_table, ep
	sld.w	TCB_p_tinib[ep], r18
	mov		r18, ep

	/* tcb_table[0].p_tinib->stk → sp */
	sld.w	TINIB_stk[ep], sp

	/* tcb_table[0].p_tinib->stksz → r19 */
	sld.w	TINIB_stksz[ep], r19

	add		r19, sp

	/* CPUロック解除状態にする */
	Lea		_lock_flag , r10
	st.w	r0, 0[r10]

	/*
	 * 割込み優先度マスク全解除状態にする
	 */
	mov 	INTPRI_ENAALL, r6
	jarl 	_set_intpri, lp

	/* dispatcher_0に分岐する */
	br		dispatcher_0

/*
 *  タスクコンテキストから呼ばれたサービスコール処理において，タスク切換え
 *  が必要になった場合には，タスクからの自発的なディスパッチ処理（dispatch）
 *  が呼び出される．
 *
 *  dispatchは，タスクコンテキスト・CPUロック状態・割込み優先度マスク全解除
 *  状態・ディスパッチ許可状態で呼び出される．
 */
 	.globl	_dispatch
 	.globl	_exit_and_dispatch
_dispatch:
	di
#ifdef TOPPERS_SUPPORT_OVRHDR
	addi	-8, sp, sp
	st.w	ep , 0[sp]
	st.w	lp , 4[sp]
	jarl	__kernel_ovrtimer_stop , lp
	mov		sp, ep
	sld.w	4[ep], lp
	sld.w	0[ep], ep
	addi	8, ep, sp
#endif	/* TOPPERS_SUPPORT_OVRHDR */
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
	Lea		__kernel_p_runtsk, ep
	sld.w	0[ep], ep
	sst.w	sp, TCB_sp[ep]
	Lea		dispatch_r, r10
	sst.w	r10, TCB_pc[ep]

_exit_and_dispatch:
	di
	Lea		_lock_flag , r10
	st.w	r0 , 0[r10]
	mov 	INTPRI_ENAALL, r6
	jarl 	_set_intpri, lp

	/* dispatcherに分岐する */
	br		dispatcher

	/* ディスパッチャの復帰ルーチン */
dispatch_r:
	/*
	 * di状態で呼び出される
	 */
	Lea		_lock_flag , r10
	mov		true, r11
	st.w	r11 , 0[r10]
	mov 	INTPRI_LOCK, r6
	jarl 	_set_intpri, lp

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
#ifdef TOPPERS_SUPPORT_OVRHDR
	addi	-8, sp, sp
	st.w	ep , 0[sp]
	st.w	lp , 4[sp]
	jarl	__kernel_ovrtimer_start , lp
	mov		sp, ep
	sld.w	4[ep], lp
	sld.w	0[ep], ep
	addi	8, ep, sp
#endif	/* TOPPERS_SUPPORT_OVRHDR */
	ei
	jmp		[lp]

	/*
	 *  タスクディスパッチャ
	 *  
	 *  dispatcherは，タスクコンテキスト・CPUロック状態・割込み優先度マスク全解
	 *  除状態・ディスパッチ許可状態で呼び出される．
	 */
dispatcher:
#ifdef LOG_DSP_ENTER
	Lea		__kernel_p_runtsk, r21
	ld.w		0[r21] , r6		/* 引数にp_runtskを指定 */
	jarl	_log_dsp_enter , lp
#endif	/* LOG_DSP_ENTER */
dispatcher_0:
	Lea		__kernel_p_runtsk, r21
	Lea		__kernel_p_schedtsk, r20
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
	jmp		[r20]	/* goto ret_int_r or dispatch_r or start_r */

	/*
	 * アイドルループ
	 */
pre_idle:
	/* 割り込みスタックに切り替え */
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp

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
	
	ei

	/* ここで割り込みを受け付ける */

	di
	/* ディスパッチャへ */
	br	dispatcher_0
	

/*
 *  タスク起動時処理
 *  start_rには，dispatcherのみから分岐し，
 *  その時の状態は，
 *   タスクコンテキスト・CPUロック状態・割込み優先度マスク全解除状態・ディスパッチ許可状態
 *  である．
 *
 *  ディスパッチャ本体から呼び出されるため，p_runtskはr21に入っている．
 *
 */
 	.globl _start_r
_start_r:
#ifdef TOPPERS_SUPPORT_OVRHDR
	jarl	__kernel_ovrtimer_start , lp
#endif	/* TOPPERS_SUPPORT_OVRHDR */
	/* 自タスク（p_runtsk）の起動番地を，拡張情報をパラメータとして呼び出す */
	mov		r21, ep		/* ep = p_runtsk */
	ld.w	0[ep], ep 	/* ep = *p_runtsk */
	ld.w	TCB_p_tinib[ep], ep
	ld.w	TINIB_exinf[ep], r6
	ld.w	TINIB_task[ep], r10
	/* タスク本体から復帰した場合にext_tskに分岐する設定を行う */
	Lea		_ext_tsk, lp
	/* タスク本体へ */
	ei
	jmp		[r10]

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
 * -4 |     task_sp    |
 *     ----------------
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
	sst.w	r13 , 40[ep]
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
	
	/* 割込み優先度マスクを，受け付けた割込みの割込み優先度に設定する */
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
#ifdef TOPPERS_SUPPORT_OVRHDR
	addi	-20, sp, sp
	st.w	ep , 0[sp]
	st.w	lp , 4[sp]
	st.w	r10 , 8[sp]
	st.w	r11 , 12[sp]
	st.w	r12 , 16[sp]
	jarl	__kernel_ovrtimer_stop , lp
	mov		sp, ep
	sld.w	16[r], r12
	sld.w	12[ep], r11
	sld.w	8[ep], r10
	sld.w	4[ep], lp
	sld.w	0[ep], ep
	addi	20, ep, sp
#endif	/* TOPPERS_SUPPORT_OVRHDR */
	mov		sp , r13
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	add		-4 , sp
	/*
	 * タスクのスタックポインタを保存する
	 */
	st.w	r13 , 0[sp]
call_int_handler:
#ifdef LOG_INH_ENTER
	/* 割込みハンドラアドレス，割込み優先度，割込み番号を退避 */
	addi	-20, sp, sp
	st.w	ep , 0[sp]
	st.w	lp , 4[sp]
	st.w	r10 , 8[sp]
	st.w	r11 , 12[sp]
	st.w	r12 , 16[sp]
	jarl	_log_inh_enter , lp
	mov		sp, ep
	sld.w	16[r], r12
	sld.w	12[ep], r11
	sld.w	8[ep], r10
	sld.w	4[ep], lp
	sld.w	0[ep], ep
	addi	20, ep, sp
#endif	/* LOG_INH_ENTER */
#ifdef LOG_INH_LEAVE
	/*
	 * 割り込みハンドラ番号格納レジスタr12は
	 * caller saved レジスタであるため，
	 * 退避する
	 */
	add		-4 , sp
	st.w	r12 , 0[sp]
#endif	/* LOG_INH_LEAVE */

	Lea		end_int_handler, lp
	ei
	/* 割込みハンドラを呼び出す */
	jmp		[r10]

end_int_handler:
	/* 割り込み禁止 */
	di
#ifdef LOG_INH_LEAVE
	ld.w	0[sp] , r6
	add		4 , sp
	jarl	_log_inh_leave , lp
#endif	/* LOG_INH_LEAVE */
	/* intnestをデクリメント */
	Lea		_intnest, r11
	ld.bu	0[r11], r10
	add		-1 , r10
	st.b	r10, 0[r11]
	cmp		r0 , r10
	be		return_to_task

	/* 割り込みコンテキスト */
	.globl return_from_int
return_from_int:
	/*
	 * di 状態で呼び出されること
	 * レジスタ復帰
	 */
	mov		sp , ep	/* EPへ切り替え */
	/*
	 * 少なくともカーネル管理の割込みを禁止した状態にする
	 */
	Lea		_current_intpri , r10
	sld.w	76[ep] , r6
	st.b	r6 , 0[r10]

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

	.globl return_to_task
return_to_task:
	/*
	 * di 状態で呼び出されること
	 */
	/* スタックをタスク用のスタックに戻し，タスクコンテキストに切り換える */
	ld.w	0[sp], sp

	Lea		__kernel_p_runtsk, r11
	ld.w	0[r11], r11
	/* p_runtsk == NULL */
	cmp 	r0, r11
	be 		return_to_idle

	/* タスク切り替えの必要があるかチェック */
	Lea		__kernel_p_schedtsk, r10
	ld.w	0[r10], r10
	cmp		r10,r11
	bne		ret_int
return_to_task_0:
	/*
	 * di状態で呼び出される
	 */
	/* (p_runtsk == p_schedtsk) */
#ifdef TOPPERS_SUPPORT_OVRHDR
	addi	-8, sp, sp
	st.w	ep , 0[sp]
	st.w	lp , 4[sp]
	jarl	__kernel_ovrtimer_start , lp
	mov		sp, ep
	sld.w	4[ep], lp
	sld.w	0[ep], ep
	addi	8, ep, sp
#endif /* TOPPERS_SUPPORT_OVRHDR */
	br		return_from_int


	.globl return_to_idle
return_to_idle:
	/*
	 * di 状態で呼び出されること
	 */
	/* タスクのスタックに保存したスクラッチレジスタ等を捨てる */
	mov		sp , ep	/* EPへ切り替え */

	/* 割込み優先度マスクは，全解除状態（TIPM_ENAALL）になって
							いなければならない … (*i) */
	mov 	INTPRI_ENAALL, r6
	jarl 	_set_intpri, lp

	/*	dispatcher_0に分岐する */
	br 		dispatcher_0


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
	/* (p_runtsk != p_schedtsk) */
	/*
	 * di状態で呼び出される
	 */

	/* 割込み優先度マスクは，全解除状態（TIPM_ENAALL）になって
										いなければならない … (*i) */
	mov 	INTPRI_ENAALL, r6
	jarl 	_set_intpri, lp

	/* スクラッチレジスタを除くすべてのレジスタをスタックに保存する */
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

	/* スタックポインタを自タスク（p_runtsk）のTCBに保存する */
	/* TCBに復帰先の情報を設定 */
	Lea		__kernel_p_runtsk, r10
	ld.w	0[r10], r10
	st.w	sp , TCB_sp[r10]

	/* ret_int_rを，実行再開番地として自タスクのTCBに保存する … (*a) */
	Lea		ret_int_r, r11
	st.w	r11 , TCB_pc[r10]
	
	/* dispatcherに分岐する */
	/* ディスパッチャへ */
	br dispatcher

	.globl ret_int_r
ret_int_r:
	/*
	 * di状態で呼び出される
	 */
	/*
	 * タスクコンテキスト・CPUロック状態・割込み優先度マスク全解除
	 * 状態・ディスパッチ許可状態
	 */
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
	br		return_to_task_0

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
	sst.w	r13 , 40[ep]
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
	st.w	r0 , 0[r14]
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
	jmp		[r10]
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
	be		return_to_task
	br		return_from_int

/*
 *  カーネルの終了処理の呼出し
 */
	.globl _call_exit_kernel
_call_exit_kernel:
	/* 非タスクコンテキストに切り替え */
	Lea		__kernel_istkpt, sp
	ld.w	0[sp], sp
	di
_call_exit_kernel_loop:
	br		_call_exit_kernel_loop



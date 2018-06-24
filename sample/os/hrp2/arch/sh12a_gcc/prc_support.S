/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *								Toyohashi Univ. of Technology, JAPAN
 *	Copyright (C) 2007		by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2007-2011 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
 *	Copyright (C) 2011-2012	by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 *	$Id: prc_support.S 2154 2011-07-13 07:09:55Z mit-kimai $
 */

/*
 *	プロセッサ依存モジュール アセンブリ言語部（SH12A用）
 *		コーディングルール
 *	 　		インデントは遅延スロットを表す。
 *	 　		該当する命令：jmp、jsr、bra、bsr、rts、rte、bf/s、bt/s、bsrf、braf
 */

#define TOPPERS_MACRO_ONLY
#define UINT_C(val) 	(val)		 /* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		 /* ulong_t型の定数を作るマクロ */
#define CAST(type, val) (val)		 /* 型キャストを行うマクロ */

/*
 *	アセンブラ用のマクロを有効にする
 */
#define TOPPERS_ASM_MACRO
#include "kernel_impl.h"
#include "offset.h"
#include "target_mpu.h"

/*
 *	注意
 *	  いずれも0x80以上になると符号拡張され、
 *	  期待した動作にならない。
 */
#if !(TCB_p_tinib < 0x80)
#error	Range error!  Check (TCB_p_tinib < 0x80)
#endif

#if !(TCB_texptn < 0x80)
#error	Range error!  Check (TCB_texptn < 0x80)
#endif

#if !(TCB_sp < 0x80)
#error	Range error!  Check (TCB_sp < 0x80)
#endif

#if !(TCB_pc < 0x80)
#error	Range error!  Check (TCB_pc < 0x80)
#endif

#if !(TCB_enatex < 0x80)
#error	Range error!  Check (TCB_enatex < 0x80)
#endif

#if !(TCB_enatex_bit < 8)
#error	Range error!  Check (TCB_enatex_bit < 8)
#endif

#if !(TCB_enatex_mask < 0x80)
#error	Range error!  Check (TCB_enatex_mask < 0x80)
#endif

/*
 *	カーネル内での全割込み禁止時のIPMの値
 *
 *	kernel_cfg.c 内のアセンブラで使用
 */
	.global _iipm_disall
_iipm_disall = IIPM_DISALL

/*
 *	タスクディスパッチャ
 *
 *	このルーチンは，
 *	 ・タスクコンテキスト(excnest_count = 0)
 *	 ・CPUロック状態
 *	 ・ディスパッチ許可状態
 *	 ・（モデル上の）割込み優先度マスク全解除状態
 *	で呼び出される．
 */

	.section .text_kernel
	.align 2
	.global _dispatch
_dispatch:
	/*
	 *	スクラッチレジスタを除くすべてのレジスタを保存
	 *	　pr,r8〜r14をスタックに保存
	 *	　r0〜r7は呼び出し元で保存しているため保存する必要が無い
	 */
#ifdef TOPPERS_SUPPORT_OVRHDR
    sts.l   pr, @-r15
    mov.l   _ovrtimer_stop_dis, r0
    jsr     @r0
    nop
    lds.l   @r15+, pr
#endif /* TOPPERS_SUPPORT_OVRHDR */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movmu.l r8,@-r15
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	sts.l  pr, @-r15
	mov.l  r14,@-r15
	mov.l  r13,@-r15
	mov.l  r12,@-r15
	mov.l  r11,@-r15
	mov.l  r10,@-r15
	mov.l  r9, @-r15
	mov.l  r8, @-r15
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	mov.l  _p_runtsk_dis,r2  /* r0 <- p_runtsk						   */
	mov.l  @r2,r0
	mov.l  r15,@(TCB_ssp,r0)  /* タスクスタックをTCBに保存 60以下ならOK */
	mov.l  _dispatch_r_dis,r1
	bra    _dispatcher
	 mov.l	r1,@(TCB_pc,r0)   /* 実行再開番地をTCBに保存  60以下ならOK	*/

_dispatch_r:
	/*
	 *	スクラッチレジスタの除くすべてのレジスタを復帰
	 *	　pr,r8〜r14をスタックから復帰
	 */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movmu.l @r15+,r8
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r8 		 /* レジスタを復帰						 */
	mov.l  @r15+,r9
	mov.l  @r15+,r10
	mov.l  @r15+,r11
	mov.l  @r15+,r12
	mov.l  @r15+,r13
	mov.l  @r15+,r14
	lds.l  @r15+,pr
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
#ifdef TOPPERS_SUPPORT_OVRHDR
    sts.l   pr, @-r15
    mov.l   r0, @-r15
    mov.l   _ovrtimer_start_dis, r0
    jsr     @r0
    nop
    mov.l   @r15+, r0
    lds.l   @r15+, pr
#endif /* TOPPERS_SUPPORT_OVRHDR */
	/*
	 *	タスク例外処理ルーチンの起動
	 *	dispatch_rはdispatcherから呼び出されるため，
	 *	tcbのアドレスはr0に入っている．
	 */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	bld.b  #TCB_enatex_bit, @(TCB_enatex,r0)
	bt   _dispatch_r_check_texptn	/* enatexがtrueならtexptnのチェックへ */
	rts/n						/*  dispatch()の呼び出し元へリターン  */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
 	/*
 	 * SH2E以前は
	 *　　・bdl.b命令なし
	 *　　・mov.b @(dips12, rn), rm命令なし（dispが0xfまで）
	 *　　
	 *　　・tst #TCB_enatex_mask,r0はディスティネーションがr0固定なの
	 *　　　で、使用できない。
	 */
	mov	   #TCB_enatex, r1
	mov.b  @(r0, r1), r2
	mov    #TCB_enatex_mask,r3
	tst    r2, r3
	bf   _dispatch_r_check_texptn	/* enatexがtrueならtexptnのチェックへ */
	rts							/*  dispatch()の呼び出し元へリターン  */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

	/* texptnが0でなく，ipmflgがtrueならば、タスク例外処理ルーチンの呼び出し */
_dispatch_r_check_texptn:
	mov.l  @(TCB_texptn,r0),r1	/* texptnをロード */
	tst    r1,r1
	bf	   _dispatch_r_check_ipmflg
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n						/*  dispatch()の呼び出し元へリターン  */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts							/*  dispatch()の呼び出し元へリターン  */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

_dispatch_r_check_ipmflg:
	mov.l  _ipmflg_dis,r1	/* ipmflgをロード */
	mov.l  @r1,r2
	tst    r2,r2
    bf      _dispatch_r_check_rundom
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n						/*  dispatch()の呼び出し元へリターン  */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts							/*  dispatch()の呼び出し元へリターン  */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

_dispatch_r_check_rundom:
	mov.l  _rundom_dis,r1	/* rundomをロード */
	mov.l  @r1,r2
	tst    r2,r2            /* rundom == 0(TACP_KERNEL) か？ */
	bt	   _dispatch_r_check_svclevel
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n						/*  dispatch()の呼び出し元へリターン  */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts							/*  dispatch()の呼び出し元へリターン  */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

_dispatch_r_check_svclevel:
    mov.b   @(TCB_svclevel, r0), r1  /* r1 = svclevel */
    tst     r1, r1              /* svclevel == 0 ? */
    bt      _dispatch_r_call_texrtn /* 拡張サービスコール呼出し中ならばリターン */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n						/*  dispatch()の呼び出し元へリターン  */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts							/*  dispatch()の呼び出し元へリターン  */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

	/*
	 * タスク例外処理ルーチンの呼び出し
	 * 　_call_texrtnからはdispatch()の呼び出し元へリターンするため、
	 * 　jmp命令で分岐する。
	 *　（prレジスタには、dispatch()の呼び出し元の番地が格納されており、
	 *　　ここでは書き換えない。）
	 */
_dispatch_r_call_texrtn:
	mov.l  _call_texrtn_dis,r2
	jmp    @r2
	 nop

/*
 * ディスパッチャの動作開始（prc_support.S）
 */
	.global _start_dispatch
_start_dispatch:
	/*
	 *	このルーチンは，カーネル起動時に，
	 *	　・すべての割込みを禁止した状態（全割込みロック状態と同等）
	 *	　・sp＝非タスクコンテキスト用スタックポインタ
	 *	の状態で呼び出されることを想定している．
	 *
	 *	prc_initializeで，
	 *	　・lock_flag＝true
	 *	　・saved_iipm＝IIPM_ENAALL
	 *	に初期化しているため，カーネル管理外の割込みを許可することで，
	 *	　・CPUロック状態
	 *	　・（モデル上の）割込み優先度マスク全解除状態
	 *	になる．
	 *	また，initialize_taskでdisdspをfalseに初期化しているため，ディ
	 *	スパッチ許可状態になっている．
	 *
	 *	よって、タスクコンテキストに切り替えるだけでよい。
	 */
	mov.l  _excnest_count_dis, r1 /* excnest_count を0に */
	mov    #0,r0
	bra    _dispatcher_0
	 mov.l	r0,@r1

/*
 *	現在のコンテキストを捨ててディスパッチ
 */
	.global _exit_and_dispatch
_exit_and_dispatch:
	/* ディスパッチャ本体（dispatcher）へ */

/*
 * ディスパッチャ本体
 */
_dispatcher:
#ifdef LOG_DSP_ENTER
	mov.l _log_dsp_enter_dis, r1 /* ログ出力関数 */
	mov.l _p_runtsk_dis, r2
	jsr   @r1
	 mov.l	@r2, r4 			 /* 引数		 */
#endif /* LOG_DSP_ENTER */
_dispatcher_0:
   /*
	*  ここにやって来るのは、上記の_dispatcherの他に、
	*  下記の2箇所からも合流してくる。
	*	   ・_start_dispatch
	*	   ・_dispatcher_2（実行すべきタスクがなく割込み待ちの場合）
	*
	*  このルーチンは，
	*	   ・タスクコンテキスト
	*	   ・CPUロック状態
	*	   ・ディスパッチ許可状態
	*	   ・（モデル上の）割込み優先度マスク全解除状態
	*  で呼び出される．
	*
	*  すなわち，
	*	   ・excnest_count=0
	*	   ・lock_flag=true
	*	   ・disdsp=false
	*	   ・dspflg=true
	*	   ・saved_iipm=IIPM_ENAALL
	*　となっている．実行再開番地へもこの状態のまま分岐する．
	*/
	mov.l  _p_schedtsk_dis,r12	/* r8 <- p_schedtsk 				*/
	mov.l  @r12,r8
	mov.l  _p_runtsk_dis,r2
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	tst    r8,r8				/* schedtsk があるか？				*/
	bt/s   _dispatcher_1		/* 無ければジャンプ 				*/
	 mov.l r8,@r2				/* schedtskをruntskに				*/
#else /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	mov.l  r8,@r2				/* schedtskをruntskに				*/
	tst    r8,r8				/* schedtsk があるか？				*/
	bt	   _dispatcher_1		/* 無ければジャンプ 				*/
#endif /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	mov.l  @(TCB_ssp,r8),r15 	/* TCBからタスクスタックを復帰		*/

	mov.l	@(TCB_p_tinib,r8), r1	   /* r1 = p_runtsk->p_tinib */
	mov.l	@(TINIB_p_dominib,r1), r2  /* r2 = p_runtsk->p_tinib->p_dominib */
	mov.l	_rundom_dis, r3            /* r3 = &rundom */
	mov.l	@(DOMINIB_domptn,r2), r4   /* r4 = domptn */
	mov.b	@(TCB_svclevel,r8), r0     /* r0 = p_runtsk->svclevel */
	cmp/eq	#0, r0                     /* p_runtsk->svclevelが0か？ */
	bf      _dispatcher_3               /* p_runtsk->svclevelが0でないなら次の命令をスキップ */
    nop
    /*
     *  rundom = p_runtsk->p_tinib->p_dominib->domptn
     */
    mov.l   r4, @r3
    bra     _dispatcher_4
    nop
_dispatcher_3:
    mov     #0, r0
    mov.l   r0, @r3       /* rundom = 0(svclevelが0でない場合) */
_dispatcher_4:
    tst     r4, r4          /* domptn == 0 （カーネルドメイン）か？*/
	bt  	_dispatcher_5  /* カーネルドメインの場合は，dispatcher_5へ */

	/*
	 *  MPUに保護領域を設定
	 *  現在は，領域の先頭アドレスと終了アドレスを
     *  4バイト（ページ単位）境界に配置しているため，
	 *  各アドレスに対し，ページ番号を取り出すための
     *  マスクをかける必要がない
	 */
	mov.l	_mpcsadr0, r3 /* 領域設定用レジスタのベースアドレスをr3に */

    /*
	 *  自タスクのユーザスタック領域 page4
     */
	mov.l	@(TINIB_sustk,r1), r0
    mov.l   r0, @(MPCADR_mpcsadr4,r3)
 	mov.l	@(TINIB_eustk,r1), r0
    mov.l   r0, @(MPCADR_mpceadr4,r3)

	/*
	 *  実行ドメインと保護ドメイン(MPUの設定)が同じならば，
     *  ドメイン毎のコード領域・データ領域をMPUに設定し直す必要はない
     */
    mov.l   _p_ctxdom_dis, r1   /* r1 = &p_ctxdom */	
    mov.l   @r1, r0             /* r0 = p_ctxdom */
	cmp/eq	r2, r0              /* 実行ドメインと保護ドメインが同じか? : 初期化ブロックのアドレスを比較 */
	bt      _dispatcher_5        /* 同じならdispatcher_5へ */
    nop
	mov.l	r2, @r1             /* p_ctxdom = p_dominib : 保護ドメインを切替え */
    
    /*
	 *  自保護ドメイン専用のコード領域 page0
     */
 	mov.l	@(DOMINIB_stext,r2), r0
    mov.l   r0, @(MPCADR_mpcsadr0,r3) /* 先頭番地 */
 	mov.l	@(DOMINIB_etext,r2), r0
    mov.l   r0, @(MPCADR_mpceadr0,r3) /* 終了番地 */

    /*
	 *  自保護ドメイン専用の定数領域 page1
     */
 	mov.l	@(DOMINIB_srodata,r2), r0
    mov.l   r0, @(MPCADR_mpcsadr1,r3) /* 先頭番地 */
 	mov.l	@(DOMINIB_erodata,r2), r0
    mov.l   r0, @(MPCADR_mpceadr1,r3) /* 終了番地 */

    /* 
     * 自保護ドメイン専用のデータ領域 page2
     */
 	mov.l	@(DOMINIB_sdata,r2), r0
    mov.l   r0, @(MPCADR_mpcsadr2,r3) /* 先頭番地 */
 	mov.l	@(DOMINIB_edata,r2), r0
    mov.l   r0, @(MPCADR_mpceadr2,r3) /* 終了番地 */

    /*
     * 共有リード/専用ライト領域の専用領域 page3
     */
 	mov.l	@(DOMINIB_ssrpw,r2), r0
    mov.l   r0, @(MPCADR_mpcsadr3,r3) /* 先頭番地 */
 	mov.l	@(DOMINIB_esrpw,r2), r0
    mov.l   r0, @(MPCADR_mpceadr3,r3) /* 終了番地 */

    /*
     * MPU領域の有効ビットをセット
     */
 	mov.l	@(DOMINIB_valid_map,r2), r2 /* 設定する値へのポインタ */
    mov.l   _mpcvld, r3                 /* 設定するデバイスレジスタ */
    mov.l   @r2, r0
    mov.l   r0, @r3

_dispatcher_5:
#ifdef LOG_DSP_LEAVE
	mov.l _log_dsp_leave_dis, r1 /* ログ出力関数 */
	jsr   @r1
	 mov   r8, r4				/* 引数	  */
#endif /* LOG_DSP_LEAVE */
	mov.l  @(TCB_pc,r8),r1		/* TCBから実行再開番地を復帰		*/
	jmp    @r1
    mov   r8, r0
    /*
     *  分岐先（実行再開番地）では、r0にp_schedtskを
     *  格納された状態を想定している。
     */

	/* 
	 * 実行すべきタスクがない場合の割込み待ち
	 */
_dispatcher_1:
	/* 
	 * イミディエイト値を毎回読み込むのは効率が悪いので事前に用意する
	 * 
	 * r0  : 0x00（割込みを許可するときにはこの値をSRに設定する）
	 * r1  : 0x01
	 * r2  : 割込み禁止のためのSRの値
	 * r8  : reqflgの番地
	 * r9  : excnest_countの番地
	 * r10 : lock_flagの番地
	 * r11 : saved_iipmの番地
	 * r12 : 番地_sleep_end
	 */
	mov    #0,r0
	mov    #1,r1
	mov.l  _iipm_disall_dis,r2		/* 割込み禁止時のSRの値 */
	mov.l  _reqflg_dis,r8
	mov.l  _excnest_count_dis,r9
	mov.l  _lock_flag_dis,r10
	mov.l  _saved_iipm_dis,r11
#ifndef OMIT_POWER_CONTROL
	mov.l _sleep_end_dis, r12
#endif /* OMIT_POWER_CONTROL */
	/*
	 *  割込みを許可したら、CPUロック解除状態になるよう準備する
	 */
	mov.l  r0,@r10					/* lock_flag←false */
	/*
	 *	非タスクコンテキストに切り替え
	 *	
	 *	ここで非タスクコンテキストに切り換えるのは，ここで発生する割込
	 *	み処理にどのスタックを使うかという問題の解決と，割込みハンドラ
	 *	内でのタスクディスパッチの防止という二つの意味がある．
	 */
	mov.l  _kernel_istkpt_dis,r13
	mov.l  r1, @r9			/* excnest_count←1 */
	mov.l  @r13,r15			/* スタック切り替え */

_dispatcher_2:
	/*
	 *　割込み待ち
	 *　
	 *　ディスパッチャの出口で実行すべきタスクがない(schedtsk==
	 *　NULL）場合は、sleep命令によってプロセッサを省電力モードに切
	 *　り替えて割込み待ちをしている。
	 *　
	 *　割込み許可後にsleep命令を実行しているため、割込み許可命令の
	 *　実行前に割込み要求が入っている場合（あるいは割込み許可直後、
	 *　sleep命令実行前に割込み要求が入った場合）、割込み許可命令の
	 *　実行と共に割込みが受け付けられ、その復帰後にsleepしたままに
	 *　なり、reqflgのチェックに進まない。
	 *　（1msec以内にタイマ割込みが入るため、実際にはsleepしたままと
	 *　いうことはない。）
	 *　
	 *　この問題は、割込みの許可とsleep状態への移行がアトミックに実
	 *　行できないことに起因する。
	 *
	 *	この問題を解決するため，rte命令の「PC,SR更新と遅延スロットの
	 *	命令を、割込みとは不可分に実行する」という仕様を利用し，割込
	 *	みの許可と省電力モードへの移行を不可分に行う．
	 */
#ifdef OMIT_POWER_CONTROL
	ldc    r0,sr				/*  割込み許可 				  */
#ifndef	LDC_NOP_DISPATCHER
	 nop
#else	/* LDC_NOP_DISPATCHER */
	 LDC_NOP_DISPATCHER			/*  CPUからINTCに伝達されるまでの遅延  */
#endif	/* LDC_NOP_DISPATCHER */

#else /*  OMIT_POWER_CONTROL  */
	mov.l r0, @-r15				/*	SRに設定する値をスタックに積む	*/
	mov.l r12, @-r15			/*	戻り番地をスタックに積む  */
	/*
	 *　割込みを許可する（スタックから読み出した値をSRに設定する）
	 *　と同時に省電力モードへ移行する。
	 *　
	 *　要検討
	 *　　ソフトウェアスタンバイモードに移行する前に
	 *　　バスステートコントローラのライトバッファから
	 *　　外部RAMへの書き込みが完了したことを確認する。
	 *　　（外部RAMへのダミーリードを行う。）
	 */
	rte
	 sleep						/*	（遅延スロット）  */
_sleep_end:
#endif /* OMIT_POWER_CONTROL */
	ldc    r2,sr			   /* 割込み禁止					 　*/
	mov.l  @r8,r6			   /* r6 <- reqflg					   */
	tst    r6,r6			   /* reqflg が falseなら			   */
	bt	   _dispatcher_2	   /*   dispatcher_2 へ				   */
	mov.l  r0,@r8	 		   /* reqflg←false					   */
	/*
	 *	タスクコンテキストに切り替え
	 *	　スタック切り替えは、分岐先の_dispatcher_0で行う。
	 */
	mov.l  r0,@r9			   /* excnest_count←0				   */
	/*
	 *	CPUロック状態に戻す．割込み待ちの間に実行した割込みハンドラによ
	 *	り，saved_iipmが書き換えられる可能性があるため，元の値に戻す必
	 *	要がある．dispatcherが実行される時は，saved_iipmがIIPM_ENAALL
	 *	となっているため，ここではsaved_iipmをIIPM_ENAALL（＝0）に戻せ
	 *	ばよい．
	 */
	mov.l  r1,@r10 		   /* lock_flag←true */
	bra    _dispatcher_0
	 mov.l	r0,@r11		   /* saved_iipm←0 */

	.align 2
#ifdef TOPPERS_SUPPORT_OVRHDR
_ovrtimer_start_dis:
    .long _ovrtimer_start
_ovrtimer_stop_dis:
    .long _ovrtimer_stop
#endif /* TOPPERS_SUPPORT_OVRHDR */
_excnest_count_dis:
	.long _excnest_count;
_iipm_disall_dis:
	.long IIPM_DISALL
_dispatch_r_dis:
	.long _dispatch_r
_p_runtsk_dis:
	.long _p_runtsk
_enatex_mask_dis:
	.long TCB_enatex_mask
_ipmflg_dis:
	.long _ipmflg
_call_texrtn_dis:
	.long _call_texrtn_stask
_p_schedtsk_dis:
	.long _p_schedtsk
_kernel_istkpt_dis:
	.long __kernel_istkpt
_reqflg_dis:
	.long  _reqflg
_lock_flag_dis:
	.long _lock_flag
_saved_iipm_dis:
	.long _saved_iipm
#ifndef OMIT_POWER_CONTROL
_sleep_end_dis:
	.long _sleep_end
#endif /* OMIT_POWER_CONTROL */
#ifdef LOG_DSP_ENTER
_log_dsp_enter_dis:
	.long _log_dsp_enter
#endif /* LOG_DSP_ENTER */
#ifdef LOG_DSP_LEAVE
_log_dsp_leave_dis:
	.long _log_dsp_leave
#endif /* LOG_DSP_LEAVE */
_rundom_dis:
    .long _rundom
_p_ctxdom_dis:
    .long _p_ctxdom
_mpcsadr0:
    .long MPCSADR0
_mpcvld:
    .long MPCVLD

/*
 *	カーネルの終了処理の呼出し
 *
 *	スタックを非タスクコンテキスト用に切り替え、
 *	exit_kernelへ分岐する。
 *	この関数は，全割込みロック状態で呼ばれる
 */
	.global _call_exit_kernel
_call_exit_kernel:
	mov.l _exit_kernel_exit,r0
	mov.l _kernel_istkpt_exit,r1
	jmp   @r0
	 mov.l @r1,r15

	.align 2
_exit_kernel_exit:
	.long _exit_kernel
_kernel_istkpt_exit:
	.long __kernel_istkpt


/*
 *	タスク起動処理
 *
 *	ディスパッチャから、以下の状態でここにくる。
 *  　・タスクコンテキスト
 *  　・CPUロック状態
 *  　・割込み優先度マスク全解除状態
 *  　・ディスパッチ許可状態
 *
 *	ディスパッチャから呼び出されるため，TCBのアドレスはr0に
 *	入っている
 */
	.section .text_kernel
	.global _start_stask_r
	.global _start_utask_r

    /* システムタスクの起動処理：HRPと同様 */
_start_stask_r:
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   r0, @-r15
    mov.l   _ovrtimer_start_sta, r0
    jsr     @r0
    nop
    mov.l   @r15+, r0
#endif /* TOPPERS_SUPPORT_OVRHDR */
	mov.l  _lock_flag_start, r1  /* CPUロック解除状態に */
	mov    #0x0, r2
	mov.l  r2,@r1
	ldc    r2,sr				 /* 割込み許可 */
	mov.l  @(TCB_p_tinib,r0),r3  /* p_runtsk->p_tinibをr3に  */
	mov.l  _ext_tsk_start,r5
	mov.l  @(TINIB_task,r3),r1	 /* タスク起動番地をr1に	 */
	lds    r5,pr				 /* 戻り番地を設定 */
	jmp    @r1
	 mov.l	@(TINIB_exinf,r3),r4 /* exinfを引数レジスタr4に  */

    /* ユーザタスクの起動処理：MPUを有効にする */
_start_utask_r:
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   r0, @-r15
    mov.l   _ovrtimer_start_sta, r0
    jsr     @r0
    nop
    mov.l   @r15+, r0
#endif /* TOPPERS_SUPPORT_OVRHDR */
	mov.l  _lock_flag_start, r1  /* CPUロック解除状態になるよう準備 */
	mov    #0x0, r2
	mov.l  r2,@r1
	mov.l  @(TCB_p_tinib,r0),r3  /* p_runtsk->p_tinibをr3に  */
	mov.l  _call_ext_tsk_start,r5
	mov.l  @(TINIB_task,r3),r1	 /* タスク起動番地をr1に	 */
	lds    r5,pr				 /* 戻り番地を設定 */
	mov.l  @(TINIB_exinf,r3),r4  /* exinfを引数レジスタr4に  */
    mov    #0xf0, r5
    ldc    r5,sr                 /* 全割込み禁止 */
    /* 
     * ユーザスタックの復帰
     */
    mov.l  @(TINIB_eustk,r3),r15 /* sp = ustk_bottom - 4 */
    add    #4, r15               /* sp = ustk_bottom */
    mov.b  r2, @(TCB_priv_mode,r0) /* priv_mode = 0 */
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_start, r0
	mov.l	_mpuvalid_start, r2
    mov.l   start_utask_r_1_start, r3
    jmp     @r3
    nop

	.align 2
#ifdef TOPPERS_SUPPORT_OVRHDR
_ovrtimer_start_sta:
    .long _ovrtimer_start
#endif /* TOPPERS_SUPPORT_OVRHDR */
_lock_flag_start:
	.long _lock_flag
_ext_tsk_start:
	.long __kernel_ext_tsk
_call_ext_tsk_start:
	.long _call_ext_tsk
_mpcmpen_start:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_start:
	.long 0xaaaa0001
start_utask_r_1_start:
    .long start_utask_r_1

    .section .text_shared
    .align 2
start_utask_r_1:
    /*
     *  R0: MPUENのアドレス
     *  R2: マジックナンバー
     */
	mov.l	r2, @r0
    
    /*
     *  cpuロック解除
     */
	mov    #0x0, r2
	ldc    r2,sr				 /* 割込み許可 */

    jmp    @r1
    nop

_call_ext_tsk:
    mov     #TFN_EXT_TSK, r1
    mov.l   _svc_entry_ext_tsk, r0
    jsr     @r0
    nop
    mov     #TFN_EXT_KER, r1
    mov.l   _svc_entry_ext_tsk, r0
    jsr     @r0
    nop

    .align 4
_svc_entry_ext_tsk:
    .long __kernel_svc_entry

/*
 *	割込みの際のスタックポインタが指し示す位置からSR,PCのコピーがある
 *	場所までのオフセット
 *　　
 *　　SP_PC_OFFSET_INT：_ret_int_2の時点での、スタックに積まれた
 *　　　　　　　　　　　PCへのオフセット
 *　　SP_SR_OFFSET_INT：_ret_int_3の時点での、スタックに積まれた
 *　　　　　　　　　　　SRへのオフセット
 *　　
 *　　
 * スタック切り替え前のスタック
 * （割込み受付時のコンテキスト用スタック）
 *
 * (1) レジスタバンクを使用しない場合
 *		  ----------------------
 *	0	 |		   PR			|	<-- _ret_int_2 / _ret_int_3 の時点
 *		  ----------------------
 *	1	 |		  MACH			|
 *		  ----------------------
 *	2	 |		  MACL			|
 *		  ----------------------
 *	3	 |		   R0			|
 *		  ----------------------
 *	4	 |		   R1			|
 *		  ----------------------
 *	5	 |		   R2			|
 *		  ----------------------
 *	6	 |		   R3			|
 *		  ----------------------
 *	7	 |		   R4			|
 *		  ----------------------
 *	8	 |		   R5			|
 *		  ----------------------
 *	9	 |		   R6			|
 *		  ----------------------
 * 10	 |		   R7			|
 *		  ----------------------
 * 11	 |		   PC			|
 *		  ----------------------
 * 12	 |		   SR			|
 *		  ----------------------
 *
 * (2) レジスタバンクを使用する場合
 *		  ----------------------
 *	0	 |		   PR			|	<-- _ret_int_3 の時点
 *		  ----------------------
 *	1	 |		  MACH			|
 *		  ----------------------
 *	2	 |		  MACL			|
 *		  ----------------------
 *	3	 |		   R0			|
 *		  ----------------------
 *	4	 |		   R1			|
 *		  ----------------------
 *	5	 |		   R2			|
 *		  ----------------------
 *	6	 |		   R3			|
 *		  ----------------------
 *	7	 |		   R4			|
 *		  ----------------------
 *	8	 |		   R5			|
 *		  ----------------------
 *	9	 |		   R6			|
 *		  ----------------------
 * 10	 |		   R7			|
 *		  ----------------------
 * 11	 |		   PC			|	<-- _ret_int_2 の時点
 *		  ----------------------
 * 12	 |		   SR			|
 *		  ----------------------
 */
#ifdef USE_BANKED_REG
#define    SP_PC_OFFSET_INT    0
#else /* USE_BANKED_REG  */
#define    SP_PC_OFFSET_INT    (11*4)
#endif /* USE_BANKED_REG */

#define    SP_SR_OFFSET_INT    (12*4)


/*
 *	共通割込みの入口処理
 *　　割込み要因に依らず、共通な部分
 *　　（コンフィギュレータが生成する入口処理（kernel_cfg_asm.S）からの
 *　　　続き）
 *	
 *	割込み要因毎に展開されるルーチンでr4,r5,r6,r7を保存し、
 *　　全割込み禁止状態
 *　　r5：割込み番号
 *　　r6：割込みハンドラの先頭アドレス
 *　　r7：割込み受付直後のSRのコピー
 *	の状態でここに来る。
 *	（r4に割り当てないのは、CPU例外ハンドラの処理となるべく共通にするため）
 */
	.section .text_shared
	.global _interrupt_entry
_interrupt_entry:
#ifndef  USE_BANKED_REG
					/*	残りのスクラッチレジスタをスタックに積む  */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l r3, @-r15
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  r3,@-r15
	mov.l  r2,@-r15
	mov.l  r1,@-r15
	mov.l  r0,@-r15
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	sts.l  mach,@-r15
	sts.l  macl,@-r15
	sts.l  pr,@-r15
#endif /* USE_BANKED_REG */
    /* 
     *  MPU無効
     */
	mov.l	_mpcmpen_int, r1
	mov.l	_mpuinvalid_int, r0
	mov.l	r0, @r1
    mov.l   _interrupt_entry_0_int, r0
    jmp     @r0
    nop

    .align 4
_interrupt_entry_0_int:
    .long _interrupt_entry_0
_mpuinvalid_int:
	.long 0xaaaa0000
_mpcmpen_int:
	.long MPCMPEN   /* MPU有効 */

    .section .text_kernel
_interrupt_entry_0:
	mov.l  _excnest_int, r0    /* 割込み/CPU例外ネストカウンタのチェック */
	mov.l  @r0,r1
	tst    r1,r1				 /* 割込み発生時のコンテキストを判定  */
	add    #0x1,r1				 /* 割込みネストカウンタをインクリメント */
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	bf/s   _interrupt_entry_1 	 /* 多重割込みならジャンプ	*/
	 mov.l	r1,@r0
#else	/*	TOPPERS_SUPPORT_SH2E_INSTRUCTION  */
	mov.l  r1,@r0
	bf	   _interrupt_entry_1 	 /* 多重割込みならジャンプ	*/
#endif	/* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   r7, @-r15
    mov.l   r6, @-r15
    mov.l   r5, @-r15
    mov.l   _ovrtimer_stop_int, r0
    jsr     @r0
    nop
    mov.l   @r15+, r5
    mov.l   @r15+, r6
    mov.l   @r15+, r7
#endif /* TOPPERS_SUPPORT_OVRHDR */
	mov.l  _kernel_istkpt_int,r3 /* スタック切り替え　*/
	mov    r15,r2
	mov.l  @r3,r15
	mov.l  r2,@-r15 			 /* タスクスタックの保存 */
_interrupt_entry_1:
	ldc    r7,sr				 /* 割込み許可	*/
	/*
	 * 割込み番号を保存
	 * 　ここで保存した割込み番号には、２つの意味がある。
	 * 　　・default_int_handlerの引数
	 * 　　・log_inh_leave_intの引数
	 */
	mov.l  r5,@-r15

#ifdef LOG_INH_ENTER
	mov.l _log_inh_enter_int, r1 /* ログ出力関数 */
	mov.l  r6,@-r15 	  /* r6（割込みハンドラの先頭アドレス）保存 */
	jsr   @r1
	 mov  r5, r4				 /* 引数		  */
	mov.l @r15+, r6 			 /* r6復帰		  */
#endif /* LOG_INH_ENTER */

	jsr    @r6					 /* 割込みハンドラ呼び出し  */
#ifndef LOG_INH_LEAVE
	  mov.l @r15+,r4			 /* default_int_handler用の引数 */
	  							 /*  （割込み番号）  */
#else  /* LOG_INH_LEAVE */
	  mov.l @r15, r4			 /* default_int_handler用の引数 */
	  							 /*  （スタックはそのまま）  　 */
	mov.l _log_inh_leave_int, r1 /* ログ出力関数				*/
	jsr  @r1
	 mov.l @r15+,r4 			 /* 割込み番号を復帰			*/
#endif /* LOG_INH_LEAVE */

	/*
	 * 割込みハンドラ出口処理
	 *
	 *	割込み/CPU例外ネストカウンタをデクリメントし，戻り先のコンテキ
	 *	ストを判定してスタックを入れ替える．
	 *	カーネル管理外の割込みをサポートする場合は，両者の間にカーネル
	 *	管理外の割込みが入る場合がある．カーネル管理外の割込みの入口は，
	 *	カーネル管理の割込みと同様であるため，割込み/CPU例外ネストカウ
	 *	ンタを参照して，スタックを入れ替える．そのため，割込み/CPU例外
	 *	ネストカウンタとスタックの入れ替えはカーネル管理外の割込みに対
	 *	しても，不可分に行う必要がある．
	 */
_ret_int:
	mov.l  _iipm_disall_int,r2  /*  全割込み（カーネル管理外も含め）禁止  */
	ldc    r2,sr
	/*
	 *　割込み/CPU例外ネストカウンタをデクリメントし、
	 *　戻り先が非タスクコンテキストならすぐにリターン
	 */
	mov.l  _excnest_int,r0
	mov.l  @r0,r1
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	dt	   r1
	bf/s   _ret_int_1_1
	 mov.l r1,@r0
#else /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	add    #0xff, r1
	tst    r1,r1
	mov.l  r1,@r0
	bf	   _ret_int_1_1
    nop
#endif /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	
	mov.l  @r15,r15 		/* 戻り先がタスクなのでスタックを戻す	  */
	mov.l  _reqflg_int,r4	/* reqflgのチェック */
	mov.l  @r4,r1
	tst    r1,r1			/* reqflgがfalseならret_int_1へ 		  */
	bt	   _ret_int_1

	/*
	 * 戻り先がタスクでreqflgがセットされている場合の処理へ分岐
	 * 　ret_int_2に分岐してディスパッチしない場合があるのでreqflgはクリアしない
	 */
    /*
     *  priv_modeを切り替える前に、全割込み禁止を解除しない
     */
	bra   _ret_int_2
    nop

_ret_int_1:
#ifdef TOPPERS_SUPPORT_OVRHDR
    /*
     *  ここでは、CPU例外が発生しないものとする
     *  CPU例外が発生した場合、priv_modeとMPU ON/OFFが
     *  矛盾する
     */
    mov.l   _ovrtimer_start_int, r0
    jsr     @r0
    nop
#endif /* TOPPERS_SUPPORT_OVRHDR */
   	/*
	 *	割込み処理からのリターンにより，CPUロック解除状態に移行するよう
	 *	準備する．割込み優先度マスクは，RTE命令により元の値に戻るため，
	 *	lock_flagをfalseにしておけばよい（lock_flagをfalseにするのは，
	 *	CPUロック状態のまま割込みハンドラから戻った時の対策）．
	 */
	mov.l _lock_flag_ret, r0
	mov    #0x0, r1
	mov.l  r1,@r0

    mov.l   _rundom_ret_int, r0     /* r0 = &rundom */
    mov.l   @r0, r0                 /* r0 = rundom */
    cmp/eq  #0, r0        /* rundom == TACP_KERNEL か？*/     
    bt      _ret_int_1_2            /* カーネルドメインならば_ret_int_1_2へ */
    mov.l   _p_runtsk_ret, r0       /* r0 = &p_runtsk */
    mov.l   @r0, r0                 /* r0 = p_runtsk */
    mov.b   @(TCB_priv_mode,r0), r0 /* r0 = p_runtsk->priv_mode */
    cmp/eq  #0x00, r0               /* ユーザモード（priv_mode == 0） か？ */
    bf      _ret_int_1_2            /* 特権モードならば_ret_int_1_2へ */
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_ret_int, r0
    mov.l   _ret_int_1_3_ret_int, r2
	mov.l	_mpuvalid_ret_int, r1
    jmp     @r2
    nop

	.align 2
_ret_int_1_3_ret_int:
    .long _ret_int_1_3

    .align 2
	/*
	 *	割込み処理からのリターン
	 *	　　ディスパッチャやタスク例外処理ルーチンを呼び出さずに
	 *	　　直接、割込み元へリターンする。
	 *	　　復元すべきレジスタがレジスタバンクに格納されている点が
	 *	　　_ret_int_r_returnと異なる。
	 */
_ret_int_1_1:
	/*
	 *	割込み処理からのリターンにより，CPUロック解除状態に移行するよう
	 *	準備する．割込み優先度マスクは，RTE命令により元の値に戻るため，
	 *	lock_flagをfalseにしておけばよい（lock_flagをfalseにするのは，
	 *	CPUロック状態のまま割込みハンドラから戻った時の対策）．
	 */
	mov.l _lock_flag_ret, r0
	mov    #0x0, r1
	mov.l  r1,@r0
_ret_int_1_2:
#ifdef USE_BANKED_REG
	resbank
#else	/*	USE_BANKED_REG	*/
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
#endif	/* USE_BANKED_REG */
	rte
	 nop

	.align 2
_kernel_istkpt_int:
	.long __kernel_istkpt
_excnest_int:					/* 割込み／CPU例外ネストカウンタ */
	.long  _excnest_count
_reqflg_int:
	.long  _reqflg
_iipm_cpu_lock_int:				/* カーネル管理内割込み禁止用マスク */
	.long  IIPM_LOCK			/* ipm以外のビットはゼロで良い */
_iipm_disall_int:				/* 割込み全禁止用マスク */
	.long  IIPM_DISALL
#ifdef LOG_INH_ENTER
_log_inh_enter_int:
	.long  _log_inh_enter
#endif /* LOG_INH_ENTER */
#ifdef LOG_INH_LEAVE
_log_inh_leave_int:
	.long  _log_inh_leave
#endif /* LOG_INH_LEAVE */

    .section .text_shared
    .align 2
    .global _ret_int_1_3
_ret_int_1_3:
    /*
     *  R0: MPUENのアドレス
     *  R1: マジックナンバー
     */
	mov.l	r1, @r0
    
#ifdef USE_BANKED_REG
	resbank
#else	/*	USE_BANKED_REG	*/
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
#endif	/* USE_BANKED_REG */
	rte
	nop

    .section .text_kernel
    .align 2
	/*
	 *	ここへは割込みハンドラの出口処理から分岐してくる
	 *	
	 *	戻り先がタスクでreqflgがセットされている場合のみここにくる。
	 */
_ret_int_2:
	/*
	 *　CPU例外/割込み受付直後に割込み禁止になっていないことへの対策
	 *　（戻り先が割込み／CPU例外の入口処理か否かの判別）
	 *　
	 *　SH12AではCPU例外/割込み受付直後に割込み禁止になっていないため、
	 *	割込みAの入口処理中に別の割込みBが入る可能性がある。
	 * （これはハードウェアのアーキテクチャ上避けようがない。）
	 *	割込みハンドラB内でタスク切り替えを起こすようなサービスコール
	 *	を呼ぶと割込みBの出口処理で別のタスクにディスパッチしてしまい、
	 *	タスク2から元のタスクに戻ってくるまで、割込みAの処理が遅れてし
	 *	まう。
	 *	これを防ぐため、割込み/例外の入口処理をまとめてvector_entryセ
	 *	クションに置き，戻り先のアドレスがこの範囲内ならディスパッチ
	 *	せずにリターンする．
	 */
	mov.l  @(SP_PC_OFFSET_INT,r15),r0	/* r0 <-スタックに積まれたpc */
	mov.l  _vector_entry_start_ret, r1
	mov.l  _vector_entry_end_ret, r2
	cmp/hs r1, r0
	bf	   _ret_int_2_1
	cmp/hi r0, r2
	bt	   _ret_int_1_1

	/*
	 *	ディスパッチャもしくはタスク例外処理の呼び出し
	 */
_ret_int_2_1:
	/*
	 *	レジスタバンクを使用する場合はスクラッチレジスタを
	 *	スタックに格納する
	 */
#ifdef USE_BANKED_REG
	resbank
	movml.l  r7, @-r15
	sts.l	 mach,@-r15
	sts.l	 macl,@-r15
	sts.l	 pr,@-r15
#endif /* USE_BANKED_REG */

	/*
	 *	ディスパッチャの呼び出し
	 *
	 *	ここへは，CPU例外ハンドラの出口処理からも分岐してくる．
	 *
	 *	ここでは，戻り先がタスクであり，スタックは，タスクスタックに
	 *	スクラッチレジスタのみが保存された状態になっている．また，
	 *	カーネル管理の割込みを禁止した状態となっている．
	 */
_ret_int_3:
	/*
	 *　reqflgのクリア
	 *　
	 *　ここに到達する前にreqflgをクリアしてしまうと、上記の割込みA
	 *　の出口処理でディスパッチャ呼び出しが必要なケースでもディス
	 *　パッチャが呼び出されない。
	 *　（割込みBでreqflgをセットして、割込みAでreqflgに変化がない場
	 *　　合）
	 */
	mov    #0x0, r0
	mov.l  _reqflg_ret,r1
	mov.l  r0, @r1

	/*
	 *	CPUロック状態に移行する．カーネル管理の割込みは既に禁止して
	 *	いるので，lock_flagとsaved_iipmを更新する．saved_iipmは，戻り
	 *	先の割込み優先度マスク（の内部表現）に設定する．
	 *
	 *	この時点でCPUロック状態とするのは，dispatcherへ分岐する時と，
	 *	call_texrtnを呼び出す時に，CPUロック状態になっている必要がある
	 *	ためである．
	 */
	mov.l  _lock_flag_ret, r2	/*  CPUロック状態に移行する  */
	mov    #0x01,r1

	/*
	 * スタック上に保存されている割込み前のIPMを取り出して、saved_iipmに
	 *
	 *　　タスクコンテキストに戻るのに、単にsaved_iipm←0としない
	 *　　（スタック上に保存されているSRからわざわざIPMを取り出す）のは、
	 *　　割込み発生前のタスクコンテキストでchg_ipmを実行し、
	 *　　割込み優先度マスク全解除状態でない状態で、割込み/CPU例外が
	 *　　発生し、ここに到達した場合に対応するためである。
	 */
	mov.l  @(SP_SR_OFFSET_INT,r15),r0
	mov.l  _saved_iipm_ret, r3
	and    #0xf0,r0		/* and #imm,r0命令はr0のみ指定可能  */
	mov.l  r1,@r2		/* lock_flag←true */
	mov.l  r0,@r3		/* saved_iimp←スタック上に積まれているSRのIPM  */

	/*
	 *	dspflgがfalseである場合と，p_runtskとp_schedtskが同じ場合には，
	 *	ディスパッチを行わない．このチェックが必要なのは，タスク例外処
	 *	理ルーチンの呼出しが必要な場合に，ディスパッチが必要なくても，
	 *	reqflgをtrueにするためである．
	 */
	mov.l  _dspflg_ret,r2
	mov.l  _p_runtsk_ret,r1
	mov.l  @r2,r3				/* r3 <- dspflg					*/
	mov.l  @r1,r0			 	/* r0 <- p_runtsk				*/

    /* R4: リターン後の状態がユーザモードかどうかを表すフラグ */
    mov     #0, r4
    /* rundomのチェック */
    mov.l   _rundom_ret_int, r1     /* r1 = &rundom */
    mov.l   @r1, r1                 /* r1 = rundom */
    tst     r1, r1                  /* rundom == TACP_KERNEL(=0) か？*/     
    bt      _ret_int_4              /* カーネルドメインならば_ret_int_4へ */

    /* ユーザタスクの場合，特権モードのチェック */
    mov.b   @(TCB_priv_mode,r0), r1 /* r1 = p_runtsk->priv_mode */
    tst     r1, r1                  /* priv_mode == 0 か？*/     
    bf      _ret_int_4              /* 特権モードならば_ret_int_4へ */
    nop

    /* ユーザスタックをシステムスタックに切り替え */
    mov.l   r15, @(TCB_usp,r0)
    mov.l   @(TCB_ssp,r0), r15
    mov     #2, r4
    mov.b   r4, @(TCB_priv_mode,r0) /* p_runtsk->priv_mode = true */

_ret_int_4:
    /* 
     *  R4: リターン後の状態がユーザモードかどうかを表すフラグ
     *  ここで、フラグをスタック上に保存するのは、割込みの出口処理や
     *  CPU例外の出口処理においてSSTKを使うとき、priv_modeを
     *  切り替えるためである
     *  もしpriv_modeを切り替えないと、SSTKを使っているときにCPU例外が
     *  発生し、その出口処理から発生元へ戻るとき、特権モードとして戻るか
     *  ユーザモードとして戻るかを判別できない
     *  ここで，priv_mode == 1は，タスク例外処理マスク状態を表し，
     *  ret_intはタスク例外処理マスク状態ではないため，priv_mode
     *  を2としている
     */
    mov.l   r4, @-r15

	mov.l  _iipm_cpu_lock_ret_int,r4
	ldc    r4,sr				  /* カーネル管理外の割込みを許可			  */

	tst    r3,r3
	bt	   _ret_int_r_1			/* dspflgがfalseならret_int_r_1へ */
	mov.l  _p_schedtsk_ret,r4	/* r5 <- schedtsk				 */
	mov.l  @r4,r5
	cmp/eq r0,r5				/* runtsk と schedtsk を比較	 */
	bt	   _ret_int_r_1 		/* 同じならret_int_r_1へ		 */

#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	/*
	 *	注意
	 *	　r8-r14に加えて、prが重複してスタックに保存されるが
	 *	　_ret_int_r側で読み出すときにも、整合させているので問題ない。
	 */
	movmu.l r8, @-r15			/*  r8-r14,prをスタックに保存  	*/
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  r14,@-r15			/* 残りのレジスタを保存 		 */
	mov.l  r13,@-r15
	mov.l  r12,@-r15
	mov.l  r11,@-r15
	mov.l  r10,@-r15
	mov.l  r9,@-r15
	mov.l  r8,@-r15
#endif	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  r15,@(TCB_ssp, r0)	/* タスクスタックをTCBに保存	  */
	mov.l  _ret_int_r_ret,r3
	bra    _dispatcher
	 mov.l	r3,@(TCB_pc, r0)	/* 実行再開番地をTCBに保存		  */


_ret_int_r:
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	/*
	 *	注意
	 *	　ここに分岐してくるのは_ret_int_3からのみである。
	 *	　_ret_int_3側でr8-r14に加えて、prもスタックに保存している。
	 */
	movmu.l @r15+,r8			/*  r8-r14,prをスタックから復帰  */
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r8
	mov.l  @r15+,r9
	mov.l  @r15+,r10
	mov.l  @r15+,r11
	mov.l  @r15+,r12
	mov.l  @r15+,r13
	mov.l  @r15+,r14
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
_ret_int_r_1:
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   r0, @-r15
    mov.l   _ovrtimer_start_int, r0
    jsr     @r0
    nop
    mov.l   @r15+, r0
#endif /* TOPPERS_SUPPORT_OVRHDR */
	/*
	 *	タスク例外処理ルーチンの起動
	 *	ret_int_r は dispatcher から呼び出されるため，tcbのアドレスはr0に
	 *	入っている．
	 *	なお，割込みの出口でタスクスイッチが発生しない場合はディスパッチャ
	 *	を通らずに ret_int から直接 ret_int_1 に飛ぶ
	 */

    mov.l   @r15+, r4   /* r4 = リターン後がユーザモードか？ */

	/*  enatexのチェック  */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	bld.b  #TCB_enatex_bit, @(TCB_enatex,r0)
    mov #0, r3  /* r3: タスク例外処理を呼び出すかどうかのフラグ */
	bf   _ret_int_r_check_rundom	/* enatexがfalseなら割込み発生元へリターン */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
 	/* SH2E以前は */
	/* 　　・bdl.b命令なし */
	/* 　　・mov.b @(dips12, rn), r0命令なし（dispが0xfまで） */
	/* 　　・tst #TCB_enatex_mask,r0はディスティネーションがr0固定 */
	mov	   #TCB_enatex, r1
	mov.b  @(r0, r1), r2
	/*	TCB_enatex_mask < 0x80なので, r3は符号拡張されない	*/
	mov    #TCB_enatex_mask,r3
	tst    r2, r3
    mov #0, r3  /* r3: タスク例外処理を呼び出すかどうかのフラグ */
	bf   _ret_int_r_check_rundom	/* enatexがfalseなら割込み発生元へリターン */
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	
	/* texptnが0で無く、ipmflgがtrueならば、タスク例外処理ルーチンの呼び出し */
	mov.l  @(TCB_texptn,r0),r1	/* texptnをロード */
	tst    r1,r1
    mov #0, r3  /* r3: タスク例外処理を呼び出すかどうかのフラグ */
	bt   _ret_int_r_check_rundom	/* enatexがfalseなら割込み発生元へリターン */
	
_ret_int_r_check_ipmflg:
	mov.l  _ipmflg_ret,r1		/* ipmflgをロード */
	mov.l  @r1,r2
	tst    r2,r2
    mov #0, r3  /* r3: タスク例外処理を呼び出すかどうかのフラグ */
	bt	   _ret_int_r_check_rundom /* ipmflgがfalseなら割込み発生元へリターン  */
    nop
    mov #1, r3

    /* rundomのチェック */
_ret_int_r_check_rundom:
    mov.l   _rundom_ret_int, r1     /* r1 = &rundom */
    mov.l   @r1, r1                 /* r1 = rundom */
    tst     r1, r1                  /* rundom == TACP_KERNEL(=0) か？*/     
    bf      _ret_int_r_usr     /* ユーザドメインならば_ret_int_r_return_0へ */
    nop
    tst     r3, r3  /* タスク例外処理を呼び出さないか？ */
    bt      _ret_int_r_return       /* リターン処理へ */
    mov.b   @(TCB_svclevel, r0), r1  /* r1 = svclevel */
    tst     r1, r1              /* svclevel == 0 ? */
    bf      _ret_int_r_return       /* リターン処理へ */
	
	mov.l  _call_texrtn_ret,r2
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	jsr/n  @r2					/* タスク例外処理ルーチンの呼び出し */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	jsr    @r2					/* タスク例外処理ルーチンの呼び出し */
	 nop
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

	/*  割込み発生元へリターン  */
_ret_int_r_return:
	mov.l  _lock_flag_ret, r2  /* lock_flagをfalseに */
	mov    #0x0, r0
	lds.l  @r15+,pr
	mov.l  r0,@r2
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rte
	 nop

    /* カーネルドメイン実行中でない場合，特権モードのチェック */
_ret_int_r_usr:
    tst     r4, r4                  /* r4 == 0 => リターン後が特権モードか？ */
    bt      _ret_int_r_return       /* 特権モードならば_ret_int_r_returnへ */
    nop

    tst     r3, r3  /* タスク例外処理を呼び出さないか？ */
    bt      _ret_int_r_return_0       /* リターン処理へ */
	
    /*
     *  ret_int_rはCPUロック状態で呼ばれる
     *  また割込み優先度マスク全解除状態で呼ばれる
     *  よって，CPUロック前の割込み優先度マスクはsaved_iipmである
     */
    mov.l   r10, @-r15
    mov.l   _saved_iipm_ret, r1
    mov.l   @r1, r10                    /* 割込み優先度マスクをR10に */
	mov.l  _prepare_texrtn_utask_ret,r2
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	jsr/n  @r2					/* タスク例外処理ルーチンの呼び出し */
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	jsr    @r2					/* タスク例外処理ルーチンの呼び出し */
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
    mov.l   _saved_iipm_ret, r1
    mov.l   r10, @r1                    /* 割込み優先度マスクを戻す */
    mov.l   @r15+, r10

_ret_int_r_return_0:
    /* システムスタックをユーザスタックに切り替え */
    mov.l   _p_runtsk_ret, r0
    mov.l   @r0, r0
    mov.l   r15, @(TCB_ssp,r0)
    mov.l   @(TCB_usp,r0), r15
    mov     #0, r2
    mov.b   r2, @(TCB_priv_mode,r0) /* p_runtsk->priv_mode = false */
	
    mov.l  _lock_flag_ret, r2  /* lock_flagをfalseに */
	mov    #0x0, r0
	mov.l  r0,@r2
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_ret_int, r0
    mov.l   _ret_int_r_retrun_1_ret_int, r2
	mov.l	_mpuvalid_ret_int, r1
    jmp     @r2
    nop

	.align 4
#ifdef TOPPERS_SUPPORT_OVRHDR
_ovrtimer_start_int:
    .long _ovrtimer_start
_ovrtimer_stop_int:
    .long _ovrtimer_stop
#endif /* TOPPERS_SUPPORT_OVRHDR */
    .extern _prepare_texrtn_utask
_iipm_cpu_lock_ret_int:				/* カーネル管理内割込み禁止用マスク */
	.long  IIPM_LOCK			/* ipm以外のビットはゼロで良い */
_rundom_ret_int:
    .long _rundom
_mpcmpen_ret_int:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_ret_int:
	.long 0xaaaa0001
_ret_int_r_retrun_1_ret_int:
    .long _ret_int_r_retrun_1
_prepare_texrtn_utask_ret:
    .long _prepare_texrtn_utask
_saved_iipm_ret:
	.long _saved_iipm
_reqflg_ret:
	.long _reqflg
_lock_flag_ret:
	.long  _lock_flag
_p_runtsk_ret:
	.long  _p_runtsk
_dspflg_ret:
	.long  _dspflg
_p_schedtsk_ret:
	.long  _p_schedtsk
_ret_int_r_ret:
	.long _ret_int_r
_ipmflg_ret:
	.long _ipmflg
_call_texrtn_ret:
	.long _call_texrtn_stask
_vector_entry_start_ret:
	.long __vector_entry_start
_vector_entry_end_ret:
	.long __vector_entry_end

    .section .text_shared
    .align 2
    .global _ret_int_r_retrun_1
_ret_int_r_retrun_1:
    /*
     *  R0: MPUENのアドレス
     *  R1: マジックナンバー
     */
	mov.l	r1, @r0

 	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rte
	nop
   
/*
 *  エミュレートされたCPU例外ハンドラの入口
 *  ここには、カーネルドメインからジャンプしてくるはずなので、
 *  MPUの処理は必要なく、カーネルドメインに配置できる
 *  CPU例外処理の共通部へジャンプするための準備をする
 *  r5: CPU例外ハンドラ番号
 *  r6: CPU例外ハンドラの先頭アドレス
 */
    .section .text_kernel
    .align 2
    .global _emulate_exception_entry
_emulate_exception_entry:
    /*  必要最小限のレジスタをスタックに待避  */ 		
    /* 
     *  共通部の処理を流用するため、例外が発生した時と同様にSRと戻り番地をスタックに積む
     *  この処理にはサブルーチンコールでくるため、戻り番地はPRに保存されている
     */
    stc.l   sr, @-r15
    sts.l   pr, @-r15
    mov.l   r7, @-r15
    mov.l   r6, @-r15
    stc     sr, r7        /* SRの保存 */
    /*	残りのスクラッチレジスタをスタックに積む  */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l r5, @-r15
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l   r5, @-r15											
	mov.l   r4, @-r15											
	mov.l   r3, @-r15
	mov.l   r2, @-r15
	mov.l   r1, @-r15
	mov.l   r0, @-r15
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	sts.l   mach, @-r15
	sts.l   macl, @-r15
    sts.l   pr, @-r15
    mov.l   _iipm_disall_emulate_exc, r0
	ldc     r0, sr        /* 全割込み禁止 */
    mov.l   r5, @-r15    /* CPU例外ハンドラ番号を保存 */
    bra     _exception_entry_0
    nop

    .align 4
    .extern __kernel_emulate_cpuexc_handler
_iipm_disall_emulate_exc: /* 割込み禁止用マスク */
    /* ipm以外のビットはゼロで良い */
    .long  __kernel_iipm_disall
 

/*
 *	共通CPU例外入口処理
 *
 *  CPU例外ハンドラは，非タスクコンテキストで実行する．そのため，CPU例
 *  外ハンドラを呼び出す前に非タスクコンテキストに切替え，リターンして
 *  きた後に元のコンテキストに戻す．元のコンテキストに戻すために，ス
 *  タック切替えする前のタスクスタックのスタックポインタを非タスクコン
 *  テキスト用スタック上に保存する．
 *  CPU例外がタスクコンテキストで発生し，reqflg が true になった時に，
 *  _ret_exc_2 へ分岐する．
 *	reqflg をチェックする前に割込みを禁止しないと，reqflg をチェック後
 *	に起動された割込みハンドラ内でディスパッチが要求された場合に，ディ
 *	スパッチされない．
 *	　
 *	一般不当命令の場合は戻り番地を2バイト進める必要があるが
 *	対応していない
 *	　
 *	CPU例外要因毎に展開されるルーチンでr4,r5,r6,r7を保存し、
 *　 全割込み禁止状態
 *	 r5 ： CPU例外ハンドラ番号
 *	 r6 ： CPU例外ハンドラの先頭アドレス
 *	 r7 ： CPU例外発生時のSRのコピー
 *	の状態でここに来る。
 *	
 * (1) CPU例外発生時のコンテキスト用スタック
 *		  ----------------------
 *	0	 | CPU例外ハンドラ番号	|	<-- p_excinf（CPU例外ハンドラの引数）、
 *		  ----------------------		_ret_exc_non_taskの時点（CPU例外発生時、非タスクコンテキストの場合）
 *	1	 |		   PR			|	<-- _ret_exc_1/_ret_exc_2の時点
 *		  ----------------------
 *	2	 |		  MACH			|
 *		  ----------------------
 *	3	 |		  MACL			|
 *		  ----------------------
 *	4	 |		   R0			|
 *		  ----------------------
 *	5	 |		   R1			|
 *		  ----------------------
 *	6	 |		   R2			|
 *		  ----------------------
 *	7	 |		   R3			|
 *		  ----------------------
 *	8	 |		   R4			|  <-- sp（_exception_entryの時点）
 *		  ----------------------
 *	9	 |		   R5			|
 *		  ----------------------
 * 10	 |		   R6			|
 *		  ----------------------
 * 11	 |		   R7			|
 *		  ----------------------
 * 12	 |		   PC			|	<-- sp（CPU例外受付直後の時点）
 *		  ----------------------
 * 13	 |		   SR			|
 *		  ----------------------
 *	
 * (2) コンテキスト切り替え後の非タスクコンテキスト用スタック
 *		  ----------------------
 *		 |	 CPU例外ハンドラの	|（log_exc_enterを呼び出す場合）
 *		 |	 先頭アドレス(r6)	|
 *		  ----------------------
 *		 |	   p_excinf(r4) 	|（log_exc_enterを呼び出す場合）
 *		  ----------------------
 *		 | CPU例外ハンドラ番号	|（log_exc_leaveを呼び出す場合）
 *		  ----------------------
 *		 |	   saved_iipm		|  <-- sp（_ret_excの時点）
 *		  ----------------------
 *		 |	   lock_flag		|
 *		  ----------------------
 *		 | タスクスタックのsp	|（スタック切替をした場合）
 *		  ----------------------
 */

/*
 *	_ret_exc_2の時点で、スタックポインタの値から
 *	「スタックに積まれているCPU例外からの戻り番地」までのオフセット
 */
#define    SP_PC_OFFSET_EXC 	(11 * 4)

	.section .text_shared
	.global _exception_entry
_exception_entry:
			/*	残りのスクラッチレジスタをスタックに積む  */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l r3,@-r15
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  r3,@-r15
	mov.l  r2,@-r15
	mov.l  r1,@-r15
	mov.l  r0,@-r15
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	sts.l  mach,@-r15
	sts.l  macl,@-r15
	sts.l  pr,@-r15
	mov.l  r5,@-r15 			 /* CPU例外ハンドラ番号を保存  */
    /* 
     *  MPU無効
     */
	mov.l	_mpcmpen_exc, r1
	mov.l	_mpuinvalid_exc, r0
	mov.l	r0, @r1
    mov.l   _exception_entry_0_exc, r0
    jmp     @r0
    nop

    .align 4
_exception_entry_0_exc:
    .long _exception_entry_0
_mpuinvalid_exc:
	.long 0xaaaa0000
_mpcmpen_exc:
	.long MPCMPEN   /* MPU有効 */

    .section .text_kernel
    .align 2
_exception_entry_0:
    /*
     *    現在、割込みハンドラの出入口で、excnest_countが0のときに、
     *    priv_mode = falseの区間があり、その区間でCPU例外が発生したとき、
     *    priv_modeとMPU ON/OFF状態の矛盾が生じる場合がある
     */
	mov.l  _excnest_exc, r0 	 /* 割込み/CPU例外ネストカウンタのチェック */
	mov.l  @r0,r1
	mov    r15, r4				 /* CPU例外ハンドラへの引数 */
	tst    r1,r1				 /* 割込み発生時のコンテキストを判定	 */
	add    #0x1,r1				 /* 割込み/CPU例外ネストカウンタをインクリメント */
							/* CPU例外発生時に非タスクコンテキストならジャンプ */
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	bf/s   _exception_entry_1
	 mov.l	r1,@r0
#else	/*	TOPPERS_SUPPORT_SH2E_INSTRUCTION  */
	mov.l  r1,@r0
	bf	   _exception_entry_1
#endif	/* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   r7, @-r15
    mov.l   r6, @-r15
    mov.l   r5, @-r15
    mov.l   _ovrtimer_stop_exc, r0
    jsr     @r0
    nop
    mov.l   @r15+, r5
    mov.l   @r15+, r6
    mov.l   @r15+, r7
#endif /* TOPPERS_SUPPORT_OVRHDR */
	mov.l  _kernel_istkpt_exc,r3 /* 非タスクスタックに切り替え			 */
	mov.l  @r3,r15
	mov.l  r4,@-r15 			 /* タスクスタックポインタの保存 */
_exception_entry_1:
	/*
	 * 元のCPUロック状態／解除状態をスタックに保存
	 */
	mov.l  _lock_flag_exc,r1
	mov.l  _saved_iipm_exc,r2
	mov.l  @r1, r1
	mov.l  @r2, r2
	mov.l  r1,@-r15
	mov.l  r2,@-r15

#ifdef LOG_EXC_LEAVE
	mov.l  r5,@-r15 			 /* CPU例外ハンドラ番号を保存 */
#endif /* LOG_EXC_LEAVE */

#ifdef LOG_EXC_ENTER
	ldc    r7,sr				 /* 割込み許可	*/
	mov.l _log_exc_enter_exc, r1 /* ログ出力関数  */
	mov.l  r4,@-r15 			 /* r4保存（CPU例外ハンドラの引数） */
	mov    r5, r4				 /* 引数（CPU例外ハンドラ番号）*/
	jsr   @r1
	 mov.l  r6,@-r15 			 /* r6保存（CPU例外ハンドラの先頭アドレス）*/

	mov.l @r15+, r6 			 /* r6復帰		  */
	jsr    @r6					 /* CPU例外ハンドラ呼び出し  */
	 mov.l @r15+, r4			 /* r4復帰		  */
#else /* LOG_EXC_ENTER */

#ifdef TOPPERS_LDC_IS_SLOT_ILLEGAL_INSTRUCTION
	ldc    r7,sr				 /* 割込み許可	*/
	jsr    @r6					 /* CPU例外ハンドラ呼び出し  */
	 nop
#else	/* TOPPERS_LDC_IS_SLOT_ILLEGAL_INSTRUCTION */
	jsr    @r6					 /* CPU例外ハンドラ呼び出し  */
	 ldc	r7,sr				 /* 割込み許可	*/
#endif	/* TOPPERS_LDC_IS_SLOT_ILLEGAL_INSTRUCTION */

#endif /* LOG_EXC_ENTER */

#ifdef LOG_EXC_LEAVE
	mov.l _log_exc_leave_exc, r1 /* ログ出力関数						*/
	jsr  @r1
	 mov.l @r15+, r4             /* 引数：CPU例外ハンドラ番号           */
#endif /* LOG_EXC_LEAVE */

	/*
	 * CPU例外ハンドラ出口処理
	 */
_ret_exc:
	/*
	 * 要検討
	 * 　excnestの更新とスタック切り替えを不可分に行うには、
	 * 　カーネル管理外の割込みも禁止する必要がある。
	 * 　　→カーネル管理外の割込みの有無で処理が変わる。
	 */
	mov.l  _iipm_disall_exc,r0    /* 全割込み禁止			 */
	ldc    r0,sr
	/*
	 * CPUロック状態／解除状態を元の状態に戻す
	 */
	mov.l  @r15+,r2 		   /* saved_iipm を元に戻す */
	mov.l  _saved_iipm_exc,r3
	mov.l  @r15+,r4 		   /* lock_flag を元に戻す	*/
	mov.l  _lock_flag_exc,r5
	mov.l  r2,@r3
	mov.l  r4,@r5
	/*
	 * コンテキスト切り替え
	 * 　「スタックに積まれたCPU例外ハンドラ番号を読み飛ばす」処理（ス
	 * 　タックポインタのインクリメント）はスタック切り替え後でないと
	 * 　できないため、戻り先のコンテキスト判定後、2箇所で行っている。
	 * 　（スタック切り替え直後と_ret_exc_non_task）
	 */
	/*
	 *	要検討
	 *	　ここで発生したカーネル管理外の割込みで破壊しないように退避
	 */
	/*	　要検討：ここから、不可分	*/
	mov    r15,r2
	/*
	 *　割込み/CPU例外ネストカウンタをデクリメントし、
	 *　戻り先が非タスクコンテキストならすぐにリターン
	 */
	mov.l  _excnest_exc,r0
	mov.l  @r0,r1

#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	dt	   r1
	bf/s   _ret_exc_non_task
	 mov.l r1,@r0
#else /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	add    #0xff,r1
	tst    r1,r1
	mov.l  r1,@r0
	bf	   _ret_exc_non_task
#endif /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */

	mov.l  @r2,r15			/* 戻り先がタスクなのでスタックを戻す	  */
	/*	要検討：ここまで、不可分  */
	
	mov.l  _reqflg_exc,r4	/* reqflgのチェック */
	mov.l  @r4,r1
	add    #4, r15
		/* スタックに積まれたCPU例外ハンドラ番号を読み飛ばす */
	
	/*
	 *　reqflgがtrueならret_exc_2へ
	 *　　ret_exc_2に分岐してディスパッチしない場合があるのでreqflgは
	 *　　クリアしない
	 */
	tst    r1,r1
	bf	  _ret_exc_2


	/*
	 *　ディスパッチャを呼ばずにリターンする場合
	 */
_ret_exc_0:
#ifdef TOPPERS_SUPPORT_OVRHDR
    mov.l   _ovrtimer_start_exc, r0
    jsr     @r0
    nop
#endif /* TOPPERS_SUPPORT_OVRHDR */
    mov.l   _rundom_ret_exc, r0     /* r0 = &rundom */
    mov.l   @r0, r0                 /* r0 = rundom */
    cmp/eq  #0, r0        /* rundom == TACP_KERNEL か？*/     
    bt      _ret_exc_1            /* カーネルドメインならば_ret_exc_1へ */
    mov.l   _p_runtsk_ret_exc, r0       /* r0 = &p_runtsk */
    mov.l   @r0, r0                 /* r0 = p_runtsk */
    mov.b   @(TCB_priv_mode,r0), r0 /* r0 = p_runtsk->priv_mode */
    cmp/eq  #0x00, r0               /* ユーザモード（priv_mode == 0） か？ */
    bf      _ret_exc_1            /* 特権モードならば_ret_exc_1へ */
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_ret_exc, r0
    mov.l   _ret_exc_1_usr_ret_exc, r2
	mov.l	_mpuvalid_ret_exc, r1
    jmp     @r2
    nop

	.align 2
#ifdef TOPPERS_SUPPORT_OVRHDR
_ovrtimer_start_exc:
    .long _ovrtimer_start
_ovrtimer_stop_exc:
    .long _ovrtimer_stop
#endif /* TOPPERS_SUPPORT_OVRHDR */
_ret_exc_1_usr_ret_exc:
    .long _ret_exc_1_usr
_rundom_ret_exc:
    .long _rundom
_p_runtsk_ret_exc:
    .long _p_runtsk
_mpcmpen_ret_exc:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_ret_exc:
	.long 0xaaaa0001

    .align 2
_ret_exc_1:
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rte
	 nop

    .section .text_shared
    .align 2
    .global _ret_exc_1_usr
_ret_exc_1_usr:
    /*
     *  R0: MPUENのアドレス
     *  R1: マジックナンバー
     */
	mov.l	r1, @r0
    
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rte
	 nop

    .section .text_kernel
    .align 2
	/*
	 *　戻り先が非タスクコンテキスト
	 *　　スタックに積まれているCPU例外ハンドラ番号を読み飛ばし、
	 *　　_ret_exc_1へ分岐
	 */
_ret_exc_non_task:
	bra    _ret_exc_1
	 add   #4, r15
		/* スタックに積まれたCPU例外ハンドラ番号を読み飛ばす */

	/*
	 *　戻り先がタスクコンテキストで、reqflg=trueの場合
	 *　（ディスパッチャまたはタスク例外処理ルーチンの呼び出し）
	 *　
	 *　割込みの場合（_ret_int_2）と共通にできないのは、
	 *　レジスタバンクがあり、レジスタ退避のタイミングが異なるため。
	 */
_ret_exc_2:
	/*
	 *　戻り先が割込み／CPU例外の入口処理か否かの判別
	 *	詳しくはret_int_2を参照のこと
	 */
	mov.l  @(SP_PC_OFFSET_EXC,r15),r0	/* r0 <-スタックに積まれたPC */
	mov.l  _vector_entry_start_exc, r1
	mov.l  _vector_entry_end_exc, r2
	cmp/hs r1, r0
	bf	   _ret_exc_2_1			/*  ディスパッチャを呼び出す場合  */
	cmp/hi r0, r2
	bt	   _ret_exc_0			/*  ディスパッチャを呼ばずにリターン  */

_ret_exc_2_1:
	bra    _ret_int_3			/*  割込みの場合と共通の処理  */
	 nop
	
	.align	2
_excnest_exc:					/* 割込み/CPU例外ネストカウンタ */
	.long  _excnest_count
_kernel_istkpt_exc:
	.long __kernel_istkpt
_lock_flag_exc:
	.long _lock_flag
_saved_iipm_exc:
	.long _saved_iipm
_iipm_disall_exc:
	.long IIPM_DISALL
_reqflg_exc:
	.long  _reqflg
_vector_entry_start_exc:
	.long __vector_entry_start
_vector_entry_end_exc:
	.long __vector_entry_end
#ifdef LOG_EXC_ENTER
_log_exc_enter_exc:
	.long _log_exc_enter
#endif /* LOG_EXC_ENTER */
#ifdef LOG_EXC_LEAVE
_log_exc_leave_exc:
	.long _log_exc_leave
#endif /* LOG_EXC_LEAVE */

/*
 *	カーネル管理外の割込みの入口処理
 *　　割込み要因に依らず、共通な部分
 *　　（コンフィギュレータが生成する入口処理（kernel_cfg_asm.S）からの
 *　　　続き）
 *	
 *	割込み要因毎に展開されるルーチンでr4,r5,r6,r7を保存し、
 *　　全割込み禁止状態
 *　　r5：割込み番号
 *　　r6：割込みハンドラの先頭アドレス
 *　　r7：割込み受付直後のSRのコピー
 *	の状態でここに来る。
 *	（r4に割り当てないのは、CPU例外ハンドラの処理となるべく共通にするため）
 */
	.section .text_shared
	.global _non_kernel_int_entry
_non_kernel_int_entry:
#ifndef  USE_BANKED_REG
					/*	残りのスクラッチレジスタをスタックに積む  */
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l r3, @-r15
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  r3,@-r15
	mov.l  r2,@-r15
	mov.l  r1,@-r15
	mov.l  r0,@-r15
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	sts.l  mach,@-r15
	sts.l  macl,@-r15
	sts.l  pr,@-r15
#endif /* USE_BANKED_REG */
    /* 
     *  MPU無効
     */
	mov.l	_mpcmpen_non_ker_int, r1
	mov.l	_mpuinvalid_non_ker_int, r0
	mov.l	r0, @r1
    mov.l   _non_kernel_int_entry_0_non_ker_int, r0
    jmp     @r0
    nop

    .align 4
_non_kernel_int_entry_0_non_ker_int:
    .long _non_kernel_int_entry_0
_mpuinvalid_non_ker_int:
	.long 0xaaaa0000
_mpcmpen_non_ker_int:
	.long MPCMPEN   /* MPU有効 */

    .section .text_kernel
    .align 4
_non_kernel_int_entry_0:
	mov.l  _excnest_non_kernel_int, r0    /* 割込み/CPU例外ネストカウンタのチェック */
	mov.l  @r0,r1
	tst    r1,r1				 /* 割込み発生時のコンテキストを判定  */
	add    #0x1,r1				 /* 割込みネストカウンタをインクリメント */
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	bf/s   _non_kernel_int_entry_1 		 /* 多重割込みならジャンプ	*/
	 mov.l	r1,@r0
#else	/*	TOPPERS_SUPPORT_SH2E_INSTRUCTION  */
	mov.l  r1,@r0
	bf	   _non_kernel_int_entry_1 		 /* 多重割込みならジャンプ	*/
#endif	/* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	mov.l  _kernel_istkpt_non_kernel_int,r3 /* スタック切り替え　*/
	mov    r15,r2
	mov.l  @r3,r15
	mov.l  r2,@-r15 			 /* タスクスタックの保存 */
_non_kernel_int_entry_1:
	ldc    r7,sr				 /* 割込み許可	*/
	/*
	 * 割込み番号を保存
	 * 　ここで保存した割込み番号には、２つの意味がある。
	 * 　　・default_int_handlerの引数
	 * 　　・log_inh_leave_intの引数
	 */
	mov.l  r5,@-r15

#ifdef LOG_INH_ENTER
	mov.l _log_inh_enter_non_kernel_int, r1 /* ログ出力関数 */
	mov.l  r6,@-r15 	  /* r6（割込みハンドラの先頭アドレス）保存 */
	jsr   @r1
	 mov  r5, r4				 /* 引数		  */
	mov.l @r15+, r6 			 /* r6復帰		  */
#endif /* LOG_INH_ENTER */

	jsr    @r6					 /* 割込みハンドラ呼び出し  */
#ifndef LOG_INH_LEAVE
	  mov.l @r15+,r4			 /* default_int_handler用の引数 */
	  							 /*  （割込み番号）  */
#else  /* LOG_INH_LEAVE */
	  mov.l @r15, r4			 /* default_int_handler用の引数 */
	  							 /*  （スタックはそのまま）  　 */
	mov.l _log_inh_leave_int, r1 /* ログ出力関数				*/
	jsr  @r1
	 mov.l @r15+,r4 			 /* 割込み番号を復帰			*/
#endif /* LOG_INH_LEAVE */

	/*
	 * カーネル管理外の割込みハンドラ出口処理
	 *
	 *	割込み/CPU例外ネストカウンタをデクリメントし，戻り先のコンテキ
	 *	ストを判定してスタックを入れ替える．
	 *	カーネル管理外の割込みをサポートする場合は，両者の間にカーネル
	 *	管理外の割込みが入る場合がある．カーネル管理外の割込みの入口は，
	 *	カーネル管理の割込みと同様であるため，割込み/CPU例外ネストカウ
	 *	ンタを参照して，スタックを入れ替える．そのため，割込み/CPU例外
	 *	ネストカウンタとスタックの入れ替えはカーネル管理外の割込みに対
	 *	しても，不可分に行う必要がある．
	 */
_ret_non_kernel_int:
	mov.l  _iipm_disall_non_kernel_int,r2  /*  全割込み（カーネル管理外も含め）禁止  */
	ldc    r2,sr
	/*
	 *　割込み/CPU例外ネストカウンタをデクリメントし、
	 *　戻り先が非タスクコンテキストならすぐにリターン
	 */
	mov.l  _excnest_non_kernel_int,r0
	mov.l  @r0,r1
#ifdef TOPPERS_SUPPORT_SH2E_INSTRUCTION
	dt	   r1
	bf/s   _ret_non_kernel_int_1
	 mov.l r1,@r0
#else /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	add    #0xff, r1
	tst    r1,r1
	mov.l  r1,@r0
	bf	   _ret_non_kernel_int_1
#endif /* TOPPERS_SUPPORT_SH2E_INSTRUCTION */
	
	mov.l  @r15,r15 		/* 戻り先がタスクなのでスタックを戻す	  */

	/*
	 *	割込み処理からのリターン
	 */
_ret_non_kernel_int_0:
    mov.l   _rundom_ret_non_ker_int, r0     /* r0 = &rundom */
    mov.l   @r0, r0                 /* r0 = rundom */
    cmp/eq  #0, r0        /* rundom == TACP_KERNEL か？*/     
    bt      _ret_non_kernel_int_1            /* カーネルドメインならば_ret_non_kernel_int_1へ */
    mov.l   _p_runtsk_ret_non_ker_int, r0       /* r0 = &p_runtsk */
    mov.l   @r0, r0                 /* r0 = p_runtsk */
    mov.b   @(TCB_priv_mode,r0), r0 /* r0 = p_runtsk->priv_mode */
    cmp/eq  #0x00, r0               /* ユーザモード（priv_mode == 0） か？ */
    bf      _ret_non_kernel_int_1            /* 特権モードならば_ret_non_kernel_int_1へ */
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_ret_non_ker_int, r0
    mov.l   _ret_non_kernel_int_1_usr_ret_non_ker_int, r2
	mov.l	_mpuvalid_ret_non_ker_int, r1
    jmp     @r2
    nop

	.align 2
_ret_non_kernel_int_1_usr_ret_non_ker_int:
    .long _ret_non_kernel_int_1_usr
_rundom_ret_non_ker_int:
    .long _rundom
_p_runtsk_ret_non_ker_int:
    .long _p_runtsk
_mpcmpen_ret_non_ker_int:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_ret_non_ker_int:
	.long 0xaaaa0001

    .align 2
_ret_non_kernel_int_1:
#ifdef USE_BANKED_REG
	resbank
#else	/*	USE_BANKED_REG	*/
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
#endif	/* USE_BANKED_REG */
	rte
	 nop

	.align 2
_kernel_istkpt_non_kernel_int:
	.long __kernel_istkpt
_excnest_non_kernel_int:			/* 割込み／CPU例外ネストカウンタ */
	.long  _excnest_count
_iipm_disall_non_kernel_int:		/* 割込み全禁止用マスク */
	.long  IIPM_DISALL
#ifdef LOG_INH_ENTER
_log_inh_enter_non_kernel_int:
	.long  _log_inh_enter
#endif /* LOG_INH_ENTER */
#ifdef LOG_INH_LEAVE
_log_inh_leave_non_kernel_int:
	.long  _log_inh_leave
#endif /* LOG_INH_LEAVE */

    .section .text_shared
    .align 2
    .global _ret_non_kernel_int_1_usr
_ret_non_kernel_int_1_usr:
    /*
     *  R0: MPUENのアドレス
     *  R1: マジックナンバー
     */
	mov.l	r1, @r0
    
#ifdef USE_BANKED_REG
	resbank
#else	/*	USE_BANKED_REG	*/
	lds.l  @r15+,pr
	lds.l  @r15+,macl
	lds.l  @r15+,mach
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	movml.l @r15+,r7
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */
	mov.l  @r15+,r0
	mov.l  @r15+,r1
	mov.l  @r15+,r2
	mov.l  @r15+,r3
	mov.l  @r15+,r4
	mov.l  @r15+,r5
	mov.l  @r15+,r6
	mov.l  @r15+,r7
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
#endif	/* USE_BANKED_REG */
	rte
	 nop


/*
 *	未登録の割込みハンドラの入口処理
 *	　割込み要因毎の入口処理から、以下の状態でここに来る。
 *　　　・r7,r6を保存
 *　　　・r6:割込み番号
 *　　　※ 全割込み禁止状態ではない点に注意
 *　　
 *　　以下のレジスタ割当で、_int_entryへ分岐する。
 *　　
 *　　レジスタ割当
 *　　　　r7：割込み受付直後のSRのコピー
 *　　　　r6：割込みハンドラの先頭アドレス(_default_int_handler)
 *　　　　r5：割込み番号
 *　　　　r4：_interrupt_entry
 *　　　　　（割込みハンドラが登録されている場合と共通の処理）
 */
	.section .vector_entry,"xa"
	.align 2
	.global  _default_int_handler_entry
_default_int_handler_entry:
#ifndef USE_BANKED_REG
	mov.l r5,@-r15				   /*  必要最小限のレジスタをスタックに待避  */
	mov.l r4,@-r15
#endif /* USE_BANKED_REG */
	mov   r6,r5						/* 割込み番号 */
	mov.l _iipm_disall_default_int_handler,r6
	stc   sr,r7						/* SRの保存 */
	ldc   r6,sr						/* 割込み禁止 */
									/* 共有割込み入口処理 */
	mov.l _common_routine_default_int_handler,r4
									/* 割込みハンドラの先頭アドレス */
	mov.l _c_routine_default_int_handler,r6
	/*	（PC相対アドレス形式なので、遅延スロットには入れられない。）  */
	jmp   @r4
	 nop

	.align 2
_iipm_disall_default_int_handler:
	.long IIPM_DISALL
_c_routine_default_int_handler:
	.long _default_int_handler
_common_routine_default_int_handler:
	.long  _interrupt_entry

/*
 *	未登録のCPU例外ハンドラの入口処理
 *	　CPU例外要因毎の入口処理から、以下の状態でここに来る。
 *　　　・r7,r6を保存
 *　　　・r6:ベクタ番号
 *　　　※ 全割込み禁止状態ではない点に注意
 *　　
 *　　以下のレジスタ割当で、_exception_entryへ分岐する。
 *　　
 *　　レジスタ割当
 *　　　　r7：CPU例外発生時のSRのコピー
 *　　　　r6：CPU例外ハンドラの先頭アドレス(_default_exc_handler)
 *　　　　r5：ベクタ番号
 *　　　　r4：_exception_entry
 *　　　　　（CPU例外ハンドラが登録されている場合と共通の処理）
 */
	.section .vector_entry,"xa"
	.align 2
	.global  _default_exc_handler_entry
_default_exc_handler_entry:
		 /*  必要最小限のレジスタをスタックに待避  */
	mov.l r5,@-r15
	mov.l r4,@-r15
	mov   r6,r5							/* ベクタ番号をコピー	*/
	mov.l _iipm_disall_default_exc_handler,r6
	stc   sr,r7							/* SRの保存   */
	ldc	  r6,sr							/* 全割込み禁止 */
										/* 共有CPU例外入口処理 */ 
	mov.l _common_routine_default_exc_handler,r4
									/* CPU例外ハンドラの先頭アドレス */
	mov.l _c_routine_default_exc_handler,r6
	/*	（PC相対アドレス形式なので、遅延スロットには入れられない。）  */
	jmp	  @r4
	 nop
		/*
		 *	ldc命令を遅延スロットに配置しないのは、
		 *	空いているレジスタがないため
		 */

	.align 2
_iipm_disall_default_exc_handler:			/* 割込み禁止用マスク */
	.long  IIPM_DISALL
_common_routine_default_exc_handler:		/* CPU例外共通の入口処理 */
	.long  _exception_entry
_c_routine_default_exc_handler:				/* CPU例外ハンドラの先頭アドレス */
	.long  _default_exc_handler


/*
 *	微少時間待ち
 */
	.text
	.align 2
	.global _sil_dly_nse
_sil_dly_nse:
	mov.l  _sil_dly_tim1, r1
	add    r1, r4
	cmp/pl r4
	bt	   _sil_dly_nse1
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
_sil_dly_nse1:
	mov.l  _sil_dly_tim2, r1
	add    r1, r4
	cmp/pl r4
	bt	   _sil_dly_nse1
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	rts/n
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	rts
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

	.align 2
_sil_dly_tim1:
	.long  -SIL_DLY_TIM1
_sil_dly_tim2:
	.long  -SIL_DLY_TIM2

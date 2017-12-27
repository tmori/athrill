/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
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
 *  $Id: start.S 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		カーネル対応のスタートアップモジュール（V850用）
 */

#define OMIT_INCLUDE_OS_CFG		/* Os_Cfg.hインクルード抑止 */

#include <v850asm.inc>

	.section .text_kernel
	.align  4
	.global __start
    .extern _bsssecinib_table
    .extern _tnum_bsssec
    .extern _datasecinib_table
    .extern _tnum_datasec
    .extern _syscall_table
FLABEL(__start)
	di                                  /* 割り込み禁止 */

	/*
	 * 各種ポインタの初期化(SP/TP/EP/GP/CTBP)
	 */
	Lea     __ostkpt, r3
	ld.w	0[r3],r3
    /* =begin modified for SC3 */
    /*
     * ep, ctbpはとりあえず使用しない
     * tp, gpはsdataアクセスでコンパイラが使用する
     */
	Lea		__tp, tp 
	/* Lea		__ep , ep */
	Lea 	__gp , gp
	/* Lea		__ctbp , r6 */
	/* ldsr	r6 , 20 */
    /*
     *  syscallの動作設定
     */
#ifdef __v850e2v3__
    mov     SYSCALL_SIZE, r6
    ldsr    r6, sccfg
    Lea     _syscall_table, r6
    ldsr    r6, scbp
#elif defined(__v850e3v5__)
    mov     SYSCALL_SIZE, r6
    ldsr    r6, 11, 1
    Lea     _syscall_table, r6
    ldsr    r6, 12, 1
    syncp
#endif /* __v850e2v3__ */

    /* =end modified for SC3 */

#ifdef TOPPERS_USE_HFLOAT
	/* 
	 *  FPUを有効に
	 */
	stsr psw, r12       /* load psw */
	mov  0x00010000,r13 /* PSW.CU0(PSW[16]) = 1 */
	or   r13, r12
	ldsr r12, psw       /* store psw */
#endif /* TOPPERS_USE_HFLOAT */
 	
	/*
	 *  hardware_init_hook の呼出し（0 でない場合）
	 *
	 *  ターゲットハードウェアに依存して必要な初期化処理がある場合
	 *  は，hardware_init_hook という関数を用意する．
	 */
	Lea		_hardware_init_hook, r1
	cmp		r0, r1
	be		hardware_init_hook_end
	jarl	_hardware_init_hook, lp
FLABEL(hardware_init_hook_end)
	
	/*
	 *  BSSセクション初期化
     *  r0: ゼロレジスタ
	 */
    mov     r0, r6                  /* r6: テーブルのインデックス i */
    Lea     _tnum_bsssec, r7
    ld.w    0[r7], r7               /* r7: テーブルの要素数 num */
    Lea     _bsssecinib_table, r8   /* r8: テーブルのアドレス */
FLABEL(bss_clear_start)
    cmp     r7, r6                  /* ループ終了条件: i >= num */
    bnl     bss_clear_end
    ld.w    0[r8], ep               /* ep: bssの先頭番地 */
    ld.w    4[r8], r9               /* r9: bssの終了番地 */
	cmp     r9, ep                  /* bssが空か？ */
	bnl 	bss_clear_start_hook    /* 空ならば次のbssへ */
    nop
FLABEL(bss_clear_body)
    /*
     *  bssの領域は，4byte境界にアラインされている
     *  よって，4byteずつデータをコピーしている
     */
	sst.w   r0, 0[ep]           /* bssの領域を0で初期化 */
	add     4, ep              /* 次の番地へ */
	cmp     r9, ep              /* bssの終了番地まで来たか？ */
	bl      bss_clear_body      /* 終了番地でなければループ */
#ifdef BSSSEC_ALIGN_MASK
    /*
     *  境界になるまで初期化を続ける
     */
    mov     BSSSEC_ALIGN_MASK, r10
FLABEL(bss_clear_body2)
    tst     r10, ep         /* ep & 0xf == 0 ? */
    bz      bss_clear_start_hook
	sst.w	r0, 0[ep]       /* 0初期化 */
    add     4, ep
    jr      bss_clear_body2
#endif /* BSSSEC_ALIGN_MASK */
    nop
FLABEL(bss_clear_start_hook)
    add     8, r8 /* テーブルのアドレスをインクリメント 4byte * 2 -> 8 */
    add     1, r6 /* テーブルのインデックスをインクリメント: i++ */
    br      bss_clear_start
    nop
FLABEL(bss_clear_end)

#ifndef OMIT_DATA_INIT
	/*
	 *  dataセクションの初期化（ROM化対応）
	 *
	 */
    mov     r0, r6                  /* r6: テーブルのインデックス i */
    Lea     _tnum_datasec, r7
    ld.w    0[r7], r7               /* r7: テーブルの要素数 num */
    Lea     _datasecinib_table, r8  /* r8: テーブルのアドレス */
FLABEL(data_init_start)
    cmp     r7, r6                  /* ループ終了条件: i >= num */
    bnl     data_init_end
    ld.w    0[r8], ep               /* ep : dataの先頭番地 */
    ld.w    4[r8], r9               /* r9 : dataの終了番地 */
    ld.w    8[r8], r2               /* r2: idataの終了番地 */
	cmp     r9, ep                  /* dataが空か？ */
	bnl 	data_init_start_hook    /* 空ならば次のdataへ */
    nop
FLABEL(data_init_body)
	ld.b	0[r2], r10      /* idataの値をロード */
	sst.b	r10, 0[ep]      /* idataの値をdataのアドレスへストア */
	add		1, ep           /* dataの次の番地へ */
	add		1, r2           /* idataの次の番地へ */
    cmp     r9, ep          /* dataの終了番地まで来たか？ */
    bl      data_init_body  /* 終了番地でなければループ */
#ifdef DATASEC_ALIGN_MASK
    /*
     *  境界になるまで初期化を続ける
     */
    mov     DATASEC_ALIGN_MASK, r10
FLABEL(data_init_body2)
    tst     r10, ep         /* ep & 0xf == 0 ? */
    bz      data_init_start_hook
	sst.b	r0, 0[ep]       /* 0初期化 */
    add     1, ep
    jr      data_init_body2
#endif /* DATASEC_ALIGN_MASK */
    nop
FLABEL(data_init_start_hook)        
    add     12, r8  /* テーブルのアドレスをインクリメント 4byte * 3 -> 12 */
    add     1, r6   /* テーブルのインデックスをインクリメント: i++ */
	br		data_init_start         
    nop
FLABEL(data_init_end)
#endif /* OMIT_DATA_INIT */

	/*
	 *  software_init_hook を呼出し（0 でない場合）
	 *
	 *  ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *  理がある場合は，software_init_hook という関数を用意すれば
	 *  よい．
	 */

	Lea		_software_init_hook, r1
	cmp		r0, r1
	be		software_init_hook_end
	jarl	_software_init_hook, r31
FLABEL(software_init_hook_end)

	/*
	 *  カーネルを起動する．
	 */
	jarl    _target_hardware_initialize, r31
	jarl    _main, r31
	halt

	.end

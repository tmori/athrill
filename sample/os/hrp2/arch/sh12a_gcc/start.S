/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *									Toyohashi Univ. of Technology, JAPAN
 *	Copyright (C) 2007		by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2008-2010 by Industrial Technology Institute,
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
 *	$Id: start.S 2083 2011-05-12 07:59:25Z mit-kimai $
 */

/*
 *	カーネル対応のスタートアップモジュール(SH12A用)
 */

#define TOPPERS_MACRO_ONLY
#define UINT_C(val) 	(val)		 /* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		 /* ulong_t型の定数を作るマクロ */
#define CAST(type, val) (val)		 /* 型キャストを行うマクロ */
#include "kernel_impl.h"

	.section .text_kernel
	.global _start
	.align	2
_start:
	/*
	 *	SRを初期化する。
	 *	（全割込みロックとする。）
	 */
	mov.l  _iipm_disall_start, r0
	ldc    r0, sr

	/*
	 *	非タスクコンテキスト用のスタックの初期化
	 */
	mov.l  _kernel_istkpt_p, r1
	mov.l  @r1, r15

	/*
	 *	_hardware_init_hook を呼び出す。(0でない場合)
	 *
	 *	ターゲットハードウェアに依存して必要な初期化処理がある場合は，
	 *	hardware_init_hookという関数を用意する．
	 */
start_0:
	mov.l  _hardware_init_hook_start, r0
	tst    r0, r0
	bt	   start_1
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	jsr/n  @r0
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	jsr    @r0
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	
	/*
	 * bssセクションをクリア
	 *
	 *	各セクションは4バイト境界に配置されているので、
	 *	_endが4バイト境界でないとしても、
	 *	隣のセクションを上書きすることはない。
	 *	（下記のdataセクションの初期化も同様）
	 */
start_1:
    mov.l   __kernel_bsssecinib_table_sta, r0 /* テーブルのアドレス */
    mov   #0, r1 /* テーブルのインデックス: i */
    mov.l   __kernel_tnum_bsssec_sta, r2 
    mov.l   @r2, r2 /* テーブルの要素数: num */
	mov   #0, r3 /* 初期化する値 */
start_bss_2:
    cmp/hi  r1, r2 /* ループ終了判定: i < num */
    bf      start_3
    mov.l   @(0,r0), r4 /* bssの先頭番地 */
    mov.l   @(4,r0), r5 /* bssの終了番地 */
	cmp/hs  r5, r4      /* bssが空か？ */
	bt	    start_bss_4 /* 空ならば次のbssへ */
    nop
start_bss_3:
    /*
     *  bssの領域は，4byte境界にアラインされている
     *  よって，4byteずつデータをコピーしている
     */
	mov.l   r3, @r4     /* bssの領域を0で初期化 */
	add     #4, r4      /* 次の番地へ */
	cmp/hi  r4, r5      /* bssの終了番地まで来たか？ */
	bt      start_bss_3 /* 終了番地ならば終了 */
    nop
start_bss_4:
    add     #8, r0 /* テーブルのアドレスをインクリメント 4byte * 2 -> 8 */
    add     #1, r1 /* テーブルのインデックスをインクリメント: i++ */
    bra     start_bss_2
    nop

    .align 4
    .global __kernel_bsssecinib_table
    .global __kernel_tnum_bsssec
__kernel_bsssecinib_table_sta:
    .long   __kernel_bsssecinib_table
__kernel_tnum_bsssec_sta:
    .long   __kernel_tnum_bsssec

	/*
	 *	dataセクションを初期化する（ROM化対応）．
	 *
	 *	__idata_start から __idata_end までを，__data_start 以降に
	 *	コピーする．
	 */
start_3:
#ifndef TOPPERS_OMIT_DATA_INIT
    mov.l   __kernel_datasecinib_table_sta, r0 /* テーブルのアドレス */
    mov     #0, r1 /* テーブルのインデックス: i */
    mov.l   __kernel_tnum_datasec_sta, r2
    mov.l   @r2, r2 /* テーブルの要素数: num */
start_data_2:
    cmp/hi  r1, r2 /* ループ終了判定: i < num */
    bf      start_5
    mov.l   @(0,r0), r3 /* dataの先頭番地 -> 初期化する変数アドレス */
    mov.l   @(4,r0), r4 /* dataの終了番地 */
    mov.l   @(8,r0), r5 /* idataの先頭番地 -> 初期化する定数値のアドレス */
	cmp/hs  r4, r3      /* dataが空か？ */
	bt	    start_data_4    /* 空ならば次のdataへ */
    nop
start_data_3:
    /*
     *  idataの領域は，ROM領域の節約のため，アラインされない
     *  よって，1byteずつデータをコピーしている
     */
	mov.b   @r5, r6         /* idataの値をロード */
    mov.b   r6, @r3         /* idataの値をdataのアドレスへストア */
	add     #1, r3          /* dataの次の番地へ */
	add     #1, r5          /* idataの次の番地へ */
	cmp/hi  r3, r4          /* dataの終了番地まで来たか？ */
	bt      start_data_3    /* 終了番地ならば終了 */
    nop
start_data_4:
    add     #12, r0 /* テーブルのアドレスをインクリメント 4byte * 3 -> 12 */
    add     #1, r1 /* テーブルのインデックスをインクリメント: i++ */
    bra     start_data_2
    nop

    .align 4
    .global __kernel_datasecinib_table
    .global __kernel_tnum_datasec
__kernel_datasecinib_table_sta:
    .long   __kernel_datasecinib_table
__kernel_tnum_datasec_sta:
    .long   __kernel_tnum_datasec
#endif /* TOPPERS_OMIT_DATA_INIT */

	/*
	 *	_software_init_hook を呼び出す（0 でない場合）．
	 *
	 *	ソフトウェア環境（特にライブラリ）に依存して必要な初期化処
	 *	理がある場合は，software_init_hook という関数を用意すれば
	 *	よい．
	 */
start_5:
	mov.l  _software_init_hook_start, r0
	tst    r0, r0
	bt	   start_6
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	jsr/n  @r0
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	jsr    @r0
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

start_6:
    /*
     *  キャッシュの初期化（SH2A固有。ただし、sh7211を除く。）
     *  　
     *  　キャッシュの初期化はキャッシュ無効空間で行う必要があるため、
     *  　関数呼び出しの際に、分岐先アドレスのビット29をセットしている。
     *  　（0x2000,0000とor演算しているのは、最初から_cache_initが
     *  　　キャッシュ無効空間にマッピングされている場合の対策）
     *  　
     *  　アドレスマップ（外部メモリのみ抜粋）：
     *  　　0x0000,0000 - 0x1fff,ffff　キャッシュ有効空間
     *  　　0x2000,0000 - 0x3fff,ffff　キャッシュ無効空間（シャドー空間）
     */
#ifdef ENABLE_CACHE
	mov.l  _cache_init_p, r1
	mov.l  _non_cachable_addr_top, r2
	or     r2, r1
	jsr/n  @r1			/*  SH2A固有の処理なので、jsr/n命令が使用できる  */
#endif /* ENABLE_CACHE */

	/*
	 *	カーネルを起動する
	 */
	mov.l  _sta_ker_start, r0
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION
	jsr/n  @r0
#else /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */
	jsr    @r0
	 nop
#endif /* TOPPERS_SUPPORT_SH2A_INSTRUCTION */

    /*
     *  キャッシュの初期化（SH2A固有。ただし、sh7211を除く。）
     */
#ifdef ENABLE_CACHE
_cache_init:
    mov.l _ccr1_addr, r1
								/*  キャッシュ・フラッシュ  */
    movi20 #(CCR1_ICF | CCR1_OCF), r0
    mov.l r2, @r1
    mov.l _ccr_init_val, r2		/*  値はターゲット依存  */
    mov.l r2, @r1
    mov.l @r1, r2				/*  ダミーリード  */
    rts/n						/*  （念のため、遅延スロットには配置しない）*/
#endif /* ENABLE_CACHE */

	.align 2
_iipm_disall_start:
	.long IIPM_DISALL
_kernel_istkpt_p:
	.long __kernel_istkpt
_hardware_init_hook_start:
	.long _hardware_init_hook
_software_init_hook_start:
	.long _software_init_hook
_sta_ker_start:
	.long _sta_ker

#ifdef ENABLE_CACHE
_cache_init_p:
	.long _cache_init
_non_cachable_addr_top:
	.long 0x20000000
_ccr1_addr:
	.long CCR1
_ccr_init_val:
	.long CCR1_INIT			/*  値はターゲット依存  */
#endif /* ENABLE_CACHE */


/*
 *  HEWのI/Oシミュレーション機能の呼び出し
 *  
 *  本来は、sys_support.Sに記述すべきだが、
 *  リンク後の_hew_io_sim_bodyのアドレスを
 *  アプリケーションによらず、一定にするため、
 *  スタートアップルーチンの直後に配置している。
 *  
 *  
 *  　以下の状態で呼ばれる。
 *  　　r4：機能コード
 *  　　r5：パラメータブロックへのポインタ
 */
#ifdef TOPPERS_HEW_SIMULATOR

    .section .text
    .align 2        
    .global _hew_io_sim
_hew_io_sim:
	mov   r4, r0			/*  機能コード  */
	mov   r5, r1			/*  パラメータブロックへのポインタ  */
	mov.l _hew_io_sim_body_addr, r3
    sts.l pr,@-r15
	jsr   @r3
	nop
    lds.l @r15+,pr
    rts
     nop

/*
 *  このアドレスに分岐することで、HEWのI/Oシミュレーション機能が
 *  作動するトリガとなる。
 *  
 */
_hew_io_sim_body:
    rts
     nop
	
	.align  4
_hew_io_sim_body_addr:
    .long  _hew_io_sim_body

#endif	/*  TOPPERS_HEW_SIMULATOR  */


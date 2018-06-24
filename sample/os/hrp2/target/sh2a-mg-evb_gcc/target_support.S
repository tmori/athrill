/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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
 */

/*
 *	ターゲット依存モジュール アセンブリ言語部（SH72AW）
 */

#define  TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t型の定数を作るマクロ */
#define ULONG_C(val)	(val)		/* ulong_t型の定数を作るマクロ */
#define CAST(type, val) (val)		 /* 型キャストを行うマクロ */

/*
 *	アセンブラ用のマクロを有効にする
 */
#define TOPPERS_ASM_MACRO
#include "kernel_impl.h"
#include "offset.h"


/*
 *	低レベルのターゲット依存の初期化
 *
 *	スタートアップモジュールの中で，メモリの初期化の前に呼び出される．
 */

/*
 *	ROM化時に，DATAセクションのサイズが0だと，BSSがTXTの直下に配置され
 *	るため，DATAセクションのサイズを0にしないためのダミー変数を定義．
 *	コンパイラの最適化により削除されるのを防ぐため、アセンブラで記述し
 *	ている。
 */
	.data
_dummy:
	.byte	0x00

	.text
	.align 2
	.global _hardware_init_hook
_hardware_init_hook:
    /*
     *  プロテクトを解除
     */
    mov.l    _spr0, r0
    mov     #SPR0_CANCEL_PROTECT, r1
    mov.b   r1, @r0

    /*
     *  メインクロック発進停止検出割込みを許可
     */
    mov.l    _mcmcr, r0
    mov     #MCMCR_LXIE, r1
    mov.b   r1, @r0

    /*
     *  FPUの停止
     */
    mov.l    _mstpcr, r0
    mov.w   _mstpcr_fpustp, r1
    mov.w   r1, @r0

    /*
     *  システムクロックの分周をなしに設定
     */
    mov.l    _scdr, r0
    mov     #SCDR_SYSD_DIV0, r1
    mov.b   r1, @r0

    /*
     *  バスクロックの分周を2分周に設定
     */
    mov.l    _bcdr, r0
    mov     #BCDR_BUSD_DIV2, r1
    mov.b   r1, @r0

    /*
     *  周辺機能クロックの設定
     *  クロックA：4分周
     *  クロックB：2分周
     */
    mov.l    _pbcdr, r0
    mov.l   _pbcdr_pbad_div4, r1
    mov.l   _pbcdr_pbbd_div2, r2
    or      r2, r1
    mov.l   r1, @r0

    /*
     *  システムクロックを一時的にメインクロックへ
     */
    mov.l    _scsr, r0
    mov     #SCSR_SCS_XIN, r1
    mov.b   r1, @r0

    /*
     *  PLLクロック周波数を160MHzに設定
     *  設定後は安定までに1msecかかる
     */
    mov.l    _plcr, r0
    mov.l   @r0, r0
    mov.w   _plcr_vcod, r1
    not     r1, r1
    mov.w   r1, @r0

    /*
     *  1msec待つ
     *  ここでは、20MHzのクロックでCPUが動いているはずなので、
     *  20M * 1m = 20000回 nopを呼べばよいはず
     */
    mov.w   _wait_pll_ready_count, r0
_wait_pll_ready:
    nop
    dt      r0          /* r0 - 1 -> r0; r0 == 0 ? */
    bf      _wait_pll_ready /* if r0 != 0 then jump to _wait_pll_ready */
    nop     /* 遅延分岐スロット */

    /*
     *  システムクロックをPLLへ
     */
    mov.l    _scsr, r0
    mov     #SCSR_SCS_PLL, r1
    mov.b   r1, @r0

    /*
     *  プロテクト
     */
    mov.l    _spr0, r0
    mov     #SPR0_PROTECT, r1
    mov.b   r1, @r0

    /*
     *  startに戻る
     */
    rts
    nop

/*
 *  クロック関連のレジスタ
 *  hardware_init_hookで使用するデータを近くに置く
 */
 /*
  *  long data
  */
 .align 4
_spr0:
    .long SPR0 /* システムプロテクトレジスタ */
_mcmcr:
    .long MCMCR /* メインクロック監視機能制御レジスタ */
_plcr:
    .long PLCR /* PLL制御レジスタ */
_scsr:
    .long SCSR /* システムクロック選択レジスタ */
_scdr:
    .long SCDR /* システムクロック分周レジスタ */
_bcdr:
    .long BCDR /* バスクロック */
_pbcdr:
    .long PBCDR /* 周辺機能クロック分周レジスタ */
_rdccr:
    .long RDCCR /* RDCクロック制御レジスタ：f(PBA)の分周 */
_cocr:
    .long COCR /* クロック出力機能制御レジスタ：f(BUS)の分周 */
_mstpcr:
    .long MSTPCR /* モジュールストップ制御レジスタ */
_pbcdr_pbad_div2:
    .long PBCDR_PBAD_DIV2 /* クロックA：2分周 */
_pbcdr_pbad_div4:
    .long PBCDR_PBAD_DIV4 /* クロックA：4分周 */
_pbcdr_pbbd_div0:
    .long PBCDR_PBBD_DIV0 /* クロックB：分周なし */
_pbcdr_pbbd_div2:
    .long PBCDR_PBBD_DIV2 /* クロックB：2分周 */
/*
 *  word data
 */
_plcr_vcod:
    .word PLCR_VCOD /* PLLの出力：120MHz（このビットを0にすると、160MHz）*/
_mstpcr_fpustp:
    .word MSTPCR_FPUSTP /* FPUを停止（再び動作させるためにはリセットする）*/
_wait_pll_ready_count:
    .word 0x4e20

    /*
     *  拡張サービスコールの出入り口処理
     *  第1〜第4パラメータ：R4〜R7
     *  第5パラメータ：R2 → システムスタックの一番下に積む
     *  機能コード：R1
     *  p_runtsk：R8
     *  呼び出し元を特定するためのビットマップ：R9
     *    0x01: excnest_count == 0 && rundom != TACP_KERNEL
     *    0x02: excnest_count == 0
     *  SVC前のIMASK：R10
     *  スタックにpr, r8，r9，r10を退避
     *  MPU無効状態
     *  特権モード実行中 or 非タスクコンテキスト
     *    priv_mode == 1 or nestcount > 0
     *  システムスタック or 非タスクコンテキスト用スタックを使用中
     *  機能コードの下限チェックは実行済み
     */
    .section .text_kernel
    .global _extsvc_entry
    .extern	__kernel_tmax_fncd
    .extern	__kernel_extsvc_table
    .extern __kernel_call_texrtn_stask

#define SIZEOF_SVCINIB 8
#define SVCINIB_svcrtn 0
#define SVCINIB_stksz  4

    .align 4
extsvc_error_RSFN:
	mov.l	_e_rsfn_svc, r0     /* エラーコードを返す */
	mov.l   r0, @-r15       /* 返り値をスタックに保存 */
    bra     svc_entry_r
    nop

extsvc_error_NOMEM:
	mov.l   _e_nomem_svc, r0    /* エラーコードを返す */
    add     #(4*2), r15         /* スタックを捨てる */
	mov.l   r0, @-r15       /* 返り値をスタックに保存 */
    bra     svc_entry_r
    nop
extsvc_error_SYS:
	mov.l   _e_sys_svc, r0      /* エラーコードを返す */
    add     #(4*2), r15         /* スタックを捨てる */
	mov.l   r0, @-r15       /* 返り値をスタックに保存 */
    bra     svc_entry_r
    nop

_extsvc_entry:
extsvc_entry_1:
    /*
     *  機能コードのチェック
     */
    mov.l   _tmax_fncd_extsvc, r0
    mov.l   @r0, r0
    cmp/gt  r0, r1                  /* 機能コードの上限チェック(tmax_fncd < r1) */
    bt      extsvc_error_RSFN       /* 上限をオーバしているならばエラー */

    /*
     *  拡張サービスコールのテーブルを引く
     *  R3 = &svcinib
     */
    mov.l   _svcinib_table_extsvc, r3
    mov     #SIZEOF_SVCINIB, r0
    add     #-1, r1                     /* 機能コード -= 1 */
    mulr    r0, r1                      /* R1 = 機能コード * sizeof(SVCINIB) */
    add     r1, r3                      /* R3 += R1(テーブルの先頭番地に対象要素へのオフセットを加える) */

    /*
     *  拡張サービスコール本体のアドレスがNULLでないか？
     */
    mov.l   @(SVCINIB_svcrtn,r3), r0    /* R0 = svcinib->svcrtn */
    cmp/eq  #0, r0                      /* R0 == NULL? */
    bt      extsvc_error_RSFN           /* NULLならばエラー */
    mov     r0, r1                      /* R1 = svcinib->svcrtn */

    add     #-4, r15                /* domid用の領域を空ける */
	mov.l   r2, @-r15               /* 第5引数をスタックに積む */
    /*
     *  呼び出し元が非タスクコンテキストか？
     */
    mov     #0x02, r0
    tst     r0, r9          /* excnest_count != 0か？ */
	bt      extsvc_entry_3	/* 非タスクコンテキストならばスキップ */

extsvc_entry_2:
    /*
     *  スタックの残量をチェック
     */
    mov.l   @(SVCINIB_stksz,r3), r0     /* R0 = svcinib->stksz */
    mov     r15, r3                     /* R3 = ssp */
    cmp/hi  r3, r0                      /* r3 < r0 ならば */
    bt      extsvc_error_NOMEM          /* エラー処理へ */
    sub     r0, r3                      /* ssp -= svcinib->stksz：R3 = 最小ssp */
    mov.l   @(TCB_p_tinib,r8), r2
    mov.l   @(TINIB_sstk_bottom,r2), r0 /* R0 = スタックの終了番地 */
    cmp/hi  r0, r3                      /* r0 < r3 ならば */
    bt      extsvc_error_NOMEM          /* エラー処理へ */
    sub     r3, r0                      /* R0 = スタックの終了番地 - 最小ssp：スタックの底からの距離 */
    mov.l   @(TINIB_sstksz,r2), r3      /* R3 = スタックサイズ */
    cmp/hi  r3, r0                      /* r0 > r3 ならば */
    bt      extsvc_error_NOMEM          /* エラー処理へ */

    /*
     *  svclevelをインクリメント
     */
    mov.b   @(TCB_svclevel,r8), r0  /* svclevelを読み込む */
    mov     #255, r3
    cmp/hs  r3, r0                  /* svclevelの上限チェック(r3 <= r0:unsigned) */
	bt      extsvc_error_SYS        /* ネストの上限だった場合エラー */
	add     #1, r0                  /* svclevel += 1 */
    mov.b   r0, @(TCB_svclevel,r8)  /* svclevelを更新 */


    /*
     *  実行ドメインをカーネルドメインへ
     */
    mov     #0x01, r0
    tst     r0, r9          /* rundom != TACP_KERNEL か？ */
	bt      extsvc_entry_3	/* カーネルドメインならばスキップ */
    mov.l   _rundom_svc, r3     /* R3 = &rundom */
    mov     #0, r0
	mov.l	r0, @r3             /* rundom = TACP_KERNEL */

    /*
     *  domidを求める
     */
    mov.l   @(TINIB_p_dominib,r2), r0   /* R0 = p_dominib */
    mov.l   _dominib_table_extsvc, r3   /* R3 = dominib_table */
    sub     r3, r0                      /* R0 = &dominib_table[x] - &dominib_table[0] */
    /*
     *  dominibのサイズは40byte
     */
    /*
     *  R0 = 40x / 8 => 5x
     *  右に3bitシフト
     */
    shar    r0
    shar    r0
    shar    r0
    /*
     *  5x / 5 => x
     */
    mov.l   _33333333_extsvc, r3    /* R3 = 0x33333333 */
    mulr    r0, r3                  /* R3 = 5x * 0x33333333 = 0xffffffff * x = -x */
    neg     r3, r0
    add     #TMIN_DOMID, r0

    bra     extsvc_entry_4
    nop

extsvc_entry_3:
	mov     #TDOM_KERNEL, r0    /* 呼び出し元はTDOM_KERNEL */
extsvc_entry_4:
	mov.l   r0, @(4, r15)       /* 呼び出し元のdomidをスタックに積む */

    /*
     *  ipmを復帰
     */
    stc     sr, r2
    mov     #0xf0, r0
    not     r0, r0
    and     r0, r2      /* imaskをクリア */
    or      r10, r2     /* imaskを復帰 */
    ldc     r2, sr

    /*
     *  拡張サービスコールの本体を呼ぶ
     */
	jsr     @r1                     /* サービスコール処理ルーチンを呼ぶ */
    nop
	mov.l   @r15+, r2               /* 第5引数をスタックから取り除く */
	mov.l   @r15+, r2               /* 呼び出し元のdomidをスタックから取り除く */

extsvc_entry_r:
    /*
     *  サブルーチンコールでここにくるため，
     *  スタック上のSRを書き換えるなどの処理は必要ない
     */
	mov.l   r0, @-r15       /* 返り値をスタックに保存 */

    /*
     *  SRにセットされているipmをr10に保存
     *  - chg_ipmやloc_cpuでIMASKを更新した時への対応
     *  全割込み禁止  
     */
    stc     sr, r0
    and     #0xf0, r0   /* IMASKのみを残す */
    mov     r0, r10     /* IMASKを保存 */
    stc     sr, r0
    or      #0xf0, r0   /* IMASKをセット */
    ldc     r0, sr

    /*
     *  呼び出し元が非タスクコンテキストか？
     */
    mov     #0x02, r0
    tst     r0, r9      /* excnest_count != 0か？ */
	bt      extsvc_prv_r   /* 非タスクコンテキストならばリターン */

    /*
     *  svclevelをデクリメント
     */
    mov.b   @(TCB_svclevel,r8), r0  /* svclevelを読み込む */
    dt      r0                      /* svclevel -= 1 && svclevelが0か？ */
    mov.b   r0, @(TCB_svclevel,r8)  /* svclevelを更新 */
    bf      extsvc_prv_r               /* 0でなければリターン */
    
    /*
     *  実行ドメインを所属ドメインに戻す
     */
   	mov.l	@(TCB_p_tinib,r8), r0       /* タスク初期化ブロックの読み出し */
	mov.l	@(TINIB_p_dominib,r0), r0   /* ドメイン初期化ブロックの読み出し */
	mov.l	@(DOMINIB_domptn,r0), r0    /* 所属ドメインの取得 */
    mov.l   _rundom_svc, r1             /* R1 = &rundom */
    mov.l   r0, @r1                     /* rundom = 所属ドメイン */
    /*
     *  サービスコール実行中に拡張サービスコールに入ることはないため、
     *  リターン先が非タスクコンテキストでないならば、リターン後は
     *  タスク例外処理保留状態ではない
     */

    /*
     *  待ち禁止状態を解除する
     */
    mov     #TCB_waifbd_mask, r0
    not     r0, r0
    mov.b   @(TCB_waifbd,r8), r1
    and     r1, r0
    mov.b   r0, @(TCB_waifbd,r8)

    /*
	 *  タスク例外処理実行条件がそろっているかチェック
	 */
	mov.l	@(TCB_texptn,r8), r0    /* 保留例外要因の読出し */
    cmp/eq  #0, r0                  /* 保留例外要因が0か？ */
	bt      extsvc_r                /* 0ならリターン */
	mov.b	@(TCB_enatex,r8), r0    /* タスク例外処理許可状態の読出し */
    and     #TCB_enatex_mask, r0
    cmp/eq  #0, r0                  /* タスク例外処理禁止状態か？ */
	bt      extsvc_r                /* 禁止ならリターン */

	/*
     *  割込みマスク全解除状態か？ 
     */
    mov.l   _ipmflg_svc, r0
    mov.l   @r0, r0
    cmp/eq  #0, r0
    bt      extsvc_r

	/*
     *  サービスコールからのリターン後のシステム状態がCPUロック解除状態か？
     */
    mov.l   _lock_flag_svc, r0
    mov.l   @r0, r0
    cmp/eq  #0, r0
    bf      extsvc_r             /* CPUロック解除状態でなければリターン */
    nop

    /*
     *  戻り先がユーザドメインかチェック
     */
    mov     #0x01, r0
    tst     r0, r9          /* 呼び出し元のドメイン == TACP_KERNELか？ */
	bf      extsvc_usr_r_texrtn
    nop
extsvc_prv_r_texrtn:
    /*
     *  CPUロック
     */
    mov.l   _lock_flag_svc, r0
    mov     #1, r1
    mov.l   r1, @r0

    mov.l   _saved_iipm_svc, r0
    mov     #IIPM_ENAALL, r1
    mov.l   r1, @r0

    /*
     *  SRのIMASKをIIPM_LOCK  
     *  ここには割込み優先度マスク全解除状態で来るため，
     *  IMASKは0のはずなので，クリアしない
     */
    mov.l   _iipm_lock_svc, r0
    stc     sr, r1
    or      r0, r1      /* imask=IIPM_LOCK */
    ldc     r1, sr

    mov.l   _call_texrtn_stask_extsvc, r0
    jsr     @r0  /* システムタスク用のタスク例外処理入り口へ */
    nop

    /*
     *  SRのIMASKを全割込み禁止  
     */
    stc     sr, r1
    mov     #0xf0, r0
    or      r0, r1      /* imask=0xf0 */
    ldc     r1, sr

    /*
     *  リターン後にCPUロック解除となるよう準備する
     */
    mov.l   _lock_flag_svc, r0
    mov     #0, r1
    mov.l   r1, @r0
extsvc_prv_r:
    bra     svc_prv_r       /* TACP_KERNELの場合 */
    nop
extsvc_usr_r_texrtn:
    /*
     *  r0 = p_runtsk
     */
    mov     r8, r0
	bsr     _prepare_texrtn_utask    /* ユーザタスク用のタスク例外処理入り口へ */
    nop
    bra     svc_usr_r       /* TACP_KERNELでない場合 */
    nop

extsvc_r:
    /*
     *  戻り先がユーザドメインかチェック
     */
    mov     #0x01, r0
    tst     r0, r9          /* 呼び出し元のドメイン == TACP_KERNELか？ */
    bt      svc_prv_r       /* TACP_KERNELの場合 */
    nop
    bra     svc_usr_r       /* TACP_KERNELでない場合 */
    nop

    .align 4
_tmax_fncd_extsvc:
    .long   _tmax_fncd
_svcinib_table_extsvc:
    .long   _svcinib_table
_e_nomem_svc:
    .long   E_NOMEM
_e_sys_svc:
    .long   E_SYS
_call_texrtn_stask_extsvc:
    .long   _call_texrtn_stask
_dominib_table_extsvc:
    .long _dominib_table
_33333333_extsvc:
    .long 0x33333333
_saved_iipm_svc:
    .long _saved_iipm

    /*
     *  サービスコールの出入り口処理
     *  第1〜第4パラメータ：R4〜R7
     *  第5パラメータ：R2 → システムスタックの一番下に積む
     *  機能コード：R1
     *  レジスタの使用方針
     *    p_runtsk：R8
     *    呼び出し元を特定するためのビットマップ：R9
     *      0x01: excnest_count == 0 && rundom != TACP_KERNEL
     *      0x02: excnest_count == 0
     *    SVC前のIMASK：R10
     *    スタックにpr, r8，r9，r10を退避
     */
    .section .text_shared
    .align 4
    .global __kernel_svc_entry
__kernel_svc_entry:
    sts.l   pr, @-r15   /* PRを退避 */
    mov.l   r8, @-r15   /* r8を退避 */
    mov.l   r9, @-r15   /* r9を退避 */
    mov.l   r10, @-r15  /* r10を退避 */

    /*
     *  SRにセットされているipmを保存
     *  全割込み禁止
     */
    stc     sr, r0
    and     #0xf0, r0   /* IMASKのみ残す */
    mov     r0, r10     /* IMASKを保存 */
    stc     sr, r0
    or      #0xf0, r0   /* IMASKを全割込み禁止にセット */
    ldc     r0, sr

    /* 
     *  MPU無効
     */
	mov.l	_mpcmpen_svc_0, r9
	mov.l	_mpuinvalid_svc, r0
	mov.l	r0, @r9
    mov.l   svc_entry_0_svc, r0
    jmp     @r0
    nop

    .align 4
svc_entry_0_svc:
    .long svc_entry_0
_mpuinvalid_svc:
	.long 0xaaaa0000
_mpcmpen_svc_0:
	.long MPCMPEN   /* MPU有効 */

    .section .text_kernel
	.extern	_svc_table
    .align 4
svc_entry_0:
    mov.l   __kernel_p_runtsk_svc, r8   /* 実行中のタスク */
    mov.l   @r8, r8   /* 実行中のタスク */
    /* 
     *  R9: 呼び出し元を特定するためのビットマップ
     *    0x01: excnest_count == 0 && rundom != TACP_KERNEL
     *    0x02: excnest_count == 0
     */
    mov     #0, r9 /* 0初期化 */

    /*
	 *  タスクコンテキストからの呼び出しかチェック
	 */
    mov.l   _excnest_count_svc, r0
    mov.l   @r0, r0
	cmp/eq  #0,	r0                      /*  多重割込みでないか？ */
	bf      svc_entry_1                 /*  多重割込みならばスキップ */
    mov     #0x02, r0
    or      r0, r9          /* excnest_count == 0のビットをセット */

    /*
     *  ユーザドメインからの呼び出しかチェック
     *  拡張サービスコール実行中であれば，
     *  カーネルドメインからの呼び出しとなるので，
     *  svclevelのチェックは必要ない
     */
    mov.l   _rundom_svc, r0
    mov.l   @r0, r0
    cmp/eq  #0, r0          /* rundom == 0(TACP_KERNEL)か？ */
    bt      svc_entry_1     /* 0ならばスキップ */
    mov     #0x01, r0
    or      r0, r9          /* rundom != TACP_KERNELのビットをセット */

    /*
     *  タスクの状態を特権モード実行中に移行する
     *  実行ドメインがカーネルドメインであり，
     *  戻り先がタスクコンテキストならば，タスク実行中であるはず
     *  なので，p_runtskがNULLかどうかのチェックは必要ないはず
     */
    mov     #1, r0
    mov.b	r0,	@(TCB_priv_mode,r8)     /* 特権モード実行中へ */

    /*
     *  ユーザスタックの保存
     *  システムスタックへの切り替え
     */
    mov.l   r15, @(TCB_usp,r8)              /* uspの保存 */
    mov.l   @(TCB_ssp,r8), r15    /* sspの初期化 */

svc_entry_1:
    /*
     *  機能コードのチェック
     */
    mov.l   _tmin_fncd_svc, r0
	cmp/ge  r0, r1       	/* 機能コードの下限のチェック(TMIN_FNCD <= r1) */
	bf      svc_entry_7     /* 下限よりも下ならばエラー */
    mov     #0, r0   
	cmp/gt  r0, r1 			/* 機能コードの符号チェック(0 < r1) */
    bf      svc_entry_2     /* サービスコールの場合 */
    nop
	bra     _extsvc_entry   /* 拡張サービスコールの場合 */
    nop

svc_entry_2:
    /*
     *  ipmを復帰
     */
    stc     sr, r2
    mov     #0xf0, r0
    not     r0, r0
    and     r0, r2      /* imaskをクリア */
    or      r10, r2     /* imaskを復帰 */
    ldc     r2, sr

	mov.l   r2, @-r15       /* 第5引数をスタックに積む */

    /*
     *  サービスコールのテーブルを引き，
     *  サービスコールの本体を呼ぶ
     */
	neg     r1, r1          /* 符号反転 */
    shal    r1
    shal    r1              /* 2bit左シフト(4倍:アドレス長が4byte) */
    mov.l   _svc_table_svc, r0
	mov.l   @(r0,r1), r1    /* テーブルを引く */
	jsr     @r1             /* サービスコール処理ルーチンを呼ぶ */
    nop
	mov.l   @r15+, r2       /* 第5引数をスタックから取り除く */

	mov.l   r0, @-r15       /* 返り値をスタックに保存 */
    /*
     *  SRにセットされているipmを保存
     *  - chg_ipmやloc_cpuでIMASKを更新した時への対応
     *  全割込み禁止  
     */
    stc     sr, r0
    and     #0xf0, r0   /* IMASKのみを残す */
    mov     r0, r10     /* IMASKを保存 */
    stc     sr, r0
    or      #0xf0, r0   /* IMASKをセット */
    ldc     r0, sr

svc_entry_r:
    /*
     *  ここには全割込み禁止状態でジャンプしてくる
     */
    /*
     *  サブルーチンコールでここにくるため，
     *  スタック上のSRを書き換えるなどの処理は必要ない
     */

    /*
     *  戻り先がユーザドメインかチェック
     *  拡張サービスコール実行中であれば，
     *  戻り先はカーネルドメインなので，
     *  svclevelのチェックは必要ない
     */
    mov     #0x01, r0
    tst     r0, r9          /* rundom == TACP_KERNELか？ */
    bt      svc_prv_r       /* TACP_KERNELならばスキップ */

    /*
	 *  戻り先がタスクコンテキストかチェック
	 */
    mov     #0x02, r0
    tst     r0, r9          /* excnest_count != 0か？ */
    bt      svc_prv_r       /* 0でなければスキップ */

	/*
	 *  待ち禁止を解除
	 */
    mov     #TCB_waifbd_mask, r0
    not     r0, r0
    mov.b   @(TCB_waifbd,r8), r1
    and     r1, r0
    mov.b   r0, @(TCB_waifbd,r8)
	
    /*
	 *  タスク例外処理実行条件がそろっているかチェック
	 */
	mov.l	@(TCB_texptn,r8), r0    /* 保留例外要因の読出し */
    cmp/eq  #0, r0                  /* 保留例外要因が0か？ */
	bt      svc_usr_r               /* 0ならリターン */
	mov.b	@(TCB_enatex,r8), r0    /* タスク例外処理許可状態の読出し */
    and     #TCB_enatex_mask, r0
    cmp/eq  #0, r0                  /* タスク例外処理禁止状態か？ */
	bt      svc_usr_r               /* 禁止ならリターン */

	/*
     *  割込みマスク全解除状態か？ 
     */
    mov.l   _ipmflg_svc, r0
    mov.l   @r0, r0
    cmp/eq  #0, r0
    bt      svc_usr_r               /* ipmflg == falseならリターン */

	/*
     *  サービスコールからのリターン後のシステム状態がCPUロック解除状態か？
     */
    mov.l   _lock_flag_svc, r0
    mov.l   @r0, r0
    cmp/eq  #0, r0
    bf      svc_usr_r             /* CPUロック解除状態でなければリターン */
    nop

    /*
     *  タスク例外呼び出し条件が揃っていれば，
     *  ユーザタスクのタスク例外処理へ
     *  r0 = p_runtsk
     */
    mov     r8, r0
    bsr      _prepare_texrtn_utask
    nop

svc_usr_r:
    mov.l   @r15+, r0   /* 返り値をスタックから復帰 */

    /*
     *  ユーザスタックへの切り替え
     */
    mov.l   r15, @(TCB_ssp,r8)          /* sspの保存 */
    mov.l   @(TCB_usp,r8), r15          /* uspのセット */

    mov     #0, r1
	mov.b	r1,	@(TCB_priv_mode,r8)     /* 特権モード実行を抜ける */
    
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_svc, r1
	mov.l	_mpuvalid_svc, r2
    mov.l   svc_usr_r_1_svc, r3
    jmp     @r3
    nop

svc_prv_r:
    mov.l   @r15+, r0   /* 返り値をスタックから復帰 */

    /*
     *  ipmを復帰
     */
    stc     sr, r1
    mov     #0xf0, r2
    not     r2, r2
    and     r2, r1      /* imaskをクリア */
    or      r10, r1     /* imaskを復帰 */
    ldc     r1, sr

    mov.l   @r15+, r10  /* r10を復帰 */
    mov.l   @r15+, r9   /* r9を復帰 */
    mov.l   @r15+, r8   /* r8を復帰 */
    lds.l   @r15+, pr   /* prを復帰 */
    rts
    nop

svc_entry_7:
	mov.l   _e_rsfn_svc, r0 /* エラーコードを返り値に */ 
	mov.l   r0, @-r15       /* 返り値をスタックに保存 */
	bra     svc_entry_r     /* サービスコールの出口処理へ */
    nop

    .align 4
__kernel_p_runtsk_svc:
    .long __kernel_p_runtsk
_rundom_svc:
    .long _rundom
_svc_table_svc:
    .long _svc_table
_tmin_fncd_svc:
    .long TMIN_FNCD
_e_rsfn_svc:
    .long E_RSFN
_excnest_count_svc:
    .long _excnest_count
_lock_flag_svc:
    .long _lock_flag
_iipm_lock_svc:
    .long IIPM_LOCK
_ipmflg_svc:
    .long _ipmflg
svc_usr_r_1_svc:
    .long svc_usr_r_1
_mpcmpen_svc:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_svc:
	.long 0xaaaa0001

    .section .text_shared
    .align 4
svc_usr_r_1:
    /*
     *  R1: MPUENのアドレス
     *  R2: マジックナンバー
     */
	mov.l	r2, @r1
    
    /*
     *  ipmを復帰
     */
    stc     sr, r1
    mov     #0xf0, r2
    not     r2, r2
    and     r2, r1      /* imaskをクリア */
    or      r10, r1     /* imaskを復帰 */
    ldc     r1, sr

    mov.l   @r15+, r10  /* r10を復帰 */
    mov.l   @r15+, r9   /* r9を復帰 */
    mov.l   @r15+, r8   /* r8を復帰 */
    lds.l   @r15+, pr   /* prを復帰 */
    rts
    nop

    /*
     *  ユーザタスクのタスク例外処理ルーチンの入り口
     *  呼出し条件
     *  - 全割込み禁止状態
     *  - 特権モード実行中（sstk使用中, MPU OFF）
     *  - R0 = p_runtsk
     *  - R10 = ipm
     */
    .global _prepare_texrtn_utask
    .global _prepare_texrtn_utask_r
    .section .text_kernel
    .align 4
_prepare_texrtn_utask:
    /*
     *  スクラッチレジスタの退避
     */
    /*
     *  ユーザスタックの残量をチェック
     *  uspの現在値
     *  特権モード時に使用するサイズ
     *    タスク例外処理本体で使用する分については，
     *    ユーザモード（MPU ON）で実行するために，
     *    実行時にスタックオーバフローを検出できる
     */
    mov.l   _mpceadr4_texrtn, r1    /* r1 = ユーザスタックの終了アドレス */
    mov.l   @r1, r1
    add     #4, r1
    mov.l   @(TCB_usp,r0), r2       /* r2 = usp */
    cmp/hi  r1, r2                  /* r2 > r1 ならば */
    bt      error_nomem_texrtn      /* エラー処理へ */
    mov     #USRTEX_STKSZ, r3       /* size */
    cmp/hi  r2, r3                  /* r3 > r2 ならば */
    bt      error_nomem_texrtn      /* エラー処理へ */
    mov.l   _mpcsadr4_texrtn, r1    /* r1 = ユーザスタックの先頭アドレス */
    mov     r2, r4
    mov.l   @r1, r1
    sub     r3, r4                  /* r4 = usp - size */
    cmp/hi  r4, r1                  /* r1 > r4 ならば */
    bt      error_nomem_texrtn      /* エラー処理へ */
    nop

    /* 
     *  ユーザスタックへの退避
     *   ディスパッチ禁止状態：disdsp   <- new usp
     *   戻り番地：PR
     *                                  <- old usp
     */
    sts.l   pr, @-r2                    /* PRを退避 */
    mov.l   _disdsp_texrtn, r1          /* r1 = &disdsp */
    mov.l   @r1, r1                     /* r1 = disdsp */
    mov.l   r1, @-r2                    /* disdspを退避 */
    mov.l   r2, @(TCB_usp,r0)           /* uspの更新 */

_prepare_texrtn_utask_r:
    /*
     *  タスク例外処理ルーチンの引数を準備
     */
    /* 第1引数：R4 = p_runtsk->texptn */
    /* 第2引数：R5 = p_runtsk->p_tinib->exinf */
    mov.l   @(TCB_texptn,r0), r4        /* 例外パターン */
    mov.l   @(TCB_p_tinib,r0), r2
    mov.l   @(TINIB_exinf,r2), r5      /* 拡張情報 */

    /*
     *  タスク例外処理ルーチンからの戻り番地をcall_ret_texに指定
     */
    mov.l   _call_ret_tex_texrtn, r1
    lds     r1, pr

    /*
     *  タスク例外処理ルーチンへジャンプ後のシステム状態をCPUロック解除状態とする
     */
    mov.l   _lock_flag_texrtn, r3
    mov     #0, r1
    mov.l   r1, @r3

    /*
     *  特権モードを解除してタスク例外処理ルーチンへジャンプ
     */
    mov.l   @(TINIB_texrtn,r2), r1      /* タスク例外処理ルーチンのアドレスを準備 */

    /*
     *  ユーザスタックへの切り替え
     */
    mov.l   r15, @(TCB_ssp,r0)          /* sspの保存 */
    mov.l   @(TCB_usp,r0), r15          /* uspのセット */

    mov     #0, r2
	mov.b	r2,	@(TCB_priv_mode,r0)     /* 特権モード実行を抜ける */
	mov.l	r2,	@(TCB_texptn,r0)        /* p_runtsk->texptn = 0 */
	mov.b	@(TCB_enatex,r0), r3        /* p_runtsk->enatex = false */
    mov     #TCB_enatex_mask, r2
    not     r2, r2
    and     r2, r3
	mov.b	r3,	@(TCB_enatex,r0)        /* p_runtsk->enatex = false */
    
    /* 
     *  MPU有効
     */
	mov.l	_mpcmpen_texrtn, r0
	mov.l	_mpuvalid_texrtn, r2
    mov.l   _texrtn_utask_shared_texrtn, r3
    jmp     @r3
    nop

error_nomem_texrtn:
    /*
     *  エミュレートされたCPU例外を呼ぶ
     */
    sts.l   pr, @-r15   /* 戻り番地を退避 */
	mov.l   _c_routine_emulate_exc_texrtn, r6
    mov.l   @r6, r6
	mov     #TEXCNO_EMULATE_TEXRTN, r5       						
    mov.l   _emulate_exception_entry_texrtn, r0
    jsr     @r0
    nop
    lds.l   @r15+, pr   /* 戻り番地を復帰 */
    rts
    nop

    .align 4
    .extern _emulate_exception_entry
_mpcsadr4_texrtn:
    .long MPCSADR4
_mpceadr4_texrtn:
    .long MPCEADR4
_call_ret_tex_texrtn:
    .long _call_ret_tex
_dspflg_texrtn:
    .long _dspflg
_disdsp_texrtn:
    .long _disdsp
_lock_flag_texrtn:
    .long _lock_flag
_emulate_exception_entry_texrtn:
    .long _emulate_exception_entry
_c_routine_emulate_exc_texrtn:
    .long  __kernel_emulate_texrtn_handler
_mpcmpen_texrtn:
	.long MPCMPEN   /* MPU有効 */
_mpuvalid_texrtn:
	.long 0xaaaa0001
_texrtn_utask_shared_texrtn:
    .long texrtn_utask_shared

    .global texrtn_utask_shared
    .section .text_shared
    .align 4
texrtn_utask_shared:
    /*
     *  R1: MPUENのアドレス
     *  R2: マジックナンバー
     */
	mov.l	r2, @r0
    
    /*
     *  // cpuロック解除
     *  ipmを復帰
     */
    stc     sr, r0
    mov     #0xf0, r2
    not     r2, r2
    and     r2, r0      /* imaskをクリア */
    or      r10, r0     /* imaskを復帰 */
    ldc     r0, sr

    jmp     @r1 
    nop

    .global _call_ret_tex
    .section .text_shared
    .align 4
_call_ret_tex:
    /*
     *  SRにセットされているipmを保存
     *  全割込み禁止
     */
    stc     sr, r0
    and     #0xf0, r0   /* IMASKのみ残す */
    mov     r0, r10     /* IMASKを保存 */
    stc     sr, r0
    or      #0xf0, r0   /* IMASKをセット */
    ldc     r0, sr

    /* 
     *  MPU無効
     */
	mov.l	_mpcmpen_ret_tex, r1
	mov.l	_mpuinvalid_ret_tex, r0
	mov.l	r0, @r1
    mov.l   _call_ret_tex_0_ret_tex, r0
    jmp     @r0
    nop

    .align 4
_call_ret_tex_0_ret_tex:
    .long _call_ret_tex_0
_mpuinvalid_ret_tex:
	.long 0xaaaa0000
_mpcmpen_ret_tex:
	.long MPCMPEN   /* MPU有効 */

#define ACVCT_ACPTN1 0
#define USTK_pr      4
#define USTK_disdsp  0

    .global _call_ret_tex_0
    .section .text_kernel
    .align 2
_call_ret_tex_0:
    /*
     *  呼び出し元が非タスクコンテキストならばエラー
     */
    mov.l   _excnest_count_ret_tex, r0
    mov.l   @r0, r0
	cmp/eq  #0,	r0                      /*  多重割込みでないか？ */
	bf      _error_context_ret_tex      /*  多重割込みならばエラー */
    /*
     *  呼び出し元がカーネルドメインならばエラー
     */
    mov.l   _rundom_ret_tex, r2
    mov.l   @r2, r2
    tst     r2, r2
    bt      _error_context_ret_tex
    nop
    /*
     *  呼び出し元が特権モードならばエラー
     */
    mov.l   _p_runtsk_ret_tex, r0
    mov.l   @r0, r0
    mov.b   @(TCB_priv_mode,r0), r1
    tst     r1, r1
    bf      _error_context_ret_tex
    nop

    /*
     *  TCBのuspの値を更新
     *  ここでTCBのuspを更新するのは，タスク例外処理中に，
     *  サービスコールや割込み処理などで，タスク例外処理
     *  呼び出し時の値から上書きされる可能性があるため
     */
    mov.l   r15, @(TCB_usp, r0)
    /*
     *  ユーザスタックの残量をチェック
     *  uspの現在値
     *  特権モード時に使用するサイズ
     *    タスク例外処理でユーザスタックを不正に操作された場合への対応
     */
    mov.l   _mpcsadr4_ret_tex, r1   /* r1 = ユーザスタックの先頭アドレス */
    mov     r15, r3                 /* r3 = usp */
    mov.l   @r1, r1
    cmp/hi  r3, r1                  /* r1 > r3 ならば */
    bt      error_nomem_ret_tex     /* エラー処理へ */
    mov     #USRTEX_STKSZ, r1              /* size */
    addv    r1, r3                  /* r3 = usp + size: オーバフローが発生したか？ */
    bt      error_nomem_ret_tex     /* エラー処理へ */
    mov.l   _mpceadr4_ret_tex, r1   /* r1 = ユーザスタックの終了アドレス */
    mov.l   @r1, r1
    cmp/hi  r1, r3                  /* r3 > r1 ならば */
    bt      error_nomem_ret_tex     /* エラー処理へ */
    nop

    /*
     *  この時点で，r2 = rundom
     */
    /*
     *  CPUロックを解除
     *  ipmを全解除
     *    タスク例外処理から，CPUロックや割込み優先度マスクを
     *    変更したままリターンした場合への対策
     *    SRはpreare_texrtn_utaskの呼び出し元からリターンする
     *    際に戻るため，ここではSRを更新しない
     */
    /*
     *  リターン後のシステム状態をCPUロック解除状態とする
     */
    mov.l   _lock_flag_ret_tex, r4
    mov     #0, r1
    mov.l   r1, @r4
    /*
     *  ipmを優先度マスク全解除状態へ
     */
    mov     #IIPM_ENAALL, r10
    /*
     *  ipmflgをtrueにする
     */
    mov.l   _ipmflg_ret_tex, r4
    mov     #1, r1
    mov.l   r1, @r4

_call_ret_tex_1:
    /*
     *  disdspをユーザスタックから復帰
     *    タスク例外処理から，ディスパッチを禁止したまま
     *    リターンした場合への対策
     */
    mov.l    _kernel_sysstat_acvct_ret_tex, r3
    mov.l    _disdsp_ret_tex, r4
    mov.l    @(ACVCT_ACPTN1,r3), r3
    tst      r3, r2                              /* r3 & r2 == 0? */
    bt       _call_ret_tex_2
    /*
     *  disdspをユーザスタックから復帰
     */
    mov.l   @(USTK_disdsp,r15), r3
    mov.l   r3, @r4

_call_ret_tex_2:
    /*
     *  dspflgを更新
     *  割込み優先度マスクは全解除状態のはずなので，
     *  disdspのみで判断すればよい
     *  タスク例外処理でCPUロックや割込み優先度マスク
     *  がセットされた場合にもdspflgは変化するため，
     *  dspflgを更新する必要があるのは，disdspを戻す
     *  場合だけでない
     */
    mov.l   @r4, r3
    mov.l   _dspflg_ret_tex, r4
    xor     r1, r3              /* r3 = 1 ^ disdsp */
    mov.l   r3, @r4

    /*
     *  p_runtsk->enatex = false
     */
	mov.b	@(TCB_enatex,r0), r1    /* タスク例外処理許可状態の読出し */
    mov     #TCB_enatex_mask, r2
    not     r2, r2
    and     r2, r1
    mov.b   r1, @(TCB_enatex,r0)

    /*
     *  dspflgがtrueか？
     */
    mov.l   _dspflg_ret_tex, r3
    mov.l   @r3, r3
    tst     r3, r3          /* dspflg == false? */
    bt      _call_ret_tex_3 /* falseならディスパッチしない */
    /*
     *  p_runtsk != p_schedtsk か？
     */
	mov.l   _p_schedtsk_ret_tex, r3 /* r3 <- schedtsk */
	mov.l   @r3, r3
	cmp/eq  r0, r3   				/* runtsk と schedtsk を比較 */
	bt	    _call_ret_tex_3         /* 同じならディスパッチしない */

    /*
     *  dispatchを呼び出すための準備
     */
    /*
     *  CPUロック状態へ
     */
    mov.l   _lock_flag_ret_tex, r4
    mov     #0, r1
    mov.l   r1, @r4

    mov.l   _saved_iipm_ret_tex, r4
    mov     #IIPM_ENAALL, r1
    mov.l   r1, @r4

    /*
     *  SRのセット
     *  カーネル管理の割込みを禁止
     */
    mov.l   _iipm_cpu_lock_ret_tex, r3
    ldc     r3, sr

    /*
     *  dispatchへ
     *  r15をuspからsspに変更: ここまでにuspは更新されないので保存する必要はない 
     */
    mov.l   @(TCB_ssp, r0), r15     
    sts.l   pr, @-r15               /* 戻り番地を保存 */
    mov.l   _dispatch_ret_tex, r2
    jsr     @r2
    nop
    lds.l   @r15+, pr                  /* 戻り番地を復帰 */
    mov.l   r15, @(TCB_ssp, r0)     /* dispatchでTCBのsspは上書きされるためここで復帰 */     
    mov.l   @(TCB_usp, r0), r15     /* r15をsspからuspに変更 */ 

    mov     #0xf0, r0   /* IMASKを全割込み禁止状態へ */
    ldc     r0, sr

    /*
     *  リターン後のシステム状態をCPUロック解除状態とする
     */
    mov.l   _lock_flag_ret_tex, r1
    mov     #0, r0
    mov.l   r0, @r1

_call_ret_tex_3:
    /*
	 *  保留例外要因があるかチェック
	 */
    mov.l   _p_runtsk_ret_tex, r0
    mov.l   @r0, r0
	mov.l	@(TCB_texptn,r0), r1    /* 保留例外要因の読出し */
    tst     r1, r1                  /* 保留例外要因が0か？ */
    bt      _call_ret_tex_r         /* 0ならばリターン処理 */
    nop
    mov.l   @(TCB_ssp, r0), r15
    mov.l   _prepare_texrtn_ret_tex, r1
    jmp     @r1                     /* 0でなければtexrtnを再実行 */
    nop

_call_ret_tex_r:
    add     #USTK_pr, r15
    lds.l   @r15+, pr      /* prを復帰 */
    /*
     *  p_runtsk->enatex = true
     */
	mov.b	@(TCB_enatex,r0), r1    /* タスク例外処理許可状態の読出し */
    mov     #TCB_enatex_mask, r2
    or      r2, r1
    mov.b   r1, @(TCB_enatex,r0)
    /*
     *  uspからsspに切り替え
     */
    mov.l   r15, @(TCB_usp,r0)
    mov.l   @(TCB_ssp,r0), r15
    mov     #2, r1
    mov.b   r1, @(TCB_priv_mode,r0) /* p_runtsk->priv_mode = true */

    rts
    nop

    /*
     *  ret_texでエラーとなる場合は，
     *  ext_tsk
     *  ext_ker
     *  を順に呼び出す
     */
_error_context_ret_tex:
    mov.l   _kernel_ext_tsk_ret_tex, r1
    jsr     @r1
    nop
    mov.l   _kernel_ext_ker_ret_tex, r1
    jsr     @r1
    nop

error_nomem_ret_tex:
    /*
     *  エミュレートされたCPU例外を呼ぶ
     */
    sts.l   pr, @-r15   /* 戻り番地を退避 */
	mov.l   _c_routine_emulate_exc_ret_tex, r6
    mov.l   @r6, r6
	mov     #TEXCNO_EMULATE_RET_TEX, r5       						
    mov.l   _emulate_exception_entry_ret_tex, r0
    jsr     @r0
    nop
    lds.l   @r15+, pr   /* 戻り番地を復帰 */
    rts
    nop

    .align 4
_excnest_count_ret_tex:
    .long _excnest_count
_rundom_ret_tex:
	.long _rundom
_mpcsadr4_ret_tex:
    .long MPCSADR4
_mpceadr4_ret_tex:
    .long MPCEADR4
_iipm_cpu_lock_ret_tex:				/* カーネル管理内割込み禁止用マスク */
	.long  IIPM_LOCK			/* ipm以外のビットはゼロで良い */
_kernel_sysstat_acvct_ret_tex:
    .long __kernel_sysstat_acvct
_call_ret_tex_ret_tex:
    .long _call_ret_tex
_lock_flag_ret_tex:
    .long _lock_flag
_ipmflg_ret_tex:
    .long _ipmflg
_saved_iipm_ret_tex:
    .long _saved_iipm
_dspflg_ret_tex:
    .long _dspflg
_disdsp_ret_tex:
    .long _disdsp
_p_schedtsk_ret_tex:
    .long _p_schedtsk
_p_runtsk_ret_tex:
    .long _p_runtsk
_prepare_texrtn_ret_tex:
    .long _prepare_texrtn_utask_r
_dispatch_ret_tex:
    .long _dispatch
_kernel_ext_tsk_ret_tex:
    .long __kernel_ext_tsk
_kernel_ext_ker_ret_tex:
    .long __kernel_ext_ker
_emulate_exception_entry_ret_tex:
    .long _emulate_exception_entry
_c_routine_emulate_exc_ret_tex:
    .long  __kernel_emulate_ret_tex_handler


$ ======================================================================
$
$   TOPPERS/HRP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       High Reliable system Profile Kernel
$
$   Copyright (C) 2011-2017 by Embedded and Real-Time Systems Laboratory
$               Graduate School of Information Science, Nagoya Univ., JAPAN
$  
$   上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
$   ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$   変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$   (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$       権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$       スコード中に含まれていること．
$   (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$       用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$       者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$       の無保証規定を掲載すること．
$   (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$       用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$       と．
$     (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$         作権表示，この利用条件および下記の無保証規定を掲載すること．
$     (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$         報告すること．
$   (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$       害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$       また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$       由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$       免責すること．
$  
$   本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$   よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$   に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$   アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$   の責任を負わない．
$
$ =====================================================================

$ 
$     パス2のアーキテクチャ依存テンプレート（ARM-M用）
$ 

$ 
$  有効な割込み番号，割込みハンドラ番号
$ 
$INTNO_VALID = RANGE(15, 20)$
$INHNO_VALID = INTNO_VALID$

$ 
$  有効なCPU例外番号
$  7,8はエミュレートされた例外
$ 
$EXCNO_VALID = { 2,3,4,5,6,7,8,11,12,14 }$

$ 
$  ATT_ISRで使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$INTNO_ATTISR_VALID = INTNO_VALID$
$INHNO_ATTISR_VALID = INHNO_VALID$

$ 
$  DEF_INT／DEF_EXCで使用できる割込みハンドラ番号／CPU例外ハンドラ番号
$ 
$INHNO_DEFINH_VALID = INHNO_VALID$
$EXCNO_DEFEXC_VALID = EXCNO_VALID$

$ 
$  CFG_INTで使用できる割込み番号と割込み優先度
$  最大優先度はBASEPRIレジスタでマスクできない優先度（内部優先度'0'）
$  そのため，カーネル管理外の割込みでのみ指定可能．
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = RANGE(-(1 << 4),-1)$
           
$ 
$  kernel/kernel.tf のターゲット依存部
$ 

$ 
$  ユーザスタック領域を確保するコードを出力する
$  ARGV[1]：タスクID
$  ARGV[2]：スタックサイズ
$ 
$FUNCTION ALLOC_USTACK$
    static STK_T _kernel_ustack_$ARGV[1]$[COUNT_STK_T($ARGV[2]$)]
    $SPC$__attribute__((section($FORMAT("\".user_stack.%s\"", ARGV[1])$)));$NL$

    $TSK.TINIB_USTKSZ[ARGV[1]] = VALUE(FORMAT("ROUND_STK_T(%s)", ARGV[2]), +ARGV[2])$
    $TSK.TINIB_USTK[ARGV[1]] = FORMAT("_kernel_ustack_%s", ARGV[1])$
$END$

$ 
$  ユーザスタック領域のセクション名を返す
$  ARGV[1]：タスクID
$ 
$FUNCTION SECTION_USTACK$
    $RESULT = FORMAT(".user_stack.%s", ARGV[1])$
$END$

$ 
$  固定長メモリプール領域を確保するコードを出力する
$  ARGV[1]：固定長メモリプールID
$  ARGV[2]：ドメインID
$  ARGV[3]：ブロック数
$  ARGV[4]：ブロックサイズ
$ 
$FUNCTION ALLOC_UMPF$
    static MPF_T _kernel_mpf_$ARGV[1]$[($ARGV[3]$) * COUNT_MPF_T($ARGV[4]$)]
    $SPC$__attribute__((section($FORMAT("\".mpf.%s\"", ARGV[1])$)));$NL$
$END$

$ 
$  固定長メモリプール領域のセクション名を返す
$  ARGV[1]：タスクID
$ 
$FUNCTION SECTION_UMPF$
    $RESULT = FORMAT(".mpf.%s", ARGV[1])$
$END$

$TARGET_MEMATR_MPFAREA = TA_MEMPRSV$

$ 
$  arch/gcc/ldscript.tfのターゲット依存部
$ 

$FUNCTION GENERATE_PROVIDE$
    PROVIDE(hardware_init_hook = 0);$NL$
    PROVIDE(software_init_hook = 0);$NL$
    PROVIDE(software_term_hook = 0);$NL$
    PROVIDE(_kernel_bsssecinib_table = 0);$NL$
    PROVIDE(_kernel_tnum_bsssec = 0);$NL$
    PROVIDE(_kernel_datasecinib_table = 0);$NL$
    PROVIDE(_kernel_tnum_datasec = 0);$NL$
    $NL$

    $IF LENGTH(DOM.ID_LIST)$
        $FOREACH domid DOM.ID_LIST$
$   RX領域（専用）
            PROVIDE(__start_text_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(__limit_text_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   R領域（専用）
            PROVIDE(__start_rodata_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(__limit_rodata_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   RWX領域（専用）
            PROVIDE(__start_ram_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(__limit_ram_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   共有リード専用ライト
            PROVIDE($FORMAT("__start_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xffffffff);$NL$
            PROVIDE($FORMAT("__limit_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xffffffff);$NL$
        $END$
        $NL$
    $END$$NL$

$  共有領域
    PROVIDE(__start_text_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
    PROVIDE(__limit_text_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
    PROVIDE(__start_rodata_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
    PROVIDE(__limit_rodata_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
    PROVIDE(__start_ram_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
    PROVIDE(__limit_ram_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
$   共有リード専用ライト領域全体
    PROVIDE(__start_srpw_all = 0xffffffff);$NL$
    PROVIDE(__limit_srpw_all = 0xffffffff);$NL$
    $NL$
$END$

$ 
$ 保護ドメイン初期化コンテキストブロックのための宣言
$ 
$FUNCTION PREPARE_DOMINICTXB$
    $IF LENGTH(DOM.ID_LIST)$
        $FOREACH domid DOM.ID_LIST$
$   RX領域（専用）
            extern char __start_text_$DOM.LABEL[domid]$;$NL$
            extern char __limit_text_$DOM.LABEL[domid]$;$NL$
$   R領域（専用）
            extern char __start_rodata_$DOM.LABEL[domid]$;$NL$
            extern char __limit_rodata_$DOM.LABEL[domid]$;$NL$
$   RWX領域（専用）
            extern char __start_ram_$DOM.LABEL[domid]$;$NL$
            extern char __limit_ram_$DOM.LABEL[domid]$;$NL$
$   共有リード専用ライト
            extern char $FORMAT("__start_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
            extern char $FORMAT("__limit_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
        $END$
        $NL$
    $END$$NL$

$  共有領域
    extern char __start_text_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_text_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __start_rodata_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_rodata_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __start_ram_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_ram_$DOM.LABEL[TDOM_NONE]$;$NL$
$   共有リード専用ライト領域全体
    extern char __start_srpw_all;$NL$
    extern char __limit_srpw_all;$NL$
    $NL$
	extern const DOMINIB_INFO _kernel_dominib_info_tbl[];$NL$
$END$

$ 
$  DOMINICTXBの初期化情報を生成
$ 
$DOMINICTXB_KERNEL = "{ NULL }"$

$FUNCTION GENERATE_DOMINICTXB$
	{ (DOMINIB_INFO *)&_kernel_dominib_info_tbl[$ARGV[1]$ - 1] }
$END$

$region_size = 32$
$region_size_mask = 0x04$
$WHILE (region_size <= (4*1024*1024*1024))$
    $hash_region_size_mask[region_size] = (region_size_mask << 1)$
    $region_size = region_size * 2$
    $region_size_mask = region_size_mask + 1$
$END$

$ 
$  TSKINICTXBの初期化情報を生成
$ 
$FUNCTION GENERATE_TSKINICTXB$
	$SPC${
    $SPC$$TSK.TINIB_SSTKSZ[ARGV[1]]$, 
	$SPC$((void *)((char *)($TSK.TINIB_SSTK[ARGV[1]]$)
    $SPC$+ ($TSK.TINIB_SSTKSZ[ARGV[1]]$))),
    $IF (TSK.DOMAIN[ARGV[1]] == TDOM_KERNEL)$
        $SPC$0, 0, 0, 0 },
    $ELSE$
        $SPC$$TSK.TINIB_USTKSZ[ARGV[1]]$, 
        $SPC$((void *)((char *)($TSK.TINIB_USTK[ARGV[1]]$)
        $SPC$+ ($TSK.TINIB_USTKSZ[ARGV[1]]$))),
        $SPC$((char *)$TSK.TINIB_USTK[ARGV[1]]$ + 0x10 + 0x03 ),
        $SPC$( TO_MPU_AP( MPU_AP_PURW ) | MPU_REGATR_C | MPU_REGATR_B | $hash_region_size_mask[TSK.STKSZ[ARGV[1]]]$ | MPU_REGATR_ENABLE ),$SPC$
        $SPC$},
    $END$
$END$

$ 
$  メモリ保護単位の実際のサイズを受け取り，受け取ったサイズが，
$  ARM MPUのサイズ制約を満たすかどうかを返す
$  
$FUNCTION CHECK_ARM_MPU_SIZE$
    $search_state = 0$
    $FOREACH bit_offset { 31,30,...,5 }$
        $compare_bit = 1 << bit_offset$
        $IF ((ARGV[1] & compare_bit) != 0)$
            $IF (search_state == 0)$
                $RESULT = 1$
                $search_state = 1$
            $ELIF (search_state == 1)$
                $RESULT = 0$
                $search_state = 2$
            $END$
        $END$
    $END$
    $IF (search_state == 0)$
        $RESULT = 0$
    $ELIF (search_state == 1) && ((ARGV[1] & 0x1f) != 0)$
        $RESULT = 0$
    $END$
$END$

$ 
$  ATT_MEMのターゲット依存部
$  ATTMEM_LIST: ATT_MEMで登録されたMEMのリスト
$  DOM.MEM_COUNT[domid]: 保護ドメインに属するMEMの数
$                       1保護ドメインあたりに1つのMEMのみ属することができる
$                       1つのMEMが共有であった場合には，他のMEMは登録できない
$  SHARED_MEM_COUNT: 共有のMEMの数
$  ALL_MEM_COUNT: MEMの数
$  DOM.MEM_BASE[domid]: 保護ドメインに属するMEMの先頭番地
$  DOM.MEM_SIZE[domid]: 保護ドメインに属するMEMのサイズ
$ATTMEM_LIST = {}$
$ALL_MEM_COUNT = 0$
$SHARED_MEM_COUNT = 0$
$FOREACH domid DOM.ID_LIST$
    $DOM.MEM_COUNT[domid] = 0$
    $DOM.MEM_BASE[domid] = 0$
    $DOM.MEM_SIZE[domid] = 0$
$END$
$FUNCTION HOOK_ERRORCHECK_MEM$
    $memid = ARGV[1]$
    $IF !CHECK_ARM_MPU_SIZE(MEM.SIZE[memid])$
        $ERROR$$FORMAT("size of MEM(base:%1%, size:%2%) does not meet MPU constraints.", MEM.BASE[memid], MEM.SIZE[memid])$$END$
    $END$

    $IF EQ(MEM.DOMAIN[memid], "")$
$       // 共有の場合
        $IF ALL_MEM_COUNT > 0$
            $ERROR$$FORMAT("ATT_MEM is too much.")$$END$
        $ELSE$
            $SHARED_MEM_COUNT = SHARED_MEM_COUNT + 1$
            $ATTMEM_LIST = APPEND(ATTMEM_LIST, memid)$

            $FOREACH domid DOM.ID_LIST$
                $DOM.MEM_BASE[domid] = MEM.BASE[memid]$
                $DOM.MEM_SIZE[domid] = MEM.SIZE[memid]$
            $END$
        $END$
    $ELIF SHARED_MEM_COUNT == 0$
$       // 共有にMEMが存在せず，保護ドメインに属する場合
        $IF DOM.MEM_COUNT[MEM.DOMAIN[memid]] > 0$
            $ERROR$$FORMAT("ATT_MEM is too much in DOMAIN(%1%).", MEM.DOMAIN[memid])$$END$
        $ELSE$
            $DOM.MEM_COUNT[MEM.DOMAIN[memid]] = DOM.MEM_COUNT[MEM.DOMAIN[memid]] + 1$
            $ATTMEM_LIST = APPEND(ATTMEM_LIST, memid)$

            $DOM.MEM_BASE[MEM.DOMAIN[memid]] = MEM.BASE[memid]$
            $DOM.MEM_SIZE[MEM.DOMAIN[memid]] = MEM.SIZE[memid]$
        $END$
    $ELSE$
        $ERROR$$FORMAT("ATT_MEM is too much.")$$END$
    $END$
    $ALL_MEM_COUNT = ALL_MEM_COUNT + 1$
$END$

$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/kernel.tf"$

$ 
$  ユーザスタックのサイズチェック
$ 
$FOREACH tskid TSK.ID_LIST$
    $IF (TSK.DOMAIN[tskid] != TDOM_KERNEL) && !CHECK_ARM_MPU_SIZE(TSK.STKSZ[tskid])$
        $ERROR$$FORMAT("the user stack of TASK(%1%) does not meet MPU size constraints. size=%2%", tskid, +TSK.STKSZ[tskid])$$END$
    $END$
$END$

/*$NL$
$SPC$*  Target-dependent Definitions (ARM-M)$NL$
$SPC$*/$NL$
$NL$

$ 
$  ベクターテーブル
$ 
$FILE "kernel_cfg.c"$
$NL$
__attribute__ ((section(".vector"))) $NL$
const FP _kernel_vector_table[] =      $NL$ 
{                                    $NL$
	$TAB$(FP)(TOPPERS_ISTKPT(TOPPERS_ISTK, TOPPERS_ISTKSZ)),   // 0 The initial stack pointer $NL$
	$TAB$(FP)_start,                 // 1 The reset handler         $NL$

$FOREACH excno {2,3,...,14}$ 
	$IF excno == 11$
		$TAB$(FP)(_kernel_svc_handler),      // 11 SVCall handler
	$ELSE$
		$TAB$(FP)(_kernel_core_exc_entry),
	$END$
	$SPC$$FORMAT("/* %d */", +excno)$$NL$
$END$

$FOREACH inhno INTNO_VALID$ 
	$IF LENGTH(INH.INHNO[inhno]) && ((INH.INHATR[inhno] & TA_NONKERNEL) != 0)$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_core_int_entry),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$


$NL$};$NL$
$NL$

$NL$
const FP _kernel_exc_tbl[] = $NL$
{$NL$
$FOREACH excno {0,1,...,14}$
	$IF LENGTH(EXC.EXCNO[excno])$
		$TAB$(FP)($EXC.EXCHDR[excno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_exc_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +excno)$$NL$
$END$


$FOREACH inhno INTNO_VALID$
	$IF LENGTH(INH.INHNO[inhno])$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_int_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$


$NL$};$NL$
$NL$

$ 
$  _kernel_bitpat_cfgintの生成
$ 

$bitpat_cfgint_num = 0$
$bitpat_cfgint = 0$


const uint32_t _kernel_bitpat_cfgint[
$IF (TMAX_INTNO & 0x0f) == 0x00 $
	$bitpat_cfgint_num = (TMAX_INTNO >> 4)$
$ELSE$
	$bitpat_cfgint_num = (TMAX_INTNO >> 4) + 1$
$END$
	$bitpat_cfgint_num$
] = {$NL$
$FOREACH num RANGE(0,(bitpat_cfgint_num-1))$
$   //boost のバージョンによって挙動が変わるための対策
$   //http://www.toppers.jp/TOPPERS-USERS/201004/msg00034.html
	$bitpat_cfgint = 1-1$
	$FOREACH inhno RANGE(num*32, (num*32)+31)$
		$IF LENGTH(INH.INHNO[inhno])$
			$bitpat_cfgint = bitpat_cfgint | (1 << (inhno & 0x01f))$
		$END$
	$END$
	$TAB$UINT32_C($FORMAT("0x%08x", bitpat_cfgint)$), $NL$
$END$

$NL$};$NL$
$NL$


$ 
$  割込み優先度テーブル（内部表現）
$ 
const uint32_t _kernel_int_iipm_tbl[] = {$NL$
$FOREACH excno {0,1,...,14}$
	$TAB$$FORMAT("UINT32_C(0x%08x), /* 0x%03x */", 0, +excno)$$NL$
$END$

$FOREACH intno INTNO_VALID$
	$IF LENGTH(INT.INTNO[intno])$
		$intpri = (((1 << TBITW_IPRI) + INT.INTPRI[intno]) << (8 - TBITW_IPRI))$
	$ELSE$
$		// LSBを1にしているのは，割込み属性が設定されていないことを判
$		// 別するためである．
		$intpri = 0 $
	$END$
	$TAB$$FORMAT("UINT32_C(0x%08x), /* 0x%03x */", intpri, +intno)$$NL$
$END$
$NL$};$NL$
$NL$


$ 
$  GCC依存部のテンプレートファイルのインクルード
$ 
$INCLUDE "gcc/ldscript.tf"$ 

$ 
$  共有領域の初期化ブロックを生成
$ 
$FILE "kernel_cfg.h"$
extern const uint_t tnum_shared_mem;$NL$
extern char * const shared_meminib_table[];$NL$
$NL$

$FILE "kernel_mem2.c"$

$PREPARE_DOMINICTXB()$

$IF LENGTH(DOMLIST) > 0$
    const DOMINIB_INFO _kernel_dominib_info_tbl[TNUM_DOMID] ={$NL$
    $JOINEACH domid DOM.ID_LIST ",\n"$
        $TAB${$NL$
$   RX領域（専用）
        $TAB$$TAB$( (char *)&__start_text_$DOM.LABEL[domid]$ ),/* region 4 */$NL$
        $TAB$$TAB$( (uint32_t)&__limit_text_$DOM.LABEL[domid]$ ),/* region 4 */$NL$
$   RWX領域（専用）
        $TAB$$TAB$( (char *)&__start_ram_$DOM.LABEL[domid]$ ),/* region 5 */$NL$
        $TAB$$TAB$( (uint32_t)&__limit_ram_$DOM.LABEL[domid]$ ),/* region 5 */$NL$
$   共有リード専用ライト
        $TAB$$TAB$$FORMAT("( (char *)&__start_ram_%s_%x_%x )", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$,/* region 6 */$NL$
        $TAB$$TAB$$FORMAT("( (uint32_t)&__limit_ram_%s_%x_%x )", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$,/* region 6 */$NL$
$   ATT_MEM領域
        $TAB$$TAB$$FORMAT("( (char *)%1% )", DOM.MEM_BASE[domid])$,/* region 7 */$NL$
        $TAB$$TAB$$FORMAT("( (uint32_t)(%1% + %2%) )", DOM.MEM_BASE[domid], DOM.MEM_SIZE[domid])$,/* region 7 */$NL$
        $TAB$}
    $END$
    ,$NL$};$NL$
    $NL$
$END$

$TNUM_SHARED_REGION = 3$
const uint_t tnum_shared_mem = $TNUM_SHARED_REGION * 2$;$NL$
char * const shared_meminib_table[$TNUM_SHARED_REGION * 2$] = {$NL$
    $TAB$((char *)&__start_text_$DOM.LABEL[TDOM_NONE]$),$TAB$/* region 0 */$NL$
    $TAB$((char *)&__limit_text_$DOM.LABEL[TDOM_NONE]$),$TAB$/* region 0 */$NL$
    $TAB$((char *)&__start_ram_$DOM.LABEL[TDOM_NONE]$),$TAB$/* region 1 */$NL$
    $TAB$((char *)&__limit_ram_$DOM.LABEL[TDOM_NONE]$),$TAB$/* region 1 */$NL$
    $TAB$((char *)&__start_srpw_all),$TAB$/* region 2 */$NL$
    $TAB$((char *)&__limit_srpw_all),$TAB$/* region 2 */$NL$
};$NL$
$NL$

$FILE "cfg2_out.tf"$
$JOINEACH domid DOM.ID_LIST "\n"$
    $$DOM.MEM_BASE[$+domid$] = $FORMAT("\"%1%\"", DOM.MEM_BASE[domid])$$$$NL$
    $$DOM.MEM_SIZE[$+domid$] = $DOM.MEM_SIZE[domid]$$$$NL$
$END$

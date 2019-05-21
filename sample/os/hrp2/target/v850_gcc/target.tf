$
$ TOPPERS/ASP Kernel
$     Toyohashi Open Platform for Embedded Real-Time Systems/
$     Advanced Standard Profile Kernel
$
$ Copyright (C) 2010 by Meika Sugimoto
$
$ 上記著作権者は，以下の(1)潤ｵ(4)の条件を満たす場合に限り，本ソフトウェ
$ ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$ 変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$ (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$     権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$     スコード中に含まれていること．
$ (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$     用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$     者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$     の無保証規定を掲載すること．
$ (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$     用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$     と．
$   (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$       作権表示，この利用条件および下記の無保証規定を掲載すること．
$   (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$       報告すること．
$ (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$     害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$     また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$     由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$     免責すること．
$
$ 本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$ よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$ に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$ アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$ の責任を負わない．
$ 
$  標準のセクションのメモリオブジェクト属性の定義
$ 
$MEMATR_TEXT = (TA_NOWRITE|TA_EXEC)$
$MEMATR_RODATA = TA_NOWRITE$
$MEMATR_DATA = TA_MEMINI$
$MEMATR_BSS = TA_NULL$
$MEMATR_PRSV = TA_MEMPRSV$

$TARGET_MEMATR_USTACK = TA_NULL$

$ 
$  標準のセクションに関する定義
$  DSEC.ORDER_LIST：IDのリスト
$  DESC.MEMREG：セクションを配置するメモリリージョン
$  DESC.SECTION：セクション名
$  DESC.MEMATR：セクションのメモリオブジェクト属性
$ 
$FUNCTION DEFINE_DSEC$
    $DSEC.ORDER_LIST = RANGE(0,4)$

    $DSEC.MEMREG[0] = STANDARD_ROM$
    $DSEC.SECTION[0] = ".text"$
    $DSEC.MEMATR[0] = MEMATR_TEXT$

    $DSEC.MEMREG[1] = STANDARD_ROM$
    $DSEC.SECTION[1] = ".rodata"$
    $DSEC.MEMATR[1] = MEMATR_RODATA$

    $DSEC.MEMREG[2] = STANDARD_RAM$
    $DSEC.SECTION[2] = ".data"$
    $DSEC.MEMATR[2] = MEMATR_DATA$

    $DSEC.MEMREG[3] = STANDARD_RAM$
    $DSEC.SECTION[3] = ".bss"$
    $DSEC.MEMATR[3] = MEMATR_BSS$

    $DSEC.MEMREG[4] = STANDARD_ROM$
    $DSEC.SECTION[4] = ".idata"$
    $DSEC.MEMATR[4] = TA_NOWRITE$

$END$

$ 
$  ユーザスタック領域を確保するコードを出力する
$  ARGV[1]：タスクID
$  ARGV[2]：スタックサイズ
$ 
$FUNCTION ALLOC_USTACK$
    static STK_T _kernel_ustack_$ARGV[1]$[COUNT_STK_T($ARGV[2]$)]
    $SPC$__attribute__((section($FORMAT("\".user_stack.%s\"", ARGV[1])$)));$NL$

    $TSK.TINIB_USTKSZ[ARGV[1]] = FORMAT("ROUND_STK_T(%s)", ARGV[2])$
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
$FUNCTION GENERATE_OUTPUT$
    OUTPUT_FORMAT("elf32-sh") $NL$
    OUTPUT_ARCH(sh)           $NL$
    STARTUP(start.o)          $NL$
    $NL$
$END$

$FUNCTION GENERATE_PROVIDE$
    PROVIDE(_hardware_init_hook = 0);$NL$
    PROVIDE(_software_init_hook = 0);$NL$
    PROVIDE(_software_term_hook = 0);$NL$
    PROVIDE(__kernel_bsssecinib_table = 0);$NL$
    PROVIDE(__kernel_tnum_bsssec = 0);$NL$
    PROVIDE(__kernel_datasecinib_table = 0);$NL$
    PROVIDE(__kernel_tnum_datasec = 0);$NL$
    $NL$

    $IF LENGTH(DOM.ID_LIST)$
        $FOREACH domid DOM.ID_LIST$
$   RX領域（専用）
            PROVIDE(___start_text_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(___limit_text_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   R領域（専用）
            PROVIDE(___start_rodata_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(___limit_rodata_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   RWX領域（専用）
            PROVIDE(___start_ram_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
            PROVIDE(___limit_ram_$DOM.LABEL[domid]$ = 0xffffffff);$NL$
$   共有リード専用ライト
            PROVIDE($FORMAT("___start_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xffffffff);$NL$
            PROVIDE($FORMAT("___limit_ram_%s_%x_%x", DOM.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xffffffff);$NL$
$  共有領域
            PROVIDE(___start_text_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
            PROVIDE(___limit_text_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
            PROVIDE(___start_rodata_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
            PROVIDE(___limit_rodata_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
            PROVIDE(___start_ram_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
            PROVIDE(___limit_ram_$DOM.LABEL[TDOM_NONE]$ = 0xffffffff);$NL$
        $END$
        $NL$
    $END$$NL$
$   共有リード専用ライト領域全体
    PROVIDE(___start_srpw_all = 0xffffffff);$NL$
    PROVIDE(___limit_srpw_all = 0xffffffff);$NL$
$END$

$FUNCTION GENERATE_SECTION_FIRST$
    $TAB$.vector_start : {$NL$
    $TAB$$TAB$__vector_start = .;$NL$
    $TAB$$TAB$*(.vector)$NL$
    $TAB$$TAB$__vector_end = .;$NL$
    $TAB$$TAB$start.o(.text_kernel)$NL$
    $TAB$$TAB$start.o(*.text*)$NL$
    $TAB$} > $REG.REGNAME[STANDARD_ROM]$$NL$
    $NL$
    $TAB$.vector_entry : {$NL$
    $TAB$$TAB$__vector_entry_start = .;$NL$
    $TAB$$TAB$*(.vector_entry)$NL$
    $TAB$$TAB$__vector_entry_end = .;$NL$
    $TAB$}  > $REG.REGNAME[STANDARD_ROM]$$NL$
    $NL$
    $TAB$.bss : {$NL$
    $TAB$} > $REG.REGNAME[STANDARD_RAM]$$NL$
    $NL$
$END$

$TARGET_PAGE_SIZE_STR = 4$
$TARGET_SEC_ALIGN_STR = 4$

$TOPPERS_SUPPORT_ATT_MOD = 1$

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
        extern uint32_t dom_valid_map_table[];
        $NL$
    $END$$NL$

$  共有領域
    extern char __start_text_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_text_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __start_rodata_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_rodata_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __start_ram_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __limit_ram_$DOM.LABEL[TDOM_NONE]$;$NL$
    extern char __start_srpw_all;$NL$
    extern char __limit_srpw_all;$NL$

    $IF LENGTH(TSK.ID_LIST)$
        $FOREACH tskid TSK.ID_LIST$
            $IF TSK.DOMAIN[tskid] != TDOM_KERNEL$
                extern char $FORMAT("__start_user_stack%s", tskid)$;$NL$
                extern char $FORMAT("__limit_user_stack%s", tskid)$;$NL$
            $END$
        $END$
    $END$
$END$

$
$  DOMINICTXBの初期化情報を生成
$ 
$FUNCTION GENERATE_DOMINICTXB$
	$SPC${
$   RX領域（専用）
    $SPC$&__start_text_$DOM.LABEL[ARGV[1]]$,
    $SPC$(&__limit_text_$DOM.LABEL[ARGV[1]]$ - 4),
$   R領域（専用）
    $SPC$&__start_rodata_$DOM.LABEL[ARGV[1]]$,
    $SPC$(&__limit_rodata_$DOM.LABEL[ARGV[1]]$ - 4),
$   RWX領域（専用）
    $SPC$&__start_ram_$DOM.LABEL[ARGV[1]]$,
    $SPC$(&__limit_ram_$DOM.LABEL[ARGV[1]]$ - 4),
$   共有リード専用ライト
    $SPC$$FORMAT("&__start_ram_%s_%x_%x", DOM.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,
    $SPC$$FORMAT("(&__limit_ram_%s_%x_%x - 4)", DOM.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,
$   領域の有効ビットのマップ
    $SPC$$FORMAT("&dom_valid_map_table[%d]", (+ARGV[1] - 1))$
	},
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
        $SPC$$FORMAT("&__start_user_stack%s", ARGV[1])$,
        $SPC$$FORMAT("(&__limit_user_stack%s - 4)", ARGV[1])$,
        },
    $END$
$END$
$
$  リンカのためのセクション記述の生成
$
$FUNCTION SECTION_DESCRIPTION$
	$IF EQ(ARGV[1], ".rodata")$
		$RESULT = ".rodata .rodata.str1.4"$
	$ELIF EQ(ARGV[1], ".bss")$
		$RESULT = ".bss COMMON"$
	$ELSE$
		$RESULT = ARGV[1]$
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
    $WARNING$$FORMAT("ATT_MEM is not supported.")$$END$
$END$


$ 
$  ユーザスタックのサイズチェック
$ 
$FOREACH tskid TSK.ID_LIST$
    $IF (TSK.DOMAIN[tskid] != TDOM_KERNEL) && (TSK.STKSZ[tskid] % 4)$
        $ERROR$$FORMAT("the user stack of TASK(%1%) does not meet MPU size constraints. size = %2%", tskid, +TSK.STKSZ[tskid])$$END$
    $END$
$END$

$ 
$  共有領域の初期化ブロックを生成
$ 
$FILE "kernel_cfg.h"$
extern const uint_t tnum_shared_mem;$NL$
extern char * const shared_meminib_table[];$NL$
$NL$

$FILE "kernel_cfg.c"$
#include "kernel.h"$NL$
#include "kernel_cfg.h"$NL$
$TNUM_SHARED_REGION = 4$
const uint_t tnum_shared_mem = $TNUM_SHARED_REGION * 2$;$NL$
char * const shared_meminib_table[$TNUM_SHARED_REGION * 2$] = {$NL$
    $SPC$&__start_text_$DOM.LABEL[TDOM_NONE]$,
    $SPC$(&__limit_text_$DOM.LABEL[TDOM_NONE]$ - 4),
    $SPC$&__start_rodata_$DOM.LABEL[TDOM_NONE]$,
    $SPC$(&__limit_rodata_$DOM.LABEL[TDOM_NONE]$ - 4),
    $SPC$&__start_ram_$DOM.LABEL[TDOM_NONE]$,
    $SPC$(&__limit_ram_$DOM.LABEL[TDOM_NONE]$ - 4),
    $SPC$&__start_srpw_all,
    $SPC$(&__limit_srpw_all - 4),
};$NL$
$NL$
$TOPPERS_SUPPORT_ATT_MOD = 1$

$INCLUDE "v850_gcc/v850es_fk3.tf"$
$IF LENGTH(TEXCNO_EMULATE_RET_TEX)$
    $TAB$.section .rodata_kernel$NL$
    $TAB$.align 4$NL$
    $TAB$.global __kernel_emulate_ret_tex_handler$NL$
    $TAB$.extern
    $IF LENGTH(EXC.EXCHDR[TEXCNO_EMULATE_RET_TEX])$
        $SPC$_$EXC.EXCHDR[TEXCNO_EMULATE_RET_TEX]$
    $ELSE$
        $SPC$__kernel_default_exc_handler
    $END$
    $NL$

    __kernel_emulate_ret_tex_handler:$NL$
    $IF LENGTH(EXC.EXCHDR[TEXCNO_EMULATE_RET_TEX])$
        $TAB$.long _$EXC.EXCHDR[TEXCNO_EMULATE_RET_TEX]$$NL$
    $ELSE$
        $TAB$.long __kernel_default_exc_handler$NL$
    $END$
$END$

$ 
$  MPU有効領域のビットマップ情報を仮生成
$ 
$FILE "kernel_mem2.c"$
$IF LENGTH(DOM.ID_LIST)$
	const uint32_t dom_valid_map_table[TNUM_DOMID] = {$NL$
	$JOINEACH domid DOM.ID_LIST ",\n"$
        $TAB$0x0
	$END$,$NL$
	};$NL$
	$NL$
$ELSE$
	const DOMINIB dom_valid_map_table[0];$NL$
$END$$NL$
$ $INCLUDE "arch/logtrace/tlv.tf"$

$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
$  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
$  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
$  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$
$  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
$  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
$  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
$  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
$      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
$      スコード中に含まれていること．
$  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
$      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
$      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
$      の無保証規定を掲載すること．
$  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
$      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
$      と．
$    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
$        作権表示，この利用条件および下記の無保証規定を掲載すること．
$    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
$        報告すること．
$  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
$      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
$      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
$      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
$      免責すること．
$
$  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
$  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
$  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
$  用する者に対して，AUTOSARパートナーになることを求めている．
$
$  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
$  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
$  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
$  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
$  の責任を負わない．
$
$  $Id: prc.tf 189 2015-06-26 01:54:57Z t_ishikawa $
$

$
$     パス2のアーキテクチャ依存テンプレート（V850用）
$

$ 
$  kernel/kernel.tf のターゲット依存部
$ 

$ 
$  ユーザスタック領域を確保するコードを出力する
$  ARGV[1]：タスクID
$  ARGV[2]：スタックサイズ
$ 
$FUNCTION ALLOC_USTACK$
    StackType _kernel_ustack_$ARGV[1]$[COUNT_STK_T($ARGV[2]$)]
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
$  ユーザスタックのアライメント制約に合わせたサイズを返す
$  ARGV[1]：スタックサイズ（アライン前）
$  kernel.tfから呼ばれる
$
$FUNCTION USTACK_ALIGN_SIZE$
	$RESULT = (ARGV[1] + CHECK_USTKSZ_ALIGN - 1) & ~(CHECK_USTKSZ_ALIGN - 1)$
$END$

$
$  基本タスクの共有スタックの確保
$  ARGV[1]：共有スタック名
$  ARGV[2]：共有スタックID(タスク優先度)
$  ARGV[3]：スタックサイズ(アライメント調整済み)
$  kernel.tfから呼ばれる
$
$FUNCTION ALLOC_SHARED_USTACK$
	StackType $ARGV[1]$[COUNT_STK_T(ROUND_STK_T($ARGV[3]$))]
	$SPC$__attribute__((section($FORMAT("\".shared_user_stack.%s\"", ARGV[2])$)));$NL$
$END$

$
$  基本タスクの共有スタックのセクション名
$  ARGV[1]：共有スタックID(タスク優先度)
$  kernel.tfから呼ばれる
$
$FUNCTION SECTION_SHARED_USTACK$
	$RESULT = FORMAT(".shared_user_stack.%s", ARGV[1])$
$END$

$ 
$  OSAP初期化コンテキストブロックのための宣言
$ 
$FUNCTION PREPARE_OSAPINICTXB$
$   データセクション初期化テーブルをROMに配置する
$   rosdata領域はRAMに配置されるため，データセクション初期化テーブルをsdata化すると
$   データセクションの初期化に失敗する
    extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_datasec;$NL$
    extern const DATASECINIB __attribute__((section(".rodata_kernel"), aligned(4))) datasecinib_table[];$NL$
    extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_bsssec;$NL$
    extern const BSSSECINIB __attribute__((section(".rodata_kernel"), aligned(4))) bsssecinib_table[];$NL$
$END$

$FUNCTION ASM_MACRO$
	.macro $ARGV[1]$ $ARGV[2]$
$END$

$INCLUDE "arch/v850_gcc/prc_common.tf"$

$IF __v850e2v3__$

$ 割込みベクタと各割込み入口処理(V850E2V3のみ)

$FILE "Os_Lcfg_asm.S"$


$
$ アセンブラ出力用の関数群
$

$FUNCTION EXCEPTION_VECTOR_SECTION$
$TAB$.section .vector,"ax"$NL$
$END$

$FUNCTION ASM_GLOBAL$
	$TAB$.global $ARGV[1]$
$END$

$FUNCTION ASM_LABEL$
	FLABEL($ARGV[1]$)
$END$

$FUNCTION ASM_COMMENT$
	//
$END$


#include <v850asm.inc>$NL$$NL$

$VECTOR_ASMOUT()$

$END$
$ =end IF __v850e2v3__


$IF __v850e3v5__$

$
$ テーブル参照方式用ベクタテーブル(v850e3v5)
$

extern void interrupt(void);$NL$
const uint32 __attribute__((aligned(512))) intbp_tbl[TNUM_INT] = {$NL$
$JOINEACH intno INTNO_VALID "\n"$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$
		$TAB$(uint32)&$ISR.INT_ENTRY[isrid]$
	$ELSE$
		$TAB$(uint32)&interrupt
	$END$
$	//カンマの出力（最後の要素の後ろに出力しない）
	$IF intno != AT(INTNO_VALID,LENGTH(INTNO_VALID) - 1)$
		,
	$END$
	$TAB$$FORMAT("/* %d */", intno)$
$END$
$NL$};$NL$

$END$

$ 
$  arch/gcc/ldscript.tfのターゲット依存部
$ 

$FUNCTION GENERATE_MEMORY$
    $NOOP()$
$END$

$FUNCTION GENERATE_OUTPUT$
    OUTPUT_FORMAT("elf32-v850-rh850","elf32-v850-rh850","elf32-v850-rh850")$NL$
    OUTPUT_ARCH(v850-rh850)$NL$
    $NL$
$END$

$FUNCTION GENERATE_PROVIDE$
    PROVIDE(_hardware_init_hook = 0);$NL$
    PROVIDE(_software_init_hook = 0);$NL$
    PROVIDE(_software_term_hook = 0);$NL$
    PROVIDE(_bsssecinib_table = 0);$NL$
    PROVIDE(_tnum_bsssec = 0);$NL$
    PROVIDE(_datasecinib_table = 0);$NL$
    PROVIDE(_tnum_datasec = 0);$NL$
    $NL$

    $IF LENGTH(OSAP.ID_LIST)$
        $FOREACH domid OSAP.ID_LIST$
            $IF !OSAP.TRUSTED[domid]$
$   RX領域（専用）
                PROVIDE(___start_text_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
                PROVIDE(___limit_text_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
$   R領域（専用）
                PROVIDE(___start_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ = 0xfffffff0);$NL$
                PROVIDE(___limit_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ = 0xfffffff0);$NL$
$   RWX領域（専用）
                PROVIDE(___start_ram_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
                PROVIDE(___limit_ram_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
                PROVIDE(___start_sram_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
                PROVIDE(___limit_sram_$OSAP.LABEL[domid]$ = 0xfffffff0);$NL$
$   共有リード専用ライト
                PROVIDE($FORMAT("___start_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xfffffff0);$NL$
                PROVIDE($FORMAT("___limit_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xfffffff0);$NL$
                PROVIDE($FORMAT("___start_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xfffffff0);$NL$
                PROVIDE($FORMAT("___limit_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$ = 0xfffffff0);$NL$
            $END$
        $END$
        $NL$
    $END$$NL$

$  共有領域
    PROVIDE(___start_text_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
    PROVIDE(___limit_text_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
    PROVIDE(___start_sram_$OSAP.LABEL[TDOM_NONE]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ = 0xfffffff0);$NL$
    PROVIDE(___limit_sram_$OSAP.LABEL[TDOM_NONE]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ = 0xfffffff0);$NL$
    PROVIDE(___start_ram_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
    PROVIDE(___limit_ram_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
    PROVIDE(___start_sram_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
    PROVIDE(___limit_sram_$OSAP.LABEL[TDOM_NONE]$ = 0xfffffff0);$NL$
$   共有リード専用ライト領域全体
    PROVIDE(___start_srpw_all = 0xfffffff0);$NL$
    PROVIDE(___limit_srpw_all = 0xfffffff0);$NL$
    PROVIDE(___start_ssrpw_all = 0xfffffff0);$NL$
    PROVIDE(___limit_ssrpw_all = 0xfffffff0);$NL$
    STARTUP(start.o);$NL$
    ENTRY(__reset);$NL$
    $NL$
$END$

$FUNCTION GENERATE_GP_LABEL$
    $TAB$$TAB$__gp = . + 32K;$NL$
    $TAB$$TAB$__tp = . + 32K;$NL$
$END$

$FILE "kernel_mem2.c"$

$IF __v850e2v3__$
$   // v850
$TNUM_SHARED_REGION = 3$
const uint32 tnum_shared_mem = $TNUM_SHARED_REGION * 2$;$NL$
uint8 * const shared_meminib_table[$TNUM_SHARED_REGION * 2$] = {$NL$
$TAB$((uint8 *)&__start_text_$OSAP.LABEL[TDOM_NONE]$),$TAB$/* iregion 2 */$NL$
$TAB$((uint8 *)&__limit_text_$OSAP.LABEL[TDOM_NONE]$),$TAB$/* iregion 2 */$NL$

$TAB$((uint8 *)NULL),$TAB$/* iregion 3 */$NL$
$TAB$((uint8 *)NULL),$TAB$/* iregion 3 */$NL$

$TAB$((uint8 *)NULL),$TAB$/* dregion 5 */$NL$
$TAB$((uint8 *)NULL),$TAB$/* dregion 5 */$NL$
};$NL$
$NL$
$ELIF __v850e3v5__$
$   // rh850
$IF TNUM_MPU_SHARED > 0$
uint8 * const shared_meminib_table[$TNUM_MPU_SHARED * 3$] = {$NL$
$FOREACH memid RANGE(0, TNUM_MPU_SHARED - 1)$
    $TAB$((uint8 *)NULL),$TAB$/* MPUL $TNUM_MPU_REG - memid$ */$NL$
    $TAB$((uint8 *)NULL),$TAB$/* MPUA $TNUM_MPU_REG - memid$ */$NL$
    $TAB$((uint8 *)NULL),$TAB$/* MPAT $TNUM_MPU_REG - memid$ */$NL$
$END$
};$NL$
$NL$
$END$

$ 
$  GCC依存部のテンプレートファイルのインクルード
$ 
$INCLUDE "gcc/ldscript.tf"$ 

$FILE "cfg2_out.tf"$
$ LNKSEC.*の出力
$$numls = $numls$$$$NL$
$NL$
$FOREACH lsid RANGE(1, numls)$
	$$LNKSEC.MEMREG[$lsid$] = $LNKSEC.MEMREG[lsid]$$$$NL$
	$$LNKSEC.SECTION[$lsid$] = $ESCSTR(LNKSEC.SECTION[lsid])$$$$NL$
	$NL$
$END$

$JOINEACH tskid TSK.ID_LIST "\n"$
	$$TSK.ID[$+tskid$] = $TSK.ID[tskid]$$$
$END$
$NL$


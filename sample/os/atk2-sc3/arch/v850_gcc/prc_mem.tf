$ ======================================================================
$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2013-2015 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$   Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
$               Graduate School of Information Science, Nagoya Univ., JAPAN
$  
$   上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
$  
$ =====================================================================

$ 
$  パス4のプロセッサ依存テンプレート（V850 GCC/GHS用）
$  

$ 
$  ユーザスタック領域のセクションラベル名を返す
$  ARGV[1]：タスクID
$ 
$FUNCTION SECTION_USTACK$
    $RESULT = FORMAT("user_stack%s", ARGV[1])$
$END$

$
$  基本タスクの共有スタックのセクションラベル名
$  ARGV[1]：共有スタックID(タスク優先度)
$
$FUNCTION SECTION_SHARED_USTACK$
	$RESULT = FORMAT("shared_user_stack%s", ARGV[1])$
$END$

$FUNCTION START_MO_SYMBOL$
    $IF !EQ(MO.PLABEL[ARGV[1]], "")$
        $RESULT = SYMBOL(CONCAT("__start_", MO.PLABEL[ARGV[1]]))$
    $ELIF !EQ(MO.MLABEL[ARGV[1]], "")$
        $RESULT = SYMBOL(CONCAT("__start_", MO.MLABEL[ARGV[1]]))$
    $ELIF MO.LINKER[ARGV[1]]$
        $ERROR$
            MO $ARGV[1]$ do not have PLABEL and MLABEL
        $END$
    $END$
$END$

$FUNCTION LIMIT_MO_SYMBOL$
    $IF !EQ(MO.PLABEL[ARGV[1]], "")$
        $RESULT = SYMBOL(CONCAT("__limit_", MO.PLABEL[ARGV[1]]))$
    $ELIF !EQ(MO.MLABEL[ARGV[1]], "")$
        $RESULT = SYMBOL(CONCAT("__limit_", MO.MLABEL[ARGV[1]]))$
    $ELIF MO.LINKER[ARGV[1]]$
        $ERROR$
            MO $ARGV[1]$ do not have PLABEL and MLABEL
        $END$
    $END$
$END$

$ 
$  OSAP初期化コンテキストブロックのための宣言
$ 
$FUNCTION PREPARE_OSAPINICTXB$
    $FILE "kernel_mem.c"$

$   // ユーザスタック領域のラベル 
    $IF LENGTH(TSK.ID_LIST)$
        $FOREACH tskid TSK.ID_LIST$
            $IF !OSAP.TRUSTED[TSK.OSAPID[tskid]]$
                extern uint8 $FORMAT("__start_user_stack%s", tskid)$;$NL$
                extern uint8 $FORMAT("__limit_user_stack%s", tskid)$;$NL$
            $END$
        $END$
    $END$
$   // 共有ユーザスタック領域のラベル 
    $FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
        $IF LENGTH(shared_ustack_size[tskpri])$
            extern uint8 $FORMAT("__start_shared_user_stack%s", tskpri)$;$NL$
            extern uint8 $FORMAT("__limit_shared_user_stack%s", tskpri)$;$NL$
        $END$
    $END$

    $IF LENGTH(OSAP.ID_LIST)$
        $FOREACH domid OSAP.ID_LIST$
            $IF !OSAP.TRUSTED[domid]$
$               // 非信頼OSAP専有領域のラベル 
                $IF USE_MPU_SRPW_DATA$
$               // 共有リード専有ライトデータ
                extern uint8 $FORMAT("__start_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                extern uint8 $FORMAT("__limit_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                $END$
                $IF USE_MPU_SRPW_SDATA$
$               // 共有リード専有ライトショートデータ
                extern uint8 $FORMAT("__start_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                extern uint8 $FORMAT("__limit_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                $END$
                $IF USE_MPU_PR_TEXT$
$               // RX領域（専有リードテキスト）
                extern uint8 __start_text_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_text_$OSAP.LABEL[domid]$;$NL$
                $END$
                $IF USE_MPU_PR_SDATA$
$               // R領域（専有リードショートデータ）
                extern uint8 __start_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
                extern uint8 __limit_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
                $END$
                $IF USE_MPU_PRW_DATA$
$               // RWX領域（専有リードライトデータ）
                extern uint8 __start_ram_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_ram_$OSAP.LABEL[domid]$;$NL$
                $END$
                $IF USE_MPU_PRW_SDATA$
$               // RWX領域（専有リードライトショートデータ）
                extern uint8 __start_sram_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_sram_$OSAP.LABEL[domid]$;$NL$
                $END$
            $END$
        $END$
        $NL$
    $END$$NL$
    $NL$
$   データセクション初期化テーブルをROMに配置する
$   rosdata領域はRAMに配置されるため，データセクション初期化テーブルをsdata化すると
$   データセクションの初期化に失敗する
    extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_datasec;$NL$
    extern const DATASECINIB __attribute__((section(".rodata_kernel"), aligned(4))) datasecinib_table[];$NL$
    extern const uint32 __attribute__((section(".rodata_kernel"), aligned(4))) tnum_bsssec;$NL$
    extern const BSSSECINIB __attribute__((section(".rodata_kernel"), aligned(4))) bsssecinib_table[];$NL$
$END$

$INCLUDE "arch/v850_gcc/prc_mem_common.tf"$


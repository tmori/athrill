$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2013-2016 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2013-2014 by FUJI SOFT INCORPORATED, JAPAN
$  Copyright (C) 2013-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2013-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2013-2014 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2013-2014 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2013-2014 by Witz Corporation, JAPAN
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
$  $Id: prc_common.tf 187 2015-06-25 03:39:04Z t_ishikawa $
$

$
$  有効な割込み番号
$
$ 
$ V850ではリセット，NMI，WDTは割込みに分類されるが，リセットは
$ カーネルが用いるため除外する

$EXCNO_VALID = {1,2,3,4,5,7}$

$TNUM_EXC = {1,2,3,4,5,6,7}$

$
$  CRE_ISR2で使用できる割込み番号
$
$INTNO_CREISR2_VALID = INTNO_VALID$

$FUNCTION EXTERN_INT_HANDLER$
$	コア間割込みハンドラ本体生成のextern宣言
	$FOREACH coreid RANGE(0, TMAX_COREID)$
		$FOREACH intno INTNO_ICI_LIST$
			$IF (intno & 0xffff0000) == ( (coreid+1) << 16)$
				extern ISR(target_ici_handler$coreid$);$NL$
			$END$
		$END$
	$END$
    $NL$
$END$

$ 
$  標準のセクションのメモリオブジェクト属性の定義
$ 
$MEMATR_TEXT = (TA_NOWRITE|TA_EXEC)$
$MEMATR_RODATA = (TA_NOWRITE|TA_EXEC)$
$MEMATR_DATA = TA_MEMINI$
$MEMATR_BSS = TA_NULL$
$MEMATR_PRSV = TA_MEMPRSV$
$MEMATR_ROSDATA = (TA_SDATA|TA_MEMINI|TA_NOWRITE)$
$MEMATR_SDATA = (TA_SDATA|TA_MEMINI)$
$MEMATR_SBSS = TA_SDATA$

$TARGET_MEMATR_USTACK = TA_NULL$

$ 
$  標準のセクションに関する定義
$  DESC.SECTION：セクション名
$  DESC.MEMREG：セクションを配置するメモリリージョン
$    1 => システム標準ROM
$    2 => システム標準RAM
$    3 => コア標準ROM
$    4 => コア標準RAM
$    5 => OSAP標準ROM
$    * => OSAP標準RAM
$  DESC.MEMATR：セクションのメモリオブジェクト属性
$  ※DSEC.ORDER_LIST：IDのリストが定義されていないターゲットでは，
$  　下記の定義は無効である（kernel.tfで無視される）
$ 
$  ショートデータはシステム標準RAMに配置する
$  そうでないと，ショートデータが各コアに配置されてしまい，
$  gp相対でアクセスできなくなる

$DSEC.SECTION[0] = ".text"$
$DSEC.MEMREG[0] = 3$
$DSEC.MEMATR[0] = MEMATR_TEXT$

$DSEC.SECTION[1] = ".rodata"$
$DSEC.MEMREG[1] = 3$
$DSEC.MEMATR[1] = MEMATR_RODATA$

$DSEC.SECTION[2] = ".data"$
$DSEC.MEMREG[2] = 4$
$DSEC.MEMATR[2] = MEMATR_DATA$

$DSEC.SECTION[3] = ".bss"$
$DSEC.MEMREG[3] = 4$
$DSEC.MEMATR[3] = MEMATR_BSS$

$DSEC.SECTION[4] = ".prsv"$
$DSEC.MEMREG[4] = 4$
$DSEC.MEMATR[4] = MEMATR_PRSV$

$DSEC.SECTION[5] = ".rosdata"$
$DSEC.MEMREG[5] = 2$
$DSEC.MEMATR[5] = MEMATR_ROSDATA$

$DSEC.SECTION[6] = ".sdata"$
$DSEC.MEMREG[6] = 2$
$DSEC.MEMATR[6] = MEMATR_SDATA$

$DSEC.SECTION[7] = ".sbss"$
$DSEC.MEMREG[7] = 2$
$DSEC.MEMATR[7] = MEMATR_SBSS$


$TNUM_MPU_OSAP_DEFAULT = 0$
$IF USE_MPU_SRPW_DATA$
$   // 共有リード専有ライトデータ
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$
$IF USE_MPU_SRPW_SDATA$
$   // 共有リード専有ライトショートデータ
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$
$IF USE_MPU_PR_TEXT$
$   // 専有リードROM
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$
$IF USE_MPU_PR_SDATA$
$   // 専有リードRAM
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$
$IF USE_MPU_PRW_DATA$
$   // 専有リードライトデータ
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$
$IF USE_MPU_PRW_SDATA$
$   // 共有リード専有ライトショートデータ
    $TNUM_MPU_OSAP_DEFAULT = TNUM_MPU_OSAP_DEFAULT + 1$
$END$

$TNUM_MPU_SHARED_DEFAULT = 0$
$IF USE_MPU_SRW_DATA_SDATA$
$   // 共有リードライト    
    $TNUM_MPU_SHARED_DEFAULT = TNUM_MPU_SHARED_DEFAULT + 1$
$END$
$IF USE_MPU_SR_TEXT$
$   // 共有リードROM
    $TNUM_MPU_SHARED_DEFAULT = TNUM_MPU_SHARED_DEFAULT + 1$
$END$
$IF USE_MPU_SR_DATA_SDATA$
$   // 共有リードRAM
    $TNUM_MPU_SHARED_DEFAULT = TNUM_MPU_SHARED_DEFAULT + 1$
$END$

$ 
$  MPUに設定する領域のアラインメントをチェックするための定数
$  TARGET_PAGE_SIZE_STR: target依存部で定義するMPU領域ページ単位(byte)
$ 
$MPU_PAGE_MASK = 0xffffffff - (TARGET_PAGE_SIZE_STR - 1)$

$ 
$  OSがデフォルトで使用するMPU数が最大値を超えていたらエラー
$  ターゲット依存部のtarget_config.hの USE_MPU_XXX の定義誤り
$ 
$IF TNUM_MPU_REG < (TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_SHARED_DEFAULT)$
    $ERROR$
        the number of MPU region used by ATK2 exceeds $TNUM_MPU_REG$
    $END$
$END$

$ 
$  OsMemoryArea 管理用データの初期化
$  ユーザスタック領域でMPU1つは必ず専有される
$ 
$TNUM_MPU_ATTMEM_REMAIN = TNUM_MPU_REG - (TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_SHARED_DEFAULT) - 1$
$TNUM_MPU_SHARED_ATTMEM = 0$
$TNUM_MPU_OSAP_ATTMEM = 0$

$TNUM_MPU_OSAP = TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_OSAP_ATTMEM$
$TNUM_MPU_SHARED = TNUM_MPU_SHARED_DEFAULT + TNUM_MPU_SHARED_ATTMEM$

$ 
$  OsMemoryArea のエラーチェック
$ 
$FUNCTION HOOK_ERRORCHECK_MEM$
    $IF TNUM_MPU_ATTMEM_REMAIN <= 0$
        $ERROR$
            the number of OsMemoryArea exceeds $TNUM_MPU_REG - (TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_SHARED_DEFAULT)$
        $END$
    $ELIF EQ(MEM.OSAPID[ARGV[1]], "")$
$       // 共有 ATTMEM
        $TNUM_MPU_SHARED_ATTMEM = TNUM_MPU_SHARED_ATTMEM + 1$
        $TNUM_MPU_ATTMEM_REMAIN = TNUM_MPU_ATTMEM_REMAIN - 1$
    $ELIF DEFAULT_ACPTN[MEM.OSAPID[ARGV[1]]] != TACP_KERNEL$
$       // 非信頼OSAP ATTMEM
        $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[MEM.OSAPID[ARGV[1]]])$
            $ENUM_MPU_OSAP_ATTMEM[MEM.OSAPID[ARGV[1]]] = ENUM_MPU_OSAP_ATTMEM[MEM.OSAPID[ARGV[1]]] + 1$
        $ELSE$
            $ENUM_MPU_OSAP_ATTMEM[MEM.OSAPID[ARGV[1]]] = 1$
        $END$
        $IF TNUM_MPU_OSAP_ATTMEM < ENUM_MPU_OSAP_ATTMEM[MEM.OSAPID[ARGV[1]]]$
            $TNUM_MPU_OSAP_ATTMEM = TNUM_MPU_OSAP_ATTMEM + 1$
            $TNUM_MPU_ATTMEM_REMAIN = TNUM_MPU_ATTMEM_REMAIN - 1$
        $END$
    $END$

    $TNUM_MPU_OSAP = TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_OSAP_ATTMEM$
    $TNUM_MPU_SHARED = TNUM_MPU_SHARED_DEFAULT + TNUM_MPU_SHARED_ATTMEM$
$END$

$FUNCTION GENERATE_TARGET_MPUINFOB$
$ 
$  保護ドメイン初期化ブロックの変更部を生成
$ 
    $FILE "kernel_mem2.c"$
    extern const uint32 tnum_shared_mem;$NL$
    extern uint8 * const shared_meminib_table[];$NL$
    $NL$

    $PREPARE_OSAPINICTXB()$

    $FOREACH osap OSAP.ID_LIST$
        $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[osap]) && 
            (ENUM_MPU_OSAP_ATTMEM[osap] > 0)$
            static MPUINFOB mpu_area_info_$osap$[$ENUM_MPU_OSAP_ATTMEM[osap]$] = {$NL$
            $FOREACH mpuinfo RANGE(1, ENUM_MPU_OSAP_ATTMEM[osap])$
                $TAB$$TAB${$NL$
                $TAB$$TAB$$TAB$0, 0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
                $TAB$$TAB$},$NL$
            $END$
            };$NL$
            $NL$
        $END$
    $END$
$END$

$FUNCTION GENERATE_OSAPINIB_MPUINFOB$
    $cur_osap_mpu_id = 1$

    $TAB$$TAB${$NL$
    $IF USE_MPU_SRPW_DATA$
$       // 共有リード専有ライトデータ
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_SRPW_SDATA$
$       // 共有リード専有ライトショートデータ
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_TEXT$
$       // 専有リードROM
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_SDATA$
$       // 専有リードRAM
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_DATA$
$       // 専有リードライトデータ
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_SDATA$
$       // 共有リード専有ライトショートデータ
        $TAB$$TAB$$TAB$0, 0, /* MPU$cur_osap_mpu_id$ */$NL$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[ARGV[1]]) &&
        (ENUM_MPU_OSAP_ATTMEM[ARGV[1]] > 0)$
        $TAB$$TAB$$TAB$mpu_area_info_$ARGV[1]$,$NL$
        $TAB$$TAB$$TAB$$ENUM_MPU_OSAP_ATTMEM[ARGV[1]]$U,$NL$
    $ELSE$
        $TAB$$TAB$$TAB$NULL,$NL$
        $TAB$$TAB$$TAB$0U,$NL$
    $END$
    $TAB$$TAB$$TAB$( (uint32)0 ),/* MPRC */$NL$
    $TAB$$TAB$}$NL$
    $NL$
$END$

$FUNCTION GENERATE_TSKINICTXB$
	$TAB$$TAB${$NL$
    $TAB$$TAB$$TAB$$TSK.TINIB_SSTKSZ[ARGV[1]]$,$NL$
	$TAB$$TAB$$TAB$((void *)((uint8 *)($TSK.TINIB_SSTK[ARGV[1]]$)
    $SPC$+ ($TSK.TINIB_SSTKSZ[ARGV[1]]$))),$NL$
	$IF OSAP.TRUSTED[TSK.OSAPID[ARGV[1]]]$
        $TAB$$TAB$$TAB$0,$NL$
        $TAB$$TAB$$TAB$0,$NL$
    $ELSE$
        $TAB$$TAB$$TAB$$TSK.TINIB_USTKSZ[ARGV[1]]$,$NL$ 
        $TAB$$TAB$$TAB$((void *)((uint8 *)($TSK.TINIB_USTK[ARGV[1]]$)
        $SPC$+ ($TSK.TINIB_USTKSZ[ARGV[1]]$))),$NL$
    $END$
	$TAB$$TAB$},$NL$
$END$

$FUNCTION GENERATE_STKMPUINFOB$
	$TAB$$TAB${$NL$
	$IF OSAP.TRUSTED[TSK.OSAPID[ARGV[1]]] || (tmin_os_restarttask <= ARGV[1])$
        $TAB$$TAB$$TAB$0,$NL$
        $TAB$$TAB$$TAB$0,$NL$
    $ELSE$
		$IF EQ(TSK.STK[ARGV[1]],"NULL")$
$			// stkがNULLの場合の処理
            $IF LENGTH(TSK.SHARED_USTK_ID[ARGV[1]])$
$               // 共有スタック
                $TAB$$TAB$$TAB$/*dummy at pass2*/$NL$
                $TAB$$TAB$$TAB$0,$NL$
                $TAB$$TAB$$TAB$0,$NL$
            $ELSE$
$               // 固有スタック
                $TAB$$TAB$$TAB$/*dummy at pass2*/$NL$
                $TAB$$TAB$$TAB$0,$NL$
                $TAB$$TAB$$TAB$0,$NL$
            $END$
        $ELSE$
$			// stkがNULLでない場合の処理
            $TAB$$TAB$$TAB$$FORMAT("(uint8 *)%s", TSK.TINIB_USTK[ARGV[1]])$,$NL$
            $TAB$$TAB$$TAB$$FORMAT("(uint8 *)((uint32)%s + %d)", TSK.TINIB_USTK[ARGV[1]], TSK.TINIB_USTKSZ[ARGV[1]])$,$NL$
        $END$
    $END$
	$TAB$$TAB$},$NL$
$END$


$
$  標準テンプレートファイルのインクルード
$
$INCLUDE "kernel/kernel.tf"$

$FOREACH intno INTNO_VALID$
	$FOREACH isrid ISR.ID_LIST$
		$IF intno == ISR.INTNO[isrid]$
			$INT.ISRID[intno] = isrid$
		$END$
	$END$
$END$


$FILE "Os_Lcfg.c"$

$
$ コア間割込みハンドラ本体生成
$
$FOREACH coreid RANGE(0, TMAX_COREID)$
	$FOREACH intno INTNO_ICI_LIST$
		$IF (intno & 0xffff0000) == ( (coreid+1) << 16)$
			void$NL$
			$CONCAT("_kernel_inthdr_", FORMAT("0x%x",+intno))$(void)$NL$
			{$NL$
			$TAB$i_begin_int($+intno$U);$NL$
			$TAB$ISRNAME(target_ici_handler$coreid$)();$NL$
			$TAB$i_end_int($+intno$U);$NL$
			}$NL$
		$END$
	$END$
$END$

$
$ C2ISRの優先度下限
$
$ ICIの優先度を考慮する必要があるため,MAX_PRI_ISR1を使用する
$min_pri_isr2_system = MAX_PRI_ISR1 + 1$

$n = 0$
$pmr_isr2_mask = 0xffff$
$WHILE (n < (min_pri_isr2_system + TNUM_INTPRI))$
$pmr_isr2_mask = pmr_isr2_mask & ~(0x01 << n)$
$n = n + 1$
$END$
$pmr_isr1_mask = ~pmr_isr2_mask & 0xffff $

const uint16 pmr_isr2_mask = $FORMAT("0x%x",pmr_isr2_mask)$;$NL$
const uint16 pmr_isr1_mask = $FORMAT("0x%x",pmr_isr1_mask)$;$NL$

$
$  割込みハンドラテーブル(EIレベル　マスカブル割込み用)
$
$FOREACH coreid RANGE(0, TMAX_COREID)$
    const FunctionRefType core$coreid$_isr_tbl[TNUM_INT] = {$NL$
    $FOREACH intno INTNO_VALID$
        $IF (((intno & 0xffff0000) == ((coreid+1) << 16)) || ((intno & 0xffff0000) == 0xffff0000))$
            $isrid = INT.ISRID[intno]$
            $IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_2")  && (OSAP.CORE[ISR.OSAPID[isrid]] == coreid)$
                $TAB$&$ISR.INT_ENTRY[isrid]$
            $ELIF LENGTH(FIND(INTNO_ICI_LIST, intno))$
$			    //コア間割割込みハンドラ
                $TAB$&$CONCAT("_kernel_inthdr_", FORMAT("0x%x",+intno))$
            $ELSE$
                $TAB$&default_int_handler
            $END$
$		    //カンマの出力（最後の要素の後ろに出力しない）
            $IF (intno & 0xffff) < TMAX_INTNO$
                ,
            $END$
            $TAB$$FORMAT("/* 0x%x */", +intno)$$NL$
        $END$
    $END$
    };$NL$
    $NL$
$END$

const uint32 isr_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(const uint32) core$coreid$_isr_tbl
$END$
$NL$};$NL$
$NL$

$
$  ISRCBの取得テーブル
$
$FOREACH coreid RANGE(0, TMAX_COREID)$
ISRCB * const core$coreid$_isr_p_isrcb_tbl[TNUM_INT] = {$NL$
$FOREACH intno INTNO_VALID$
	$IF ((intno & 0xffff0000) == ((coreid+1) << 16)) || ((intno & 0xffff0000) == 0xffff0000) $
		$isrid = INT.ISRID[intno]$
		$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_2") && (OSAP.CORE[ISR.OSAPID[isrid]] == coreid)$
			$TAB$&_kernel_isrcb_$isrid$
		$ELSE$
			$TAB$NULL
		$END$
$		//カンマの出力（最後の要素の後ろに出力しない）
		$IF (intno & 0xffff) < TMAX_INTNO$
			,
		$END$
		$TAB$$FORMAT("/* 0x%x */", +intno)$$NL$
	$END$
$END$
};$NL$
$NL$
$END$

const uint32 isr_p_isrcb_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(const uint32) core$coreid$_isr_p_isrcb_tbl
$END$
$NL$};$NL$
$NL$

$
$ 割込みベクタと各割込み入口処理の出力関数
$

$FUNCTION VECTOR_ASMOUT$

$EXCEPTION_VECTOR_SECTION()$

$ASM_GLOBAL("__reset")$
$NL$
$ASM_LABEL("__reset")$
$NL$
$TAB$$ASM_COMMENT()$
$FORMAT(" Exception 0x%x ", 0)$$NL$
$TAB$jr __start$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$
$ ベクタテーブル(例外 No1-7)
$
$ 例外 No1
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 1)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_EXCEPTION$, r0, r2$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No2
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 2)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_EXCEPTION$, r0, r2$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No3
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 3)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_MEMORY$, r0, r2$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No4
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 4)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_EXCEPTION$, r0, r2$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No5
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 5)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_EXCEPTION$, r0, r2$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No6
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 6)$$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$ 例外 No7
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 7)$$NL$
$TAB$ldsr  r2, eiwr$NL$
$TAB$movea $E_OS_PROTECTION_EXCEPTION$, r0, r2$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$
$ ベクタテーブル(EIレベル マスカブル割込み用(8-255)
$
$NL$$FOREACH intno INTNO_VALID$
$TAB$$ASM_COMMENT()$$FORMAT("0x%x",intno*16 + 0x80)$ $NL$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid)$
		$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
            $TAB$ldsr  r2, eiwr$NL$
            $TAB$movea $intno$, r0, r2$NL$ 
            $TAB$jr    _interrupt$NL$
            $TAB$nop$NL$
            $TAB$nop$NL$
		$ELSE$
			$TAB$jr _$ISR.INT_ENTRY[isrid]$$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
		$END$
	$ELSE$
$		// 割込みハンドラの登録がない場合
		$TAB$jr _default_int_handler$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$END$
$END$

$END$

$ 
$  MPU使用枚数の定義出力
$ 
$FILE "target_Os_Cfg.h"$
#ifndef TARGET_OS_CFG_H
#define TARGET_OS_CFG_H

#define TNUM_MPU_OSAP_DEFAULT ($TNUM_MPU_OSAP_DEFAULT$)$NL$
#define TNUM_MPU_OSAP_ATTMEM ($TNUM_MPU_OSAP_ATTMEM$)$NL$
#define TNUM_MPU_OSAP ($TNUM_MPU_OSAP_DEFAULT + TNUM_MPU_OSAP_ATTMEM$)$NL$
#define TNUM_MPU_SHARED_DEFAULT ($TNUM_MPU_SHARED_DEFAULT$)$NL$
#define TNUM_MPU_SHARED_ATTMEM ($TNUM_MPU_SHARED_ATTMEM$)$NL$
#define TNUM_MPU_SHARED ($TNUM_MPU_SHARED_DEFAULT + TNUM_MPU_SHARED_ATTMEM$)$NL$

#endif /* TARGET_OS_CFG_H */

$ 
$  MPU設定処理の生成
$ 
$IF __v850e3v5__$
$FILE "Os_Cfg_asm.inc"$
$ASM_MACRO("DISPATCHER_MPU_SETTING", "reg1, reg2, reg3, reg4, reg5, reg6")$$NL$
$IF TNUM_MPU_OSAP > 0$
$ 
$   // ディスパッチャにおけるMPU設定処理の生成
$ 
    $cur_osap_mpu_id = 1$
    $sysreg = 6*32 + 4$
    $IF USE_MPU_SRPW_DATA$
$       // 共有リード専有ライトデータ
$         $TAB$*$NL$
$         $TAB$* 共有リード/専用ライト領域の専用領域$NL$
$         $TAB$*$NL$
        $TAB$ld.w   OSAPINIB_start_srpw[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_srpw[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_SRPW_SDATA$
$       // 共有リード専有ライトショートデータ
$         $TAB$;$NL$
$         $TAB$; 共有リード/専用ライト領域の専用領域(sdata)$NL$
$         $TAB$;$NL$
        $TAB$ld.w   OSAPINIB_start_ssrpw[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_ssrpw[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_TEXT$
$       // 専有リードROM
$         $TAB$;$NL$
$         $TAB$; 自保護ドメイン専用のrom領域$NL$
$         $TAB$;$NL$
        $TAB$ld.w   OSAPINIB_start_text[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_text[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_SDATA$
$       // 専有リードRAM
$         $TAB$;$NL$
$         $TAB$; 自保護ドメイン専用のrosdata領域$NL$
$         $TAB$;$NL$
        $TAB$ld.w   OSAPINIB_start_rosdata[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_rosdata[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_DATA$
$       // 専有リードライトデータ
$         $TAB$;$NL$
$         $TAB$; 自保護ドメイン専用のRWX領域$NL$
$         $TAB$;$NL$
        $TAB$ld.w   OSAPINIB_start_ram[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_ram[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_SDATA$
$       // 共有リード専有ライトショートデータ
$         $TAB$;$NL$
$         $TAB$; 自保護ドメイン専用のRWX領域(sdata)$NL$
$         $TAB$;$NL$
        $TAB$ld.w   OSAPINIB_start_sram[reg1], reg2$NL$
        $TAB$ld.w   OSAPINIB_limit_sram[reg1], reg3$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 3$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $TAB$ld.w   OSAPINIB_mprc[reg1], reg2$NL$
    $TAB$ldsr   reg2, 1, 5$NL$
    $IF TNUM_MPU_OSAP_ATTMEM$
        $TAB$mov    r0, reg6$NL$
        $TAB$ld.b   OSAPINIB_tnum_mpu_area[reg1], reg5$NL$
        $TAB$cmp    reg5, reg6$NL$
        $TAB$be     exit_mpu_setting$NL$
        $TAB$ld.w   OSAPINIB_mpu_area_info[reg1], reg1$NL$
        $FOREACH mpuid RANGE(cur_osap_mpu_id, TNUM_MPU_OSAP - 1)$
$         $TAB$/*$NL$
$         $TAB$* 自保護ドメイン専用のATT_MEM領域(sdata)$NL$
$         $TAB$*/$NL$
            $TAB$ld.w   0[reg1], reg2$NL$
            $TAB$ld.w   4[reg1], reg3$NL$
            $TAB$ld.w   8[reg1], reg4$NL$
            $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
            $sysreg = sysreg + 1$
            $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
            $sysreg = sysreg + 1$
            $TAB$ldsr   reg4, $sysreg % 32$, $sysreg / 32$$NL$
            $sysreg = sysreg + 2$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
            $TAB$addi   1, reg6, reg6$NL$
            $TAB$cmp    reg5, reg6$NL$
            $TAB$be     exit_mpu_setting$NL$
            $TAB$addi   12, reg1, reg1$NL$
        $END$
        $TAB$ld.w   0[reg1], reg2$NL$
        $TAB$ld.w   4[reg1], reg3$NL$
        $TAB$ld.w   8[reg1], reg4$NL$
        $TAB$ldsr   reg2, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg3, $sysreg % 32$, $sysreg / 32$$NL$
        $sysreg = sysreg + 1$
        $TAB$ldsr   reg4, $sysreg % 32$, $sysreg / 32$$NL$

        $NL$
        exit_mpu_setting:$NL$
    $END$
$ELSE$
    $NL$
$END$
.endm$NL$

$ 
$ // カーネル初期化におけるMPU設定処理の生成
$ 
$FILE "Os_Lcfg.c"$
$IF TNUM_MPU_SHARED > 0$
extern uint8* shared_meminib_table[];$NL$
$NL$
$END$

/*LOCAL_INLINE */void$NL$
mpu_shared_area_initialize(void)$NL$
{$NL$
$IF TNUM_MPU_SHARED > 0$
$ 
$   // 共有領域のMPU設定処理の生成
$ 
    $sysreg = 6*32 + (TNUM_MPU_REG - 1)*4$
    $TAB$uint32 mpur;$NL$
    $NL$
    $FOREACH memid RANGE(0, TNUM_MPU_SHARED - 1)$
        $TAB$/*$NL$
        $TAB$ * MPU$TNUM_MPU_REG - 1 - memid$$NL$
        $TAB$ */$NL$
        $TAB$mpur = (uint32)shared_meminib_table[$memid*3$];$NL$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)mpur);$NL$
        $TAB$mpur = (uint32)shared_meminib_table[$memid*3 + 1$];$NL$
        $TAB$LDSR_REG($(sysreg + 1) % 32$, $(sysreg + 1) / 32$, (uint32)mpur);$NL$
        $TAB$mpur = (uint32)shared_meminib_table[$memid*3 + 2$];$NL$
        $TAB$LDSR_REG($(sysreg + 2) % 32$, $(sysreg + 2) / 32$, (uint32)mpur);$NL$
        $sysreg = sysreg - 4$
    $END$
$END$
$IF TNUM_MPU_OSAP > 0$
$ 
$   // 非信頼OSAP専有MPU領域のMPAT設定処理の生成
$ 
    $cur_osap_mpu_id = 1$
    $sysreg = 6*32 + 4$
    $IF USE_MPU_SRPW_DATA$
$       // 共有リード専有ライトデータ
        $TAB$/*$NL$
        $TAB$ * 共有リード/専用ライト領域の専用領域$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_SRPW_SDATA$
$       // 共有リード専有ライトショートデータ
        $TAB$/*$NL$
        $TAB$ * 共有リード/専用ライト領域の専用領域(sdata)$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_TEXT$
$       // 専有リードROM
        $TAB$/*$NL$
        $TAB$ * 自保護ドメイン専用のrom領域$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PR_SDATA$
$       // 専有リードRAM
        $TAB$/*$NL$
        $TAB$ * 自保護ドメイン専用のrosdata領域$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_DATA$
$       // 専有リードライトデータ
        $TAB$/*$NL$
        $TAB$ * 自保護ドメイン専用のRWX領域$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
    $IF USE_MPU_PRW_SDATA$
$       // 専有リードライトショートデータ
        $TAB$/*$NL$
        $TAB$ * 自保護ドメイン専用のRWX領域(sdata)$NL$
        $TAB$ * MPU$cur_osap_mpu_id$$NL$
        $TAB$ */$NL$
        $sysreg = sysreg + 2$
        $TAB$LDSR_REG($sysreg % 32$, $sysreg / 32$, (uint32)(MPAT_E | MPAT_G | MPAT_UX | MPAT_UW | MPAT_UR))$NL$
        $sysreg = sysreg + 2$
        $cur_osap_mpu_id = cur_osap_mpu_id + 1$
    $END$
$END$
}$NL$

$END$

$FILE "cfg2_out.tf"$

$FOREACH osap OSAP.ID_LIST$
    $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[osap]) && 
        (ENUM_MPU_OSAP_ATTMEM[osap] > 0)$
        $$ENUM_MPU_OSAP_ATTMEM[$+osap$] = $ENUM_MPU_OSAP_ATTMEM[osap]$$$$NL$
    $END$
$END$

$$TNUM_MPU_OSAP_DEFAULT = $TNUM_MPU_OSAP_DEFAULT$$$$NL$
$$TNUM_MPU_OSAP = $TNUM_MPU_OSAP$$$$NL$
$$TNUM_MPU_SHARED = $TNUM_MPU_SHARED$$$$NL$
$$MPU_PAGE_MASK = $MPU_PAGE_MASK$$$$NL$
$NL$

$FILE "Os_Lcfg.c"$


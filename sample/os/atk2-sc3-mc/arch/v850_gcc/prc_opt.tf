$ ======================================================================
$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
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

$DEBUG_OPT_TF = 1$

$ 
$  arch/v850_gcc/ldscript.tfのターゲット依存部
$ 
$FUNCTION GENERATE_MEMORY$
    $NOOP()$
$END$

$FUNCTION GENERATE_OUTPUT$
    OUTPUT_FORMAT("elf32-v850","elf32-v850","elf32-v850")$NL$
    OUTPUT_ARCH(v850)$NL$
    $NL$
$END$

$
$  保護ドメインのラベルの作成
$	OSAP.LABEL[domid]：保護ドメインのラベル
$
$OSAP.LABEL[TDOM_KERNEL] = "kernel"$
$FOREACH domid OSAP.ID_LIST$
	$OSAP.LABEL[domid] = domid$
$END$
$OSAP.LABEL[TDOM_NONE] = "shared"$

$ 
$  標準のセクションのメモリオブジェクト属性の定義
$ 
$MEMATR_TEXT = (TA_NOWRITE|TA_EXEC)$
$MEMATR_RODATA = (TA_NOWRITE|TA_EXEC)$
$MEMATR_DATA = TA_MEMINI$
$MEMATR_BSS = TA_NULL$
$MEMATR_PRSV = TA_MEMPRSV$
$MEMATR_ROSDATA = (TA_SDATA|TA_MEMINI|TA_NOWRITE|TA_EXEC)$
$MEMATR_SDATA = (TA_SDATA|TA_MEMINI)$
$MEMATR_SBSS = TA_SDATA$


$ 
$  保護ドメイン毎のデフォルトのアクセス許可パターンの作成
$ 
$DEFAULT_ACPTN[TDOM_KERNEL] = VALUE("TACP_KERNEL", TACP_KERNEL)$
$FOREACH domid OSAP.ID_LIST$
	$DEFAULT_ACPTN[domid] = VALUE(FORMAT("TACP(%1%)", domid), 1 << (domid - 1))$
$END$
$DEFAULT_ACPTN[TDOM_NONE] = VALUE("TACP_SHARED", TACP_SHARED)$

$ 
$  メモリ保護単位の実際のサイズを受け取り，受け取ったサイズ以上で，
$  ARM MPUのサイズ制約を満たす最小値を返す
$  ARM MPUのサイズ制約は，16B~4GBの範囲の2のべき乗
$  ARGV[1]：実際のサイズ
$  RESULT：ARM MPUのサイズ制約を満たす最小値
$FUNCTION SEARCH_ARM_MPU_SIZE$
    $IF ARGV[1] <= 0$
        $RESULT = 0$
    $ELSE$
        $search_state = 0$
        $FOREACH bit_offset { 31,30,...,4 }$
            $compare_bit = 1 << bit_offset$
            $IF ((ARGV[1] & compare_bit) != 0)$
                $IF (search_state == 0)$
                    $result_size = compare_bit$
                    $search_state = 1$
                $ELIF (search_state == 1)$
                    $result_size = (result_size << 1)$
                    $search_state = 2$
                $END$
            $END$
        $END$
        $IF (search_state == 0)$
            $result_size = (1 << 4)$
        $ELIF (search_state == 1) && ((ARGV[1] & 0xf) != 0)$
            $result_size = (result_size << 1)$
        $END$
        $RESULT = result_size$
    $END$
$END$

$ 
$  受け取った番地以上で，ARM MPUのアライン制約の最小値を返す
$  ARGV[1]：番地
$  ARGV[2]：サイズ
$  RESULT：ARM MPUのアライン制約を満たす最小値
$ 
$FUNCTION SEARCH_ARM_MPU_ALIGN$
    $result_align = ARGV[1]$
    $compare_mask = ARGV[2] - 1$
    $IF ((ARGV[1] & compare_mask) != 0)$
        $result_align = ARGV[1] & ~compare_mask$
        $result_align = result_align + ARGV[2]$
    $END$

    $RESULT = result_align$
$END$

$ 
$  tsize_meminibの補正
$  sdata_sharedの先頭アドレスをマスクベース方式でアラインするため，
$  その直前のセクションのlimitとsdata_sharedのstartが異なる可能性がある
$  よって+1しておく
$  prsv_sharedの終端アドレスをマスクベース方式でアラインするため，
$  prsv_sharedのサイズが空でも，パス4ではstartとlimitが異なる可能性がある
$  よって+1しておく
$  
$FUNCTION CALC_TSIZE_MEMINIB$
    $tsize_meminib = tsize_meminib + 2$
$END$

$OPTIMIZE_MEMINIB = 1$
$OPTIMIZE_DATASEC_LIST = 1$
$OPTIMIZE_BSSSEC_LIST = 1$

$ 
$  OSAP初期化コンテキストブロックのための宣言
$ 
$FUNCTION PREPARE_OSAPINICTXB$
    $IF LENGTH(TSK.ID_LIST)$
        $FOREACH tskid TSK.ID_LIST$
            $IF !OSAP.TRUSTED[TSK.OSAPID[tskid]]$
                extern uint8 $FORMAT("__start_user_stack%s", tskid)$;$NL$
                extern uint8 $FORMAT("__limit_user_stack%s", tskid)$;$NL$
            $END$
        $END$
    $END$
    $FOREACH tskpri RANGE(TMIN_TPRI, TMAX_TPRI)$
        $IF LENGTH(shared_ustack_size[tskpri])$
            extern uint8 $FORMAT("__start_shared_user_stack%s", tskpri)$;$NL$
            extern uint8 $FORMAT("__limit_shared_user_stack%s", tskpri)$;$NL$
        $END$
    $END$


    $IF LENGTH(OSAP.ID_LIST)$
        $FOREACH domid OSAP.ID_LIST$
            $IF !OSAP.TRUSTED[domid]$
$   RX領域（専用）
                extern uint8 __start_text_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_text_$OSAP.LABEL[domid]$;$NL$
$   R領域（専用）
                extern uint8 __start_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
                extern uint8 __limit_sram_$OSAP.LABEL[domid]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
$   RWX領域（専用）
                extern uint8 __start_ram_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_ram_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __start_sram_$OSAP.LABEL[domid]$;$NL$
                extern uint8 __limit_sram_$OSAP.LABEL[domid]$;$NL$
$   共有リード専用ライト
                extern uint8 $FORMAT("__start_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                extern uint8 $FORMAT("__limit_ram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                extern uint8 $FORMAT("__start_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
                extern uint8 $FORMAT("__limit_sram_%s_%x_%x", OSAP.LABEL[domid], +DEFAULT_ACPTN[domid], +TACP_SHARED)$;$NL$
            $END$
        $END$
        $NL$
    $END$$NL$

$  共有領域
    extern uint8 __start_text_$OSAP.LABEL[TDOM_NONE]$;$NL$
    extern uint8 __limit_text_$OSAP.LABEL[TDOM_NONE]$;$NL$
    extern uint8 __start_sram_$OSAP.LABEL[TDOM_NONE]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
    extern uint8 __limit_sram_$OSAP.LABEL[TDOM_NONE]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$;$NL$
    extern uint8 __start_ram_$OSAP.LABEL[TDOM_NONE]$;$NL$
    extern uint8 __limit_ram_$OSAP.LABEL[TDOM_NONE]$;$NL$
    extern uint8 __start_sram_$OSAP.LABEL[TDOM_NONE]$;$NL$
    extern uint8 __limit_sram_$OSAP.LABEL[TDOM_NONE]$;$NL$
$   共有リード専用ライト領域全体
    extern uint8 __start_srpw_all;$NL$
    extern uint8 __limit_srpw_all;$NL$
    extern uint8 __start_ssrpw_all;$NL$
    extern uint8 __limit_ssrpw_all;$NL$
    $NL$
$END$


$FUNCTION GENERATE_TARGET_MPUINFOB$
$ 
$  保護ドメイン初期化ブロックのmem依存部を生成
$ 
    $FILE "kernel_mem3.c"$

    $PREPARE_OSAPINICTXB()$

$END$

$ 
$  TSKINICTXBの初期化情報を生成
$ 
$DOMINICTXB_KERNEL = "{ NULL }"$

$FUNCTION GENERATE_OSAPINIB_MPUINFOB$
    $TAB$$TAB${$NL$
    $IF !OSAP.TRUSTED[ARGV[1]]$
$   RX領域（専用）
        $TAB$$TAB$$TAB$( (uint8 *)&__start_text_$OSAP.LABEL[ARGV[1]]$ ), /* iregion 0 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)&__limit_text_$OSAP.LABEL[ARGV[1]]$ ), /* iregion 0 */$NL$
$   RX領域（専用ショートデータ）
        $TAB$$TAB$$TAB$( (uint8 *)&__start_sram_$OSAP.LABEL[ARGV[1]]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ ),/* iregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)&__limit_sram_$OSAP.LABEL[ARGV[1]]$_$FORMAT("%x", MEMATR_ROSDATA & ~TA_MEMINI)$ ),/* iregion 1 */$NL$
$   RWX領域（専用）
        $TAB$$TAB$$TAB$( (uint8 *)&__start_ram_$OSAP.LABEL[ARGV[1]]$ ),/* dregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)&__limit_ram_$OSAP.LABEL[ARGV[1]]$ ),/* dregion 1 */$NL$
$   RWX領域（専用ショートデータ）
        $TAB$$TAB$$TAB$( (uint8 *)&__start_sram_$OSAP.LABEL[ARGV[1]]$ ),/* dregion 2 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)&__limit_sram_$OSAP.LABEL[ARGV[1]]$ ),/* dregion 2 */$NL$
$   共有リード専用ライト
        $TAB$$TAB$$TAB$$FORMAT("( (uint8 *)&__start_ram_%s_%x_%x )", OSAP.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,/* dregion 3 */$NL$
        $TAB$$TAB$$TAB$$FORMAT("( (uint8 *)&__limit_ram_%s_%x_%x )", OSAP.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,/* dregion 3 */$NL$
$   共有リード専用ライト（ショートデータ）
        $TAB$$TAB$$TAB$$FORMAT("( (uint8 *)&__start_sram_%s_%x_%x )", OSAP.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,/* dregion 4 */$NL$
        $TAB$$TAB$$TAB$$FORMAT("( (uint8 *)&__limit_sram_%s_%x_%x )", OSAP.LABEL[ARGV[1]], +DEFAULT_ACPTN[ARGV[1]], +TACP_SHARED)$,/* dregion 4 */$NL$
    $ELSE$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* iregion 0 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* iregion 0 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* iregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* iregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 1 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 2 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 2 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 3 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 3 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 4 */$NL$
        $TAB$$TAB$$TAB$( (uint8 *)NULL ),/* dregion 4 */$NL$
    $END$
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
	$IF OSAP.TRUSTED[TSK.OSAPID[ARGV[1]]]$
        $TAB$$TAB$$TAB$0,$NL$
        $TAB$$TAB$$TAB$0,$NL$
    $ELSE$
		$IF EQ(TSK.STK[tskid],"NULL")$
$			// stkがNULLの場合の処理
            $IF LENGTH(TSK.SHARED_USTK_ID[tskid])$
$               // 共有スタック
                $TAB$$TAB$$TAB$$FORMAT("&__start_shared_user_stack%s", TSK.PRIORITY[ARGV[1]])$,$NL$
                $TAB$$TAB$$TAB$$FORMAT("(&__limit_shared_user_stack%s - 0x10)", TSK.PRIORITY[ARGV[1]])$,$NL$
            $ELSE$
$               // 固有スタック
                $TAB$$TAB$$TAB$$FORMAT("&__start_user_stack%s", ARGV[1])$,$NL$
                $TAB$$TAB$$TAB$$FORMAT("(&__limit_user_stack%s - 0x10)", ARGV[1])$,$NL$
            $END$
        $ELSE$
$			// stkがNULLでない場合の処理
            $TAB$$TAB$$TAB$$FORMAT("(uint8 *)%s", TSK.TINIB_USTK[ARGV[1]])$,$NL$
            $TAB$$TAB$$TAB$$FORMAT("(uint8 *)((uint32)%s + %d - 0x10)", TSK.TINIB_USTK[ARGV[1]], TSK.TINIB_USTKSZ[ARGV[1]])$,$NL$
        $END$
    $END$
	$TAB$$TAB$},$NL$
$END$

$INCLUDE "kernel/kernel_opt.tf"$

$ 
$  共有リードライト領域 / 共有リード専有ライト領域（sdata） / rosdata_shared領域
$  の情報取得
$  
$check_shared_mo = 0x00$
$check_srpw_mo = 0x00$
$check_rosdata_mo = 0x00$
$check_sdata_mo = 0x00$
$LIST_SHARED_MO = {}$
$LIST_SRPW_MO = {}$
$LIST_ROSDATA_MO = {}$
$preid = -1$
$FOREACH moid MO_START_LIST$
$   // メモリオブジェクトの先頭をパス2時点での順にチェック
    $IF LENGTH(FIND(MO_MPROTECT_LIST, moid))$
$       // メモリ保護単位の先頭の場合
        $IF (MO.ACPTN1[moid] == TACP_SHARED) && (MO.ACPTN2[moid] == TACP_SHARED)$
            $IF ((check_shared_mo & 0x10) != 0x10)$
$               // 共有リードライト領域の先頭の場合
                $SHARED_AREA_BASE[check_shared_mo] = MO.BASEADDR[moid]$
                $SHARED_AREA_MO[check_shared_mo] = moid$
                $LIST_SHARED_MO = APPEND(LIST_SHARED_MO, check_shared_mo)$
                $check_shared_mo = check_shared_mo | 0x10$
            $END$
        $ELIF (check_shared_mo & 0x10) == 0x10$
$           // 共有リードライト領域の終端の場合
            $IF preid == -1$
                $ERROR$
                    $FORMAT("unexpected preid.")$
                $END$
            $END$
            $check_shared_mo = check_shared_mo & 0x0f$
            $SHARED_AREA_LIMIT[check_shared_mo] = MO.LIMITADDR[preid]$
            $SHARED_AREA_SIZE[check_shared_mo] = SHARED_AREA_LIMIT[check_shared_mo] - SHARED_AREA_BASE[check_shared_mo]$
            $SHARED_AREA_LIMIT_MO[check_shared_mo] = preid$
            $check_shared_mo = check_shared_mo + 1$
        $END$

    $END$

    $preid = moid$
$END$
$IF (check_shared_mo & 0x10) == 0x10$
    $IF preid == -1$
        $ERROR$
            $FORMAT("unexpected preid.")$
        $END$
    $END$
    $check_shared_mo = check_shared_mo & 0x0f$
    $SHARED_AREA_LIMIT[check_shared_mo] = MO.LIMITADDR[preid]$
    $SHARED_AREA_LIMIT_MO[check_shared_mo] = preid$
    $SHARED_AREA_SIZE[check_shared_mo] = SHARED_AREA_LIMIT[check_shared_mo] - SHARED_AREA_BASE[check_shared_mo]$
    $check_shared_mo = check_shared_mo + 1$
$END$
$ 
$  エラーチェック
$  
$FOREACH id LIST_SHARED_MO$
    $IF DEBUG_OPT_TF$
        $WARNING$
            $id$$NL$
            $SHARED_AREA_MO[id]$$NL$
            $SHARED_AREA_LIMIT_MO[id]$$NL$
            $FORMAT("0x%x", SHARED_AREA_BASE[id])$$NL$
            $FORMAT("0x%x", SHARED_AREA_LIMIT[id])$$NL$
            $FORMAT("0x%x", SHARED_AREA_SIZE[id])$$NL$
        $END$
    $END$
    $IF ((MO.MEMATR[SHARED_AREA_MO[id]] & TA_SDATA) == TA_SDATA)$
        $IF LENGTH(SHARED_DATA.BASE)$
            $ERROR$
                $FORMAT("unexpected mematr: %d, %x", id, MO.MEMATR[SHARED_AREA_MO[id]])$
            $END$
        $ELSE$
            $SHARED_DATA.BASE = SHARED_AREA_BASE[id]$
            $SHARED_DATA.LIMIT = SHARED_AREA_LIMIT[id]$
            $SHARED_DATA.SIZE = SHARED_AREA_SIZE[id]$
            $SHARED_DATA.BASE_MO = SHARED_AREA_MO[id]$
            $SHARED_DATA.LIMIT_MO = SHARED_AREA_LIMIT_MO[id]$
        $END$
    $ELSE$
        $ERROR$
            $FORMAT("unexpected mematr: %d, %x", id, MO.MEMATR[SHARED_AREA_MO[id]])$
        $END$
    $END$
$END$

$ 
$  data_sharedの配置アドレスを求める
$ 
$SHARED_DATA.REALSIZE = SEARCH_ARM_MPU_SIZE(SHARED_DATA.SIZE)$
$SHARED_DATA.REALBASE = SEARCH_ARM_MPU_ALIGN(SHARED_DATA.BASE, SHARED_DATA.REALSIZE)$
$IF DEBUG_OPT_TF$
    $WARNING$
        $FORMAT("data_shared: base[0x%x], size[0x%x]", SHARED_DATA.REALBASE, SHARED_DATA.REALSIZE)$$NL$
    $END$
$END$

$  
$  配置アドレスを指定するセクションに対する処理
$  配置換え対象のセクション: 必ずあるかどうか
$    srpw_data_all: 0
$    srpw_sdata_all: 0
$    rosdata_shared: 1
$    sdata_shared: 1
$    data_shared: 1
$  
$check_shared_mo = 0$
$preid = -1$
$limit_align_flag = 1$
$FOREACH moid MO_START_LIST$
$   // メモリオブジェクトの先頭をパス2時点での順にチェック
    $IF MO.LINKER[moid]$
        $IF DEBUG_OPT_TF$
            $WARNING$
                $FORMAT("check mo %d: SEFLAG=0x%x", moid, MO.SEFLAG[moid])$$NL$
            $END$
        $END$
        $IF (moid == SHARED_DATA.BASE_MO)$
$           // data_sharedの先頭
            $MO.REALBASE[moid] = SHARED_DATA.REALBASE$
            $MO.COMMENT[moid] = "data_shared_top"$
            $IF DEBUG_OPT_TF$
                $WARNING$
                    $FORMAT("%s[%d]=0x%x", MO.COMMENT[moid], moid, MO.REALBASE[moid])$$NL$
                $END$
            $END$
        $END$
        $IF (preid == SHARED_DATA.LIMIT_MO)$
$           // data_sharedの終端はマスクベース方式なのでサイズの倍数にアライン
            $at_mo_order = FIND(MO_ORDER, preid)$
            $check_flag = 1$
            $WHILE check_flag$
                $IF LENGTH(AT(MO_ORDER, at_mo_order))$
                    $IF DEBUG_OPT_TF$
                        $WARNING$
                            $FORMAT("check: %d, 0x%x", AT(MO_ORDER, at_mo_order), +MO.SEFLAG[AT(MO_ORDER, at_mo_order)])$$NL$
                        $END$
                    $END$
                    $IF (MO.SEFLAG[AT(MO_ORDER, at_mo_order)] & 0x20) == 0x20$
                        $IF DEBUG_OPT_TF$
                            $WARNING$
                                $FORMAT("hit: %d, 0x%x", AT(MO_ORDER, at_mo_order), +MO.SEFLAG[AT(MO_ORDER, at_mo_order)])$$NL$
                            $END$
                        $END$
                        $MO.REALALIGN[AT(MO_ORDER, at_mo_order)] = SHARED_DATA.REALSIZE$
                        $check_flag = 0$
                    $END$
                $ELSE$
                    $ERROR$
                        $FORMAT("shared data does not have limit label")$$NL$
                    $END$
                    $check_flag = 0$
                $END$
                $at_mo_order = at_mo_order + 1$
            $END$
        $END$
    $END$

    $preid = moid$
$END$
$IF (preid == SHARED_DATA.LIMIT_MO)$
$   // data_sharedの終端はマスクベース方式なのでサイズの倍数にアライン
    $at_mo_order = FIND(MO_ORDER, preid)$
    $check_flag = 1$
    $WHILE check_flag$
        $IF LENGTH(AT(MO_ORDER, at_mo_order))$
            $IF DEBUG_OPT_TF$
                $WARNING$
                    $FORMAT("check: %d, 0x%x", AT(MO_ORDER, at_mo_order), +MO.SEFLAG[AT(MO_ORDER, at_mo_order)])$$NL$
                $END$
            $END$
            $IF (MO.SEFLAG[AT(MO_ORDER, at_mo_order)] & 0x20) == 0x20$
                $IF DEBUG_OPT_TF$
                    $WARNING$
                        $FORMAT("hit: %d, 0x%x", AT(MO_ORDER, at_mo_order), +MO.SEFLAG[AT(MO_ORDER, at_mo_order)])$$NL$
                    $END$
                $END$
                $MO.REALALIGN[AT(MO_ORDER, at_mo_order)] = SHARED_DATA.REALSIZE$
                $check_flag = 0$
            $END$
        $ELSE$
            $ERROR$
                $FORMAT("shared data does not have limit label")$$NL$
            $END$
            $check_flag = 0$
        $END$
        $at_mo_order = at_mo_order + 1$
    $END$
$END$

$ 
$  arch/v850_gcc/ldscript.tfのターゲット依存部
$ 

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
    STARTUP(start.o)
    ENTRY(__reset)
    $NL$
$END$

$FUNCTION GENERATE_GP_LABEL$
    $TAB$$TAB$__gp = . + 32K;$NL$
    $TAB$$TAB$__tp = . + 32K;$NL$
$END$

$TOPPERS_ATTMOD = TOPPERS_ATTSEC + 1$
$TOPPERS_MPFAREA = TOPPERS_ATTSEC + 2$

$INCLUDE "v850_gcc/ldscript.tf"$

$FOREACH moid MO_ORDER$
	$IF MO.LINKER[moid]$
$		// セクションの開始記述の生成
		$IF (MO.SEFLAG[moid] & 0x01) != 0$
$			// 共有リード専有ライト領域の全体の開始番地
            $IF (MO.SEFLAG[moid] & 0x100) != 0$
$               // 最初に更新された値が有効となる
                $IF !LENGTH(SRPW.BASE)$
                    $SRPW.BASE = MO.BASEADDR[moid]$
                    $IF DEBUG_OPT_TF$
                        $WARNING$
                            $FORMAT("hit: %d, 0x%x", moid, +MO.SEFLAG[moid])$$NL$
                        $END$
                    $END$
                $END$
            $ELIF MO.ACPTN1[moid] == 0 && MO.ACPTN2[moid] == TACP_SHARED
                && (MO.MEMATR[moid] & TA_SDATA) != 0$
$               // 最初に更新された値が有効となる
                $IF !LENGTH(SRPW.BASE)$
                    $SRPW.BASE = MO.BASEADDR[moid]$
                $END$
            $END$
		$END$

$	    // 共有リード専有ライト領域の全体の終了番地
		$IF (MO.SEFLAG[moid] & 0x200) != 0$
            $IF !LENGTH(SRPW.BASE)$
                $ERROR$
                    "start_srpw label is not found"
                $END$
            $END$
$           // 最後に更新された値が有効となる
$             $SRPW.LIMIT = MO.LIMITADDR[moid]$
            $SRPW.LIMIT = LIMIT_SYMBOL(MO.MLABEL[moid])$
            $IF DEBUG_OPT_TF$
                $WARNING$
                    $FORMAT("hit end: %d, 0x%x", moid, SRPW.LIMIT)$$NL$
                $END$
            $END$
        $ELIF MO.ACPTN1[moid] == 0 && MO.ACPTN2[moid] == TACP_SHARED
            && (MO.MEMATR[moid] & TA_SDATA) != 0$
$           // 最後に更新された値が有効となる
$            $SRPW.LIMIT = MO.LIMITADDR[moid]$
            $SRPW.LIMIT = LIMIT_SYMBOL(MO.MLABEL[moid])$
            $IF DEBUG_OPT_TF$
                $WARNING$
                    $FORMAT("hit end: %d, 0x%x", moid, SRPW.LIMIT)$$NL$
                $END$
            $END$
		$END$

	$END$
$END$

$IF DEBUG_OPT_TF$
    $WARNING$
        $FORMAT("srpw area: 0x%x, 0x%x", SRPW.BASE, SRPW.LIMIT)$$NL$
    $END$
$END$

$ 
$  共有領域の初期化ブロックを生成
$ 
$FILE "kernel_mem3.c"$

$TNUM_SHARED_REGION = 3$
const uint32 tnum_shared_mem = $TNUM_SHARED_REGION * 2$;$NL$
uint8 * const shared_meminib_table[$TNUM_SHARED_REGION * 2$] = {$NL$
$shared_info = SYMBOL("shared_meminib_table")$
$   RX領域（共有）
$start_label = PEEK(shared_info, 4)$
$limit_label = PEEK(shared_info + 4, 4)$
$TAB$((uint8 *)&__start_text_$OSAP.LABEL[TDOM_NONE]$),$TAB$/* iregion 2 */$NL$
$TAB$((uint8 *)&__limit_text_$OSAP.LABEL[TDOM_NONE]$),$TAB$/* iregion 2 */$NL$

$   RX領域（共有ショートデータ，共有リード専有ライトの全体）
$   rosdata_shared
$   srpw_data_all
$start_label = PEEK(shared_info + 8, 4)$
$limit_label = PEEK(shared_info + 12, 4)$
$IF LENGTH(SRPW.BASE) && LENGTH(SRPW.LIMIT)$
    $TAB$((uint8 *)$FORMAT("0x%x", SRPW.BASE)$),$TAB$/* iregion 3 */$NL$
    $TAB$((uint8 *)$FORMAT("0x%x", SRPW.LIMIT)$),$TAB$/* iregion 3 */$NL$
$ELSE$
    $TAB$((uint8 *)NULL),$TAB$/* iregion 3 */$NL$
    $TAB$((uint8 *)NULL),$TAB$/* iregion 3 */$NL$
$END$

$   RWX領域（共有ショートデータ，共有データ）
$   //sdata_sharedの先頭からdata_sharedの終端（dataの先頭）まで
$   data_sharedの先頭からsdata_sharedの終端（dataの先頭）まで
$start_label = PEEK(shared_info + 16, 4)$
$limit_label = PEEK(shared_info + 20, 4)$
$start_label = SHARED_DATA.REALBASE$
$limit_label = (SHARED_DATA.REALSIZE - 1) & ~0x0f$
$TAB$$FORMAT("( (uint8 *)0x%x )", start_label)$,$TAB$/* dregion 5 : $FORMAT("0x%x", start_label)$ */$NL$
$TAB$$FORMAT("( (uint8 *)0x%x )", limit_label)$,$TAB$/* dregion 5 : $FORMAT("0x%x", limit_label)$ */$NL$

};$NL$
$NL$



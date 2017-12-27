$ ======================================================================
$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2015-2016 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
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
$  パス4のプロセッサ依存テンプレート（V850用）
$

$  システムスタックの先頭番地取得 (for kernel_mem.tf)
$FUNCTION GET_SSTK_TSKINICTXB$
    $bottom = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_sstk_bottom, sizeof_void_ptr)$
    $size = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_sstksz, sizeof_StackType)$
    $RESULT = (bottom - size)$
$END$

$  ユーザスタックの先頭番地取得 (for kernel_mem.tf)
$FUNCTION GET_USTK_TSKINICTXB$
    $bottom = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_stk_bottom, sizeof_void_ptr)$
    $size = PEEK(ARGV[1] + offsetof_TINIB_TSKINICTXB_stksz, sizeof_StackType)$
    $RESULT = (bottom - size)$
$END$

$ 
$  OSAP.ID_LISTを読み込むため
$ 
$INCLUDE "cfg2_out.tf"$

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
$  MPU設定情報生成の準備
$  状態定義：
$    check == 0 : 初期状態
$    check == 1 : 共有リードでないor非信頼からライトできない範囲
$    check == 2 : 共有リードライトデータの範囲
$    check == 3 : 共有リード専有ライトショートデータの範囲
$    check == 4 : 共有リード専有ライトデータの範囲
$    check_r == 0 : 初期状態
$    check_r == 1 : 対象外
$    check_r == 2 : 共有リードROMの範囲
$    check_r == 3 : 共有リードRAMの範囲
$ 
$check = 0$
$check_r = 0$
$memtop_table = SYMBOL("memtop_table")$
$offset = 0$
$FOREACH osap OSAP.ID_LIST$
    $IF !OSAP.TRUSTED[osap]$
        $OSAP.mem_area_mo[osap] = {}$
    $END$
$END$
$shared_mem_area_mo = {}$
$FOREACH mo MO_MPROTECT_LIST$
    $start_sym = ""$
    $IF EQ(MO.TYPE[mo], "TOPPERS_ATTMEM")$
$       // OsMemoryArea対応
        $IF EQ(MO.OSAPID[mo], "TDOM_NONE")$
$           // 共有領域
            $shared_mem_area_mo = APPEND(shared_mem_area_mo, mo)$
        $ELIF !EQ(MO.OSAPID[mo], "TDOM_KERNEL")$
$           // 非信頼OSAP専有領域
            $TRACE("ATT_MEM?")$
            $TRACE(mo)$
            $OSAP.mem_area_mo[MO.OSAPID[mo]] = APPEND(OSAP.mem_area_mo[MO.OSAPID[mo]], mo)$
        $END$
    $END$
    $IF LENGTH(pre_mo) && ((check >= 2) || (check_r >= 2))$
$       // 空でないメモリオブジェクトを探す
        $FOREACH at RANGE(FIND(MO_ORDER, pre_mo), FIND(MO_ORDER, mo) - 1)$
            $cur_mo = AT(MO_ORDER, at)$
            $start = START_MO_SYMBOL(cur_mo)$
            $limit = LIMIT_MO_SYMBOL(cur_mo)$
            $TRACE("cur_mo")$
            $TRACE(cur_mo)$
            $IF LENGTH(start) && !EQ(start, "")$
                $IF EQ(start_sym, "")$
                    $start_sym = start$
                $END$
                $IF LENGTH(limit)$
                    $limit_sym = limit$
                $END$
            $END$
        $END$
    $END$

    $IF check == 2$
$       // 共有領域の終端番地
        $IF !LENGTH(shared_mo_start) || EQ(shared_mo_start, "")$
            $shared_mo_start = start_sym$
        $END$
        $shared_mo_limit = limit_sym$
        $TRACE("##############")$
        $TRACE(shared_mo_limit)$
    $ELIF check == 3$
$       // 共有リード専有ライトsdata領域の終端番地
        $sosap_mo_start[MO.OSAPID[pre_mo]] = start_sym$
        $sosap_mo_limit[MO.OSAPID[pre_mo]] = limit_sym$
    $ELIF check == 4$
$       // 共有リード専有ライト領域の終端番地
        $osap_mo_start[MO.OSAPID[pre_mo]] = start_sym$
        $osap_mo_limit[MO.OSAPID[pre_mo]] = limit_sym$
        $TRACE("########## check srpw ##########")$
        $TRACE(MO.OSAPID[pre_mo])$
        $TRACE(osap_mo_start)$
    $ELIF check == 5$
$       // 非信頼OSAPの専有リードROM領域
        $osap_text_start[MO.OSAPID[pre_mo]] = start_sym$
        $osap_text_limit[MO.OSAPID[pre_mo]] = limit_sym$
    $ELIF check == 6$
$       // 非信頼OSAPの専有リードRAM領域
        $sosap_text_start[MO.OSAPID[pre_mo]] = start_sym$
        $sosap_text_limit[MO.OSAPID[pre_mo]] = limit_sym$
    $ELIF check == 7$
$       // 非信頼OSAPの専有リードライトショート領域
        $sosap_data_start[MO.OSAPID[pre_mo]] = start_sym$
        $sosap_data_limit[MO.OSAPID[pre_mo]] = limit_sym$
    $ELIF check == 8$
$       // 非信頼OSAPの専有リードライト領域
        $TRACE("########## check prw ##########")$
        $TRACE(MO.OSAPID[pre_mo])$
        $TRACE(start_sym)$
        $TRACE(osap_data_start[MO.OSAPID[pre_mo]])$
        $osap_data_start[MO.OSAPID[pre_mo]] = start_sym$
        $osap_data_limit[MO.OSAPID[pre_mo]] = limit_sym$
    $END$

    $IF check_r == 2$
$       // 共有リードROM領域の終端番地
        $IF !LENGTH(shared_rom_mo_start) || EQ(shared_rom_mo_start, "")$
            $shared_rom_mo_start = start_sym$
        $END$
        $shared_rom_mo_limit = limit_sym$
        $TRACE("########## shared_rom limit ##########")$
        $TRACE(shared_rom_mo_limit)$
    $ELIF check_r == 3$
$       // 共有リードRAM領域の終端番地
        $IF !LENGTH(shared_ram_mo_start) || EQ(shared_ram_mo_start, "")$
            $shared_ram_mo_start = start_sym$
        $END$
        $shared_ram_mo_limit = limit_sym$
        $TRACE("########## shared_ram limit ##########")$
        $TRACE(shared_ram_mo_limit)$
    $END$

    $start_sym = START_MO_SYMBOL(mo)$
    $limit_sym = LIMIT_MO_SYMBOL(mo)$
    $IF EQ(MO.TYPE[mo], "TOPPERS_ATTMEM") || EQ(MO.TYPE[mo], "TOPPERS_USTACK")$
        $check = 1$
        $check_r = 1$
    $ELIF !EQ(MO.ACPTN_R[mo], "TACP_SHARED")$
$       // 共有リードでない
        $IF !EQ(MO.ACPTN_R[mo], "TACP_KERNEL")$
$           // 非信頼OSAPの専有リード領域
            $IF EQ(MO.ACPTN_W[mo], "TACP_KERNEL")$
$               // 非信頼OSAPの専有リード領域
                $IF (MO.MEMATR[mo] & TA_SDATA) == 0$
$                   // 非信頼OSAPの専有リードROM領域
                    $osap_text_start[MO.OSAPID[mo]] = start_sym$
                    $check = 5$
                $ELSE$
$                   // 非信頼OSAPの専有リードRAM領域
                    $sosap_text_start[MO.OSAPID[mo]] = start_sym$
                    $check = 6$
                $END$
            $ELSE$
$               // 非信頼OSAPの専有リードライト領域
                $IF (MO.MEMATR[mo] & TA_SDATA) != 0$
$                   // sdata        
                    $sosap_data_start[MO.OSAPID[mo]] = start_sym$
                    $check = 7$
                $ELSE$
$                   // not sdata        
                    $osap_data_start[MO.OSAPID[mo]] = start_sym$
                    $check = 8$
                $END$
            $END$
        $ELSE$
            $check = 1$
            $check_r = 1$
        $END$
    $ELIF EQ(MO.ACPTN_W[mo], "TACP_SHARED")$
$       // 共有リードライト        
        $IF LENGTH(shared_mo_start)$
            $IF check != 2$
$               // pre_moが共有リードライトでない場合はメモリ領域が連続していない
$               // ためエラーとする
                $ERROR$
                    shared mo duplicate
                $END$
            $END$
        $ELSE$
            $shared_mo_start = start_sym$
        $END$
        $TRACE("start_srw")$
        $TRACE(mo)$
        $check = 2$
        $check_r = 1$
    $ELIF MO.MEMREG[mo] == STANDARD_RAM$
$       // 共有リードRAM領域        
        $IF !EQ(MO.ACPTN_W[mo], "TACP_KERNEL")$
$           // 共有リード専有ライト        
            $IF (MO.MEMATR[mo] & TA_SDATA) != 0$
$               // sdata        
                $sosap_mo_start[MO.OSAPID[mo]] = start_sym$
                $check = 3$
            $ELSE$
$               // not sdata        
                $osap_mo_start[MO.OSAPID[mo]] = start_sym$
                $check = 4$
            $END$
        $ELSE$
            $check = 1$
        $END$
        $IF LENGTH(shared_ram_mo_start)$
            $IF check_r != 3$
$               // pre_moが共有リードでない場合はメモリ領域が連続していない
$               // ためエラーとする
                $ERROR$
                    shared_ram mo duplicate
                $END$
            $END$
        $ELSE$
            $shared_ram_mo_start = start_sym$
        $END$
        $check_r = 3$
    $ELIF MO.MEMREG[mo] == STANDARD_ROM$
$       // 共有リードROM領域        
        $check = 1$
        $IF LENGTH(shared_rom_mo_start)$
            $IF check_r != 2$
$               // pre_moが共有リードでない場合はメモリ領域が連続していない
$               // ためエラーとする
                $ERROR$
                    shared_rom mo duplicate
                $END$
            $END$
        $ELSE$
            $shared_rom_mo_start = start_sym$
        $END$
        $check_r = 2$
    $ELSE$
        $check = 1$
        $check_r = 1$
    $END$

    $pre_mo = mo$
$END$
$TRACE(pre_mo)$
$TRACE(mo)$
$IF ((check >= 2) || (check_r >= 2))$
    $start_sym = ""$
$       // 空でないメモリオブジェクトを探す
    $FOREACH at RANGE(FIND(MO_ORDER, pre_mo), LENGTH(MO_ORDER) - 1)$
        $cur_mo = AT(MO_ORDER, at)$
        $start = START_MO_SYMBOL(cur_mo)$
        $limit = LIMIT_MO_SYMBOL(cur_mo)$
        $IF LENGTH(start) && !EQ(start, "")$
            $IF EQ(start_sym, "")$
                $start_sym = start$
            $END$
            $IF LENGTH(limit)$
                $limit_sym = limit$
            $END$
        $END$
    $END$
$END$

$IF check == 2$
$   // 共有領域の終端番地
    $IF !LENGTH(shared_mo_start) || EQ(shared_mo_start, "")$
        $shared_mo_start = start_sym$
    $END$
    $shared_mo_limit = limit_sym$
    $TRACE("**************")$
    $TRACE(shared_mo_limit)$
$ELIF check == 3$
$   // 共有リード専有ライトsdata領域の終端番地
    $sosap_mo_start[MO.OSAPID[pre_mo]] = start_sym$
    $sosap_mo_limit[MO.OSAPID[pre_mo]] = limit_sym$
$ELIF check == 4$
$   // 共有リード専有ライト領域の終端番地
    $osap_mo_start[MO.OSAPID[pre_mo]] = start_sym$
    $osap_mo_limit[MO.OSAPID[pre_mo]] = limit_sym$
$END$

$IF check_r == 2$
$   // 共有リードROM領域の終端番地
    $IF !LENGTH(shared_rom_mo_start) || EQ(shared_rom_mo_start, "")$
        $shared_rom_mo_start = start_sym$
    $END$
    $shared_rom_mo_limit = limit_sym$
    $TRACE("########## shared_rom limit ##########")$
    $TRACE(shared_rom_mo_limit)$
$ELIF check_r == 3$
$   // 共有リードRAM領域の終端番地
    $IF !LENGTH(shared_ram_mo_start) || EQ(shared_ram_mo_start, "")$
        $shared_ram_mo_start = start_sym$
    $END$
    $shared_ram_mo_limit = limit_sym$
    $TRACE("########## shared_ram limit ##########")$
    $TRACE(shared_ram_mo_limit)$
$END$

$ 
$  MPUにセットする上下限アドレスを取得し，
$  OSAPごとのMPRC設定値をセットする
$  ARGV[1] : 先頭アドレス 
$  ARGV[2] : 末尾アドレス 
$  ARGV[3] : MPRCのビット位置
$  RETURN : {MPUにセットする上限アドレス, 下限アドレス}
$ 
$FUNCTION GET_MPUL_SET_MPRC$
    $start_address = ALT(ARGV[1], MPU_PAGE_MASK)$
    $end_address = ALT(ARGV[2],   MPU_PAGE_MASK)$
    $TRACE(start_address)$
    $ipal = (start_address & MPU_PAGE_MASK)$ 
    $ipau = (end_address & MPU_PAGE_MASK)$ 
    $WARNING$
        $FORMAT("info%d: start=0x%x, end=0x%x", info, start_address, end_address)$
    $END$
    $IF (ipal != start_address) || (ipau != end_address)$
        $ERROR$
            $FORMAT("Not aligned: start=0x%x, end=0x%x", start_address, end_address)$
        $END$
    $END$
    $IF ipal != ipau$
        $ipau = (ipau - 0x1) & 0xfffffffc$
        $FOREACH osap OSAP.ID_LIST$
            $IF !OSAP.TRUSTED[osap]$
                $OSAP.MPRC[osap] = OSAP.MPRC[osap] | (0x1 << ARGV[3])$
                $TRACE(OSAP.MPRC[osap])$
            $END$
        $END$
    $END$
    $WARNING$
        $FORMAT("info%d: ipal=0x%x, ipau=0x%x", info, ipal, ipau)$
    $END$

    $RESULT = APPEND(ipal, ipau)$
$END$

$ 
$  MPUにセットする上下限アドレスを取得し，
$  OSAPごとのMPRC設定値をセットする
$  ARGV[1] : 先頭アドレス 
$  ARGV[2] : 末尾アドレス 
$  ARGV[3] : MPRCのビット位置
$  ARGV[4] : OSAPID
$  RETURN : {MPUにセットする上限アドレス, 下限アドレス}
$ 
$FUNCTION GET_MPUL_SET_MPRC_OSAP$
    $TRACE("GET_MPUL_SET_MPRC_OSAP")$
    $TRACE(ARGV[4])$
    $start_address = ALT(ARGV[1], MPU_PAGE_MASK)$
    $end_address = ALT(ARGV[2],   MPU_PAGE_MASK)$
    $ipal = (start_address & MPU_PAGE_MASK)$ 
    $ipau = (end_address & MPU_PAGE_MASK)$ 
    $WARNING$
        $FORMAT("info%d: start=0x%x, end=0x%x", info, start_address, end_address)$
    $END$
    $IF (ipal != start_address) || (ipau != end_address)$
        $ERROR$
            $FORMAT("Not aligned: start=0x%x, end=0x%x", start_address, end_address)$
        $END$
    $END$
    $IF ipal != ipau$
        $ipau = (ipau - 0x1) & 0xfffffffc$
        $OSAP.MPRC[ARGV[4]] = OSAP.MPRC[ARGV[4]] | (0x1 << ARGV[3])$
    $END$
    $WARNING$
        $FORMAT("info%d: ipal=0x%x, ipau=0x%x", info, ipal, ipau)$
    $END$

    $RESULT = APPEND(ipal, ipau)$
$END$

$ 
$ 共有領域初期化ブロックのMPU設定に関連したデータを生成
$ 初期化ブロックをkernel_mem.cに出力する処理は，pass2と
$ 合わせるために，ここで実施しない
$ 
$FUNCTION GENERATE_TARGET_MPUINFOB$
$   // OSAPごとのMPRCレジスタ設定値を初期化    
$   // ユーザスタック領域MPU0は必ず有効
    $FOREACH osap OSAP.ID_LIST$
        $IF !OSAP.TRUSTED[osap]$
            $OSAP.MPRC[osap] = 0x01$
        $END$
    $END$
    $cur_shared_mpu_id = 0$

    $PREPARE_OSAPINICTXB()$

    $FOREACH osap OSAP.ID_LIST$
        $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[osap]) && 
            (ENUM_MPU_OSAP_ATTMEM[osap] > 0)$
            static MPUINFOB mpu_area_info_$osap$[$ENUM_MPU_OSAP_ATTMEM[osap]$] = {$NL$

            $cur_osap_mpu_id = TNUM_MPU_OSAP_DEFAULT + 1$
            $FOREACH mo OSAP.mem_area_mo[osap]$
$               // 専有ATT_MEM
                $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(MO.BASEADDR[mo], 
                MO.LIMITADDR[mo], 
                cur_osap_mpu_id,
                osap)$
$               // MPATのデフォルトはE|G|SX|SW|SR
                $osap_mpu_mpat = 0xF8$
                $IF !EQ(MO.ACPTN_R[mo], "TACP_KERNEL")$
                    $osap_mpu_mpat = osap_mpu_mpat | 0x01$
                $END$
                $IF !EQ(MO.ACPTN_W[mo], "TACP_KERNEL")$
                    $osap_mpu_mpat = osap_mpu_mpat | 0x02$
                $END$
                $IF !EQ(MO.ACPTN_X[mo], "TACP_KERNEL")$
                    $osap_mpu_mpat = osap_mpu_mpat | 0x04$
                $END$
                $ipal = AT(osap_mpu_reg, 0)$
                $ipau = AT(osap_mpu_reg, 1)$
                $TAB$$TAB${$NL$
                $TAB$$TAB$$TAB$( (uint32)($FORMAT("0x%x", +ipal)$) ), /* MPUL$cur_osap_mpu_id$ */$NL$
                $TAB$$TAB$$TAB$( (uint32)($FORMAT("0x%x", +ipau)$) ), /* MPUA$cur_osap_mpu_id$ */$NL$
                $TAB$$TAB$$TAB$( (uint32)($FORMAT("0x%x", +osap_mpu_mpat)$) ), /* MPAT$cur_osap_mpu_id$ */$NL$
                $TAB$$TAB$},$NL$
                $cur_osap_mpu_id = cur_osap_mpu_id + 1$
            $END$
            };$NL$
            $NL$
        $END$
    $END$

    $IF USE_MPU_SRW_DATA_SDATA$
$       // 共有リードライト    
        $shared_mpu_reg[cur_shared_mpu_id] = 
        GET_MPUL_SET_MPRC(shared_mo_start, shared_mo_limit, 
        TNUM_MPU_REG - cur_shared_mpu_id - 1)$
        $shared_mpu_reg[cur_shared_mpu_id] = APPEND(shared_mpu_reg[cur_shared_mpu_id], { 1, 1, 1 })$
        $TRACE("********** srw *********")$
        $TRACE(cur_shared_mpu_id)$
        $TRACE(shared_mpu_reg[cur_shared_mpu_id])$
        $cur_shared_mpu_id = cur_shared_mpu_id + 1$
    $END$
    $IF USE_MPU_SR_TEXT$
$       // 共有リードROM
        $TRACE(shared_rom_mo_start)$
        $shared_mpu_reg[cur_shared_mpu_id] = 
        GET_MPUL_SET_MPRC(shared_rom_mo_start, shared_rom_mo_limit, 
        TNUM_MPU_REG - cur_shared_mpu_id - 1)$
        $shared_mpu_reg[cur_shared_mpu_id] = APPEND(shared_mpu_reg[cur_shared_mpu_id], { 1, 0, 1 })$
        $cur_shared_mpu_id = cur_shared_mpu_id + 1$
    $END$
    $IF USE_MPU_SR_DATA_SDATA$
$       // 共有リードRAM
        $shared_mpu_reg[cur_shared_mpu_id] = 
        GET_MPUL_SET_MPRC(shared_ram_mo_start, shared_ram_mo_limit, 
        TNUM_MPU_REG - cur_shared_mpu_id - 1)$
        $shared_mpu_reg[cur_shared_mpu_id] = APPEND(shared_mpu_reg[cur_shared_mpu_id], { 1, 0, 1 })$
        $cur_shared_mpu_id = cur_shared_mpu_id + 1$
    $END$
    $FOREACH mo shared_mem_area_mo$
$       // 共有ATT_MEM
        $shared_mpu_reg[cur_shared_mpu_id] = 
        GET_MPUL_SET_MPRC(MO.BASEADDR[mo], MO.LIMITADDR[mo], 
        TNUM_MPU_REG - cur_shared_mpu_id - 1)$
        $shared_mpu_reg[cur_shared_mpu_id] = APPEND(shared_mpu_reg[cur_shared_mpu_id], { MO.ACPTN_R[mo], 
        MO.ACPTN_W[mo],
        MO.ACPTN_X[mo] })$
        $cur_shared_mpu_id = cur_shared_mpu_id + 1$
    $END$
    $FOREACH mpuid RANGE(cur_shared_mpu_id, TNUM_MPU_SHARED - 1)$
        $shared_mpu_reg[cur_shared_mpu_id] = { 0, 0, 0, 0, 0 }$
    $END$
$END$

$FUNCTION GENERATE_OSAPINIB_MPUINFOB$
    $cur_osap_mpu_id = 1$

$   // 関数呼び出しでARGVが上書きされるためここで保存する
    $arg_osapid = ARGV[1]$

    $TAB$$TAB${$NL$
    $IF !OSAP.TRUSTED[arg_osapid]$
        $IF !LENGTH(OSAP.MPRC[arg_osapid])$
            $OSAP.MPRC[arg_osapid] = 0x01$
        $END$
        $IF USE_MPU_SRPW_DATA$
$           // 共有リード専有ライトデータ
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(osap_mo_start[arg_osapid], 
            osap_mo_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPUL$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPUA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_SRPW_SDATA$
$           // 共有リード専有ライトショートデータ
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(sosap_mo_start[arg_osapid], 
            sosap_mo_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PR_TEXT$
$           // 専有リードROM
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(osap_text_start[arg_osapid], 
            osap_text_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PR_SDATA$
$           // 専有リードRAM
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(sosap_text_start[arg_osapid], 
            sosap_text_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPLA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PRW_DATA$
$           // 専有リードライトデータ
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(osap_data_start[arg_osapid], 
            osap_data_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPUL$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPUA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PRW_SDATA$
$           // 共有リード専有ライトショートデータ
            $osap_mpu_reg = GET_MPUL_SET_MPRC_OSAP(sosap_data_start[arg_osapid], 
            sosap_data_limit[arg_osapid], 
            cur_osap_mpu_id,
            arg_osapid)$
            $ipal = AT(osap_mpu_reg, 0)$
            $ipau = AT(osap_mpu_reg, 1)$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipal)$) ), /* MPUL$cur_osap_mpu_id$ */$NL$
            $TAB$$TAB$$TAB$( (uint8 *)($FORMAT("0x%x", +ipau)$) ), /* MPUA$cur_osap_mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF LENGTH(ENUM_MPU_OSAP_ATTMEM[arg_osapid]) &&
            (ENUM_MPU_OSAP_ATTMEM[arg_osapid] > 0)$
            $TAB$$TAB$$TAB$mpu_area_info_$arg_osapid$, /* mpu_area_info */$NL$
            $TAB$$TAB$$TAB$$ENUM_MPU_OSAP_ATTMEM[arg_osapid]$U,$NL$
        $ELSE$
            $TAB$$TAB$$TAB$NULL, /* mpu_area_info */$NL$
            $TAB$$TAB$$TAB$0U,$NL$
        $END$
        $TAB$$TAB$$TAB$( (uint32)$FORMAT("0x%x", +OSAP.MPRC[arg_osapid])$ ),/* MPRC */$NL$
    $ELSE$
        $IF USE_MPU_SRPW_DATA$
$           // 共有リード専有ライトデータ
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_SRPW_SDATA$
$           // 共有リード専有ライトショートデータ
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PR_TEXT$
$           // 専有リードROM
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PR_SDATA$
$           // 専有リードRAM
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PRW_DATA$
$           // 専有リードライトデータ
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $IF USE_MPU_PRW_SDATA$
$           // 共有リード専有ライトショートデータ
            $TAB$$TAB$$TAB$0, 0, /* MPU$mpu_id$ */$NL$
            $cur_osap_mpu_id = cur_osap_mpu_id + 1$
        $END$
        $TAB$$TAB$$TAB$NULL, /* mpu_area_info */$NL$
        $TAB$$TAB$$TAB$0U,$NL$
        $TAB$$TAB$$TAB$( (uint32)0 ),/* MPRC */$NL$
    $END$
    $TAB$$TAB$}$NL$
    $NL$
$END$

$FUNCTION GENERATE_TSKINICTXB$
    $TAB$$TAB${$NL$
    $TAB$$TAB$$TAB$$TSK.TINIB_SSTKSZ[ARGV[1]]$,$NL$
    $TAB$$TAB$$TAB$((void *)((uint8 *)($TSK.TINIB_SSTK[ARGV[1]]$)
    $SPC$+ ($TSK.TINIB_SSTKSZ[ARGV[1]]$))),$NL$
    $IF OSAP.TRUSTED[TSK.OSAPID[ARGV[1]]] || (tmin_os_restarttask <= ARGV[1])$
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
                $section = SECTION_SHARED_USTACK(TSK.SHARED_USTK_ID[ARGV[1]])$
            $ELSE$
$               // 固有スタック
                $section = SECTION_USTACK(ARGV[1])$
            $END$
            $start_address = START_SYMBOL(section)$
            $end_address = LIMIT_SYMBOL(section)$
        $ELSE$
$			// stkがNULLでない場合の処理
            $p_tinib = SYMBOL("tinib_table")$
            $p_tinib = p_tinib + TSK.ID[ARGV[1]] * sizeof_TINIB$
            $start_address = PEEK(p_tinib + offsetof_TINIB_STKMPUINFOB_start_ustk, 4)$
            $end_address = PEEK(p_tinib + offsetof_TINIB_STKMPUINFOB_limit_ustk, 4)$
        $END$
        $mpl = (start_address & MPU_PAGE_MASK)$ 
        $mpu = (end_address & MPU_PAGE_MASK)$ 
        $IF (mpl != start_address) || (mpu != end_address)$
            $ERROR$
                $FORMAT("task is %s", ARGV[1])$$NL$
                $FORMAT("user stack in not aligned: start=0x%x, end=0x%x", start_address, end_address)$$NL$
                $FORMAT("mpla=0x%x, mpua=0x%x", mpl, mpu)$
            $END$
        $END$
        $IF mpl == mpu$
            $ERROR$
                $FORMAT("task is %s", ARGV[1])$$NL$
                $FORMAT("user stack size is 0: start=0x%x, end=0x%x", start_address, end_address)$$NL$
                $FORMAT("mpla=0x%x, mpua=0x%x", mpl, mpu)$
            $END$
        $END$
        $mpu = (mpu - 0x1) & 0xfffffffc$
        $TAB$$TAB$$TAB$$FORMAT("(void *)0x%x /* &__start_user_stack %s */", +mpl, ARGV[1])$,$NL$
        $TAB$$TAB$$TAB$$FORMAT("(void *)0x%x /* &__limit_user_stack %s */", +mpu, ARGV[1])$,$NL$
    $END$
    $TAB$$TAB$},$NL$
$END$

$MEMORY_ALIGN = 4$
$FUNCTION HOOK_ERRORCHECK_MEM_PASS4$
$	// baseがメモリ保護境界の制約に合致していない場合 NOS0813
    $TRACE(MEMORY_ALIGN)$
    $IF (MO.BASEADDR[ARGV[1]] & (MEMORY_ALIGN - 1)) != 0$
        $ERROR MO.TEXT_LINE[ARGV[1]]$
            $FORMAT(_("%1% `%2%\' is not aligned to %3%"),
            "OsMemoryAreaStartAddress", MO.BASEADDR[ARGV[1]], MEMORY_ALIGN)$
        $END$
    $END$
$END$

$ 
$  非依存部の読込み
$ 
$INCLUDE "kernel/kernel_mem.tf"$

$ 
$  共有のMPU設定情報を生成
$  pass2での宣言位置と合わせる
$ 
$FILE "kernel_mem.c"$

$IF TNUM_MPU_SHARED > 0$
    uint8 * const shared_meminib_table[$TNUM_MPU_SHARED * 3$] = {$NL$
    $FOREACH memid RANGE(0,TNUM_MPU_SHARED-1)$
        $TRACE(memid)$
        $TRACE(shared_mpu_reg[memid])$
        $TRACE(AT(shared_mpu_reg[memid], 0))$
        $TRACE(AT(shared_mpu_reg[memid], 1))$
        $TAB$$FORMAT("( (uint8 *)0x%x )", +AT(shared_mpu_reg[memid], 0))$, /* MPUL$TNUM_MPU_REG - 1 - memid$ */$NL$
        $TAB$$FORMAT("( (uint8 *)0x%x )", +AT(shared_mpu_reg[memid], 1))$, /* MPUA$TNUM_MPU_REG - 1 - memid$ */$NL$
$   // MPATのデフォルトはE|G|SX|SW|SR
        $osap_mpu_mpat = 0xF8$
        $IF AT(shared_mpu_reg[memid], 2) != 0$
$       // MPAT |= UR
            $osap_mpu_mpat = osap_mpu_mpat | 0x01$
        $END$
        $IF AT(shared_mpu_reg[memid], 3) != 0$
$       // MPAT |= UW
            $osap_mpu_mpat = osap_mpu_mpat | 0x02$
        $END$
        $IF AT(shared_mpu_reg[memid], 4) != 0$
$       // MPAT |= UX
            $osap_mpu_mpat = osap_mpu_mpat | 0x04$
        $END$
        $TAB$$FORMAT("( (uint8 *)0x%x )", +osap_mpu_mpat)$, /* MPAT$TNUM_MPU_REG - 1 - memid$ */$NL$
    $END$
    };$NL$
    $NL$
$END$


$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2007-2015 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
$  Copyright (C) 2011-2013 by Spansion LLC, USA
$  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
$  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2011-2015 by Witz Corporation
$  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
$  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
$  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
$  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
$  $Id: ldscript.tf 425 2015-12-07 08:06:19Z witz-itoyo $
$

$ =====================================================================
$  仮のリンカスクリプト（cfg2_out.ld）の生成
$ =====================================================================

$
$  生成するファイルの指定
$
$FILE "cfg2_out.ld"$

$
$  ターゲット依存のOUTPUT記述の生成
$
$IF ISFUNCTION("GENERATE_OUTPUT")$
	$GENERATE_OUTPUT()$
$END$

$
$  MEMORY記述の生成
$
MEMORY {$NL$
$FOREACH reg REG.ORDER_LIST$
	$TAB$$FORMAT("%s : ORIGIN = 0x%x, LENGTH = 0x%x",
				REG.REGNAME[reg], +REG.BASE[reg], +REG.SIZE[reg])$$NL$
$END$
$IF ISFUNCTION("GENERATE_MEMORY")$
	$GENERATE_MEMORY()$
$END$
}$NL$
$NL$

$
$  ターゲット依存のPROVIDE記述の生成
$
$IF ISFUNCTION("GENERATE_PROVIDE")$
	$GENERATE_PROVIDE()$
$END$

$
$  変数の初期化
$
$reset_lma = 0$

$provide_gp = 0$

$
$  リンク指定の生成
$
SECTIONS {$NL$
$ ターゲット依存のセクション記述の生成
$IF ISFUNCTION("GENERATE_SECTION_FIRST")$
	$GENERATE_SECTION_FIRST()$
$END$

$NL$

$TAB$/* Kernel data on local memory of each cores.(core1...core4)$NL$
$TAB$$SPC$* Do not relocate these section! */$NL$
$FOREACH coreid RANGE(0,TMAX_COREID)$
	$TAB$.kernel_ccb_data$coreid$ :$NL$
	$TAB${$NL$
	    $TAB$$TAB$. = ALIGN(32 / 8);$NL$
	    $TAB$$TAB$__data$coreid$_start = ABSOLUTE(.);$NL$
	    $TAB$$TAB$*(._kernel_core$coreid$s_ccb)$NL$
	    $TAB$$TAB$__data$coreid$_end = ABSOLUTE(.);$NL$
	$TAB$} > onchip_osdataram$coreid$$NL$
	$NL$
$END$

$FOREACH coreid LOCAL_MEM_CORE_LIST$
	$TAB$.kernel_data_core$coreid$ :$NL$
	$TAB${$NL$
	    $TAB$$TAB$. = ALIGN(32 / 8);$NL$
	    $TAB$$TAB$__data_core$coreid$_start = ABSOLUTE(.);$NL$
	    $TAB$$TAB$*(._kernel_core$coreid$*)$NL$
	    $TAB$$TAB$__data_core$coreid$_end = ABSOLUTE(.);$NL$
		$TAB$$TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
		$TAB$$TAB$__limit_data_core$coreid$_end = .;$NL$
	$TAB$} > $REG.REGNAME[CORE_RAM[coreid]]$$NL$
	$NL$
$END$

$FOREACH osapid LOCAL_MEM_OSAP_LIST$
	$IF LENGTH(OSAP.RAM_SEC[osapid])$
		$TAB$.kernel_data_osap$+osapid$ :$NL$
		$TAB${$NL$
		    $TAB$$TAB$. = ALIGN(32 / 8);$NL$
		    $TAB$$TAB$__data_osap$+osapid$_start = ABSOLUTE(.);$NL$
		    $TAB$$TAB$*(._kernel_osap$+osapid$*)$NL$
		    $TAB$$TAB$__data_osap$+osapid$_end = ABSOLUTE(.);$NL$
			$TAB$$TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
			$TAB$$TAB$__limit_data_osap$+osapid$_end = .;$NL$
		$TAB$} > $REG.REGNAME[OSAP.RAM[osapid]]$$NL$
	$END$
	$NL$
$END$

$IF TOPPERS_LABEL_ASM$
	$PREFIX_START = "___start_"$
	$PREFIX_END   = "___end_"$
	$PREFIX_LIMIT = "___limit_"$
$ELSE$
	$PREFIX_START = "__start_"$
	$PREFIX_END   = "__end_"$
	$PREFIX_LIMIT = "__limit_"$
$END$

$FOREACH moid MO_ORDER$
	$IF MO.LINKER[moid]$
$		// セクションの開始記述の生成
		$IF (MO.SEFLAG[moid] & 0x01) != 0$
			$TAB$.$MO.SLABEL[moid]$$SPC$
			$IF (MO.SEFLAG[moid] & 0x40) == 0$
				ALIGN($TARGET_SEC_ALIGN_STR$)$SPC$
			$END$
			$IF (MO.MEMATR[moid] & TA_NOWRITE) == 0
							&& (MO.MEMATR[moid] & TA_MEMINI) == 0$
				(NOLOAD)$SPC$
			$END$
			:$SPC$
			$IF !EQ(MO.ILABEL[moid], "")$
				ALIGN($TARGET_SEC_ALIGN_STR$)$SPC$
			$END$
			{$NL$
		$END$

$		// セクションとメモリオブジェクトの開始ラベルの生成
		$IF (MO.SEFLAG[moid] & 0x400) != 0$
			$IF ISFUNCTION("GENERATE_GP_LABEL")$
				$GENERATE_GP_LABEL()$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x100) != 0$
			$IF (MO.MEMATR[moid] & TA_SDATA) == 0$
				$TAB$$TAB$$PREFIX_START$srpw_all = .;$NL$
			$ELSE$
				$TAB$$TAB$$PREFIX_START$ssrpw_all = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x10) != 0$
			$IF !EQ(MO.PLABEL[moid], "")$
				$TAB$$TAB$$PREFIX_START$$MO.PLABEL[moid]$ = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x01) != 0$
			$IF !EQ(MO.SLABEL[moid], ALT(omit_start_slabel,""))$
				$TAB$$TAB$$PREFIX_START$$MO.SLABEL[moid]$ = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x04) != 0$
			$IF (MO.SEFLAG[moid] & 0x01) == 0$
				$NL$
			$END$
			$IF !EQ(MO.MLABEL[moid], MO.SLABEL[moid])
					&& !EQ(MO.MLABEL[moid], ALT(omit_start_mlabel,""))$
				$TAB$$TAB$$PREFIX_START$$MO.MLABEL[moid]$ = .;$NL$
			$END$
			$TAB$$TAB$
		$END$

$		// リンクするファイル記述の生成
		$section_dscr = SECTION_DESCRIPTION(MO.SECTION[moid])$
		$IF MO.TYPE[moid] == TOPPERS_ATTMOD$
			$IF EQ(MO.MODULE[moid], "kernel_mem.o")$
				kernel_mem2.o
			$ELSE$
				$MO.MODULE[moid]$
			$END$
			($section_dscr$)$SPC$
		$ELIF MO.TYPE[moid] == TOPPERS_ATTSEC$
			*($section_dscr$)$SPC$
		$ELSE$
$			// ユーザスタック領域
			Os_Lcfg.o($section_dscr$)$SPC$
		$END$

$		// セクションとメモリオブジェクトの終了ラベルの生成
		$IF (MO.SEFLAG[moid] & 0x08) != 0$
			$NL$
			$IF !EQ(MO.MLABEL[moid], MO.SLABEL[moid])$
				$TAB$$TAB$$PREFIX_END$$MO.MLABEL[moid]$ = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x02) != 0$
			$TAB$$TAB$$PREFIX_END$$MO.SLABEL[moid]$ = .;$NL$
		$END$
		$IF (MO.SEFLAG[moid] & 0x200) != 0$
			$IF (MO.MEMATR[moid] & TA_SDATA) == 0$
				$TAB$$TAB$$PREFIX_END$srpw_all = .;$NL$
			$ELSE$
				$TAB$$TAB$$PREFIX_END$ssrpw_all = .;$NL$
			$END$
		$END$

$		// セクションの終了記述の生成
		$IF (MO.SEFLAG[moid] & 0x02) != 0$
			$TAB$} > $REG.REGNAME[MO.MEMREG[moid]]$
			$IF !OMIT_IDATA$
				$IF !EQ(MO.ILABEL[moid], "")$
					$SPC$AT > $REG.REGNAME[STANDARD_ROM]$
					$reset_lma = 1$
				$ELIF reset_lma$
					$SPC$AT > $REG.REGNAME[MO.MEMREG[moid]]$
					$reset_lma = 0$
				$END$
			$END$
			$IF ISFUNCTION("TARGET_CODE_FILLER")$
				$TARGET_CODE_FILLER(moid)$
			$END$
			$NL$
			$IF !OMIT_IDATA && !EQ(MO.ILABEL[moid], "")$
				$TAB$$PREFIX_START$$MO.ILABEL[moid]$ = LOADADDR(.$MO.SLABEL[moid]$);$NL$
			$END$
		$END$

$		// アライン記述の生成
		$IF (MO.SEFLAG[moid] & 0x20) != 0$
			$TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
		$ELIF (MO.SEFLAG[moid] & 0x08) != 0$
$			// セクションが継続している場合には，TABを1つ出力
			$IF (MO.SEFLAG[moid] & 0x02) == 0$$TAB$$END$
			$TAB$. = ALIGN($TARGET_SEC_ALIGN_STR$);$NL$
		$END$

$		// セクションとメモリオブジェクトの上限ラベルの生成
		$IF (MO.SEFLAG[moid] & 0x08) != 0$
			$IF !EQ(MO.MLABEL[moid], MO.SLABEL[moid])$
$				// セクションが継続している場合には，TABを1つ出力
				$IF (MO.SEFLAG[moid] & 0x02) == 0$$TAB$$END$
				$TAB$$PREFIX_LIMIT$$MO.MLABEL[moid]$ = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x02) != 0$
			$TAB$$PREFIX_LIMIT$$MO.SLABEL[moid]$ = .;$NL$
		$END$
		$IF (MO.SEFLAG[moid] & 0x20) != 0$
			$IF !EQ(MO.PLABEL[moid], "")$
				$TAB$$PREFIX_LIMIT$$MO.PLABEL[moid]$ = .;$NL$
			$END$
		$END$
		$IF (MO.SEFLAG[moid] & 0x200) != 0$
			$IF (MO.MEMATR[moid] & TA_SDATA) == 0$
				$TAB$$PREFIX_LIMIT$srpw_all = .;$NL$
			$ELSE$
				$TAB$$PREFIX_LIMIT$ssrpw_all = .;$NL$
			$END$
		$END$

		$IF (MO.SEFLAG[moid] & 0x02) != 0$
			$NL$
		$END$

$		// メモリリージョン末でアラインさせるためのダミーセクションの生成
		$IF (MO.SEFLAG[moid] & 0x80) != 0$
			$TAB$.$REG.REGNAME[MO.MEMREG[moid]]$_align$SPC$
					ALIGN($TARGET_SEC_ALIGN_STR$) : {$NL$
			$TAB$$TAB$__$REG.REGNAME[MO.MEMREG[moid]]$_align = .;$NL$
			$TAB$} > $REG.REGNAME[MO.MEMREG[moid]]$$NL$
			$NL$
		$END$
	$END$
$END$

$FOREACH lsid RANGE(1, numls)$
	$TAB$$LNKSEC.SECTION[lsid]$ : {$NL$
	$TAB$$TAB$*($LNKSEC.SECTION[lsid]$)$NL$
	$TAB$} > $REG.REGNAME[LNKSEC.MEMREG[lsid]]$$NL$
	$NL$
$END$

$	// C++testで必要な標準ライブラリで使用するセクション
$TAB$.dummy_for_cpptest : {$NL$
$TAB$$TAB$*(.ctors.65535) *(.eh_frame)$NL$
$TAB$$TAB$_end = .;$NL$
$TAB$} > RAM$NL$
$TAB$. = ALIGN(64);$NL$$NL$

$TAB$/DISCARD/ : {$NL$
$TAB$$TAB$*(.rel.dyn)$NL$
$TAB$}$NL$
$NL$

$TAB$/* DWARF debug sections.$NL$
$TAB$Symbols in the DWARF debugging sections are relative to $NL$
$TAB$the beginning of the section so we begin them at 0.  */$NL$
$NL$
$TAB$/* DWARF 1 */$NL$
$TAB$.debug          0 : { *(.debug) }$NL$
$TAB$.line           0 : { *(.line) }$NL$
$NL$
$TAB$/* GNU DWARF 1 extensions */$NL$
$TAB$.debug_srcinfo  0 : { *(.debug_srcinfo) }$NL$
$TAB$.debug_sfnames  0 : { *(.debug_sfnames) }$NL$
$NL$
$TAB$/* DWARF 1.1 and DWARF 2 */$NL$
$TAB$.debug_aranges  0 : { *(.debug_aranges) }$NL$
$TAB$.debug_pubnames 0 : { *(.debug_pubnames) }$NL$
$NL$
$TAB$/* DWARF 2 */$NL$
$TAB$.debug_info     0 : { *(.debug_info) }$NL$
$TAB$.debug_abbrev   0 : { *(.debug_abbrev) }$NL$
$TAB$.debug_line     0 : { *(.debug_line) }$NL$
$TAB$.debug_frame    0 : { *(.debug_frame) }$NL$
$TAB$.debug_str      0 : { *(.debug_str) }$NL$
$TAB$.debug_loc      0 : { *(.debug_loc) }$NL$
$TAB$.debug_macinfo  0 : { *(.debug_macinfo) }$NL$
$TAB$.debug_ranges   0 : { *(.debug_ranges) }$NL$
$NL$
$TAB$/* SGI/MIPS DWARF 2 extensions */$NL$
$TAB$.debug_weaknames 0 : { *(.debug_weaknames) }$NL$
$TAB$.debug_funcnames 0 : { *(.debug_funcnames) }$NL$
$TAB$.debug_typenames 0 : { *(.debug_typenames) }$NL$
$TAB$.debug_varnames  0 : { *(.debug_varnames) }$NL$
$NL$
}$NL$

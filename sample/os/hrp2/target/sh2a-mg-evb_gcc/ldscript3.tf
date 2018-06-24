$ ======================================================================
$
$   TOPPERS/HRP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       High Reliable system Profile Kernel
$
$   Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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
$ =====================================================================

$ =====================================================================
$ リンカスクリプトの生成
$ =====================================================================
$FILE "ldscript3.ld"$

$
$  ターゲット依存のOUTPUT記述の生成
$
OUTPUT_FORMAT("elf32-sh") $NL$
OUTPUT_ARCH(sh)           $NL$
$NL$

$
$  MEMORY記述の生成
$
MEMORY {$NL$
$FOREACH reg REG.ORDER_LIST$
	$TAB$$REG.REGNAME[reg]$$SPC$:$SPC$
	ORIGIN = $REG.BASE[reg]$, LENGTH = $REG.SIZE[reg]$$NL$
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
$  リンク指定の生成
$
SECTIONS {$NL$
$ ターゲット依存のセクション記述の生成
$TAB$.vector_start : {$NL$
$TAB$$TAB$$PRE_OBJ$(.vector_start)$NL$
$TAB$} > $REG.REGNAME[STANDARD_ROM]$$NL$
$NL$
$TAB$.vector_entry : {$NL$
$TAB$$TAB$$PRE_OBJ$(.vector_entry)$NL$
$TAB$}  > $REG.REGNAME[STANDARD_ROM]$$NL$
$NL$
$TAB$.bss : {$NL$
$TAB$} > $REG.REGNAME[STANDARD_RAM]$$NL$
$NL$


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
$		// セクションとメモリオブジェクトの開始記述の生成
		$IF (MO.SEFLAG[moid] & 0x01) != 0$
            $TAB$.$MO.SLABEL[moid]$
            $IF LENGTH(POSITION[MO.MEMREG[moid]])$
                $SPC$($POSITION[MO.MEMREG[moid]]$)$SPC$
            $ELSE$
                $SPC$
            $END$
            : {$NL$
		$END$
		$IF (MO.SEFLAG[moid] & 0x02) != 0$

			$IF !OMIT_IDATA && !EQ(MO.ILABEL[moid], "")$
                $TAB$$TAB$$MO.SLABEL[moid]$.$PRE_OBJ$(*)$NL$
			    $TAB$} > $REG.REGNAME[MO.MEMREG[moid]]$$NL$
                $TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
                $POSITION[MO.MEMREG[moid]] = CONCAT(PREFIX_LIMIT,MO.SLABEL[moid])$
                $TAB$$POSITION[MO.MEMREG[moid]]$ = .;$NL$
                $TAB$.$MO.ILABEL[moid]$ ($POSITION[STANDARD_ROM]$) : {$NL$
                $TAB$$TAB$$PRE_OBJ$(.$MO.ILABEL[moid]$)$NL$
                $TAB$} > $REG.REGNAME[STANDARD_ROM]$$NL$
                $TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
                $POSITION[STANDARD_ROM] = CONCAT(PREFIX_LIMIT,MO.ILABEL[moid])$
                $TAB$$POSITION[STANDARD_ROM]$ = .;$NL$
            $ELSE$
                $TAB$$TAB$$PRE_OBJ$(.$MO.SLABEL[moid]$)$NL$
			    $TAB$} > $REG.REGNAME[MO.MEMREG[moid]]$$NL$
                $TAB$. = ALIGN($TARGET_PAGE_SIZE_STR$);$NL$
                $POSITION[MO.MEMREG[moid]] = CONCAT(PREFIX_LIMIT,MO.SLABEL[moid])$
                $TAB$$POSITION[MO.MEMREG[moid]]$ = .;$NL$
			$END$
		$END$

		$IF (MO.SEFLAG[moid] & 0x02) != 0$
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

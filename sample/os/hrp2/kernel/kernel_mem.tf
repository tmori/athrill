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
$   $Id: kernel_mem.tf 837 2012-12-26 15:09:59Z ertl-hiro $
$  
$ =====================================================================

$
$  パス2，パス3からの情報の読込み
$
$INCLUDE "cfg2_out.tf"$
$INCLUDE "cfg3_out.tf"$

$
$  データセクションのLMAからVMAへのコピー
$
$FOREACH lma LMA.ORDER_LIST$
	$start_data = SYMBOL(LMA.START_DATA[lma])$
	$end_data = SYMBOL(LMA.END_DATA[lma])$
	$start_idata = SYMBOL(LMA.START_IDATA[lma])$
	$IF !LENGTH(start_data)$
		$ERROR$$FORMAT(_("symbol '%1%' not found"), LMA.START_DATA[lma])$$END$
	$ELIF !LENGTH(end_data)$
		$ERROR$$FORMAT(_("symbol '%1%' not found"), LMA.END_DATA[lma])$$END$
	$ELIF !LENGTH(start_idata)$
		$ERROR$$FORMAT(_("symbol '%1%' not found"), LMA.START_IDATA[lma])$$END$
	$ELSE$
		$BCOPY(start_idata, start_data, end_data - start_data)$
	$END$
$END$

$ =====================================================================
$ kernel_mem.cの共通部分の生成
$ =====================================================================

$FILE "kernel_mem.c"$
/* kernel_mem.c */$NL$
#include "kernel/kernel_int.h"$NL$
#include "kernel_cfg.h"$NL$
$NL$
#if TKERNEL_PRID != 0x06u$NL$
#error "The kernel does not match this configuration file."$NL$
#endif$NL$
$NL$

/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$

$ =====================================================================
$ シンボルと仮メモリオブジェクト初期化ブロックの読込みと前処理
$ =====================================================================

$
$ シンボルと仮メモリオブジェクト初期化ブロックの読込み
$
$ MO.BASEADDR[moid]：メモリオブジェクトの先頭番地
$ MO.LIMITADDR[moid]：メモリオブジェクトの上限番地
$ MO.POFFSET[moid]：物理アドレスとのオフセット
$ MO_MEMTOP_LIST：サイズが0でないメモリオブジェクトのリスト

$IF !OMIT_STANDARD_MEMINIB$
	$MO_MEMTOP_LIST = {}$
	$memtop_table = SYMBOL("_kernel_memtop_table")$
	$offset = 0$
	$FOREACH moid MO_START_LIST$
		$IF MO.LINKER[moid]$
			$MO.BASEADDR[moid] = SYMBOL(CONCAT("__start_", MO.MLABEL[moid]))$
			$MO.LIMITADDR[moid] = SYMBOL(CONCAT("__limit_", MO.MLABEL[moid]))$
			$MO.POFFSET[moid] = 0$
		$ELSE$
			$MO.BASEADDR[moid] = PEEK(memtop_table + offset, sizeof_void_ptr)$
			$offset = offset + sizeof_void_ptr$
			$MO.LIMITADDR[moid] = (MO.BASEADDR[moid] + MO.SIZE[moid]) & ((1 << sizeof_void_ptr * 8) - 1)$
			$IF LENGTH(MO.PADDR[moid])$
				$MO.POFFSET[moid] = MO.PADDR[moid] - MO.BASEADDR[moid]$
			$ELSE$
				$MO.POFFSET[moid] = 0$
			$END$
		$END$

		$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
			$IF MO.BASEADDR[moid] < MO.LIMITADDR[moid]
											|| MO.LIMITADDR[moid] == 0$
				$MO_MEMTOP_LIST = APPEND(MO_MEMTOP_LIST, moid)$
			$ELIF MO.TYPE[moid] == TOPPERS_ATTMEM$
				$ERROR MO.TEXT_LINE[moid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' is too large in %3%"),
								"size", MO.SIZE[moid], MO.APINAME[moid])$
				$END$
			$ELSE$
				$ERROR MO.TEXT_LINE[moid]$E_SYS: 
					$FORMAT(_("illegal memory object size"))$
				$END$
			$END$
		$END$
	$END$
$END$

$
$ メモリオブジェクトの重なりのチェック
$
$ MO.SUPPRESSLIMIT[moid]：次のメモリオブジェクトと隣接している
$ MO.MERGED[moid]：併合されたユーザスタック領域の併合先
$ MO_MEMTOP_ORDER：メモリオブジェクトのベースアドレス順のリスト

$MO_MEMTOP_ORDER = {}$
$prev = 0$
$FOREACH moid SORT(MO_MEMTOP_LIST, "MO.BASEADDR")$
	$IF !prev || (MO.LIMITADDR[prev] <= MO.BASEADDR[moid]
											&& MO.LIMITADDR[prev] != 0)$
		$MO_MEMTOP_ORDER = APPEND(MO_MEMTOP_ORDER, moid)$
		$IF prev && MO.LIMITADDR[prev] == MO.BASEADDR[moid]$
			$MO.SUPPRESSLIMIT[prev] = 1$
		$END$
		$prev = moid$
	$ELSE$
$		// メモリオブジェクトの領域に重なりがある場合
		$IF OMIT_CHECK_USTACK_OVERLAP
					&& MO.TYPE[moid] == TOPPERS_USTACK
					&& MO.TYPE[prev] == TOPPERS_USTACK
					&& MO.DOMAIN[prev] == MO.DOMAIN[moid]$
$			// ユーザスタック領域の併合処理
			$MO.TSKID[prev] = 0$
			$MO.TSKID[moid] = 0$
			$MO.BASE[moid] = MO.BASE[prev]$
			$MO.BASEADDR[moid] = MO.BASEADDR[prev]$
			$IF MO.LIMITADDR[prev] < MO.LIMITADDR[moid]
											&& MO.LIMITADDR[prev] != 0$
				$MO.SIZE[prev] = MO.LIMITADDR[moid] - MO.BASEADDR[prev]$
				$MO.LIMITADDR[prev] = MO.LIMITADDR[moid]$
			$ELSE$
				$MO.SIZE[moid] = MO.SIZE[prev]$
				$MO.LIMITADDR[moid] = MO.LIMITADDR[prev]$
			$END$
			$MO.MERGED[moid] = prev$
		$ELSE$
$			// エラーメッセージの出力
			$IF MO.TYPE[moid] == TOPPERS_ATTMEM$
				$ERROR MO.TEXT_LINE[moid]$E_OBJ: 
					$FORMAT(_("memory object registered with %1% overlaps with another memory object"), MO.APINAME[moid])$
				$END$
			$ELIF MO.TYPE[prev] == TOPPERS_ATTMEM$
				$ERROR MO.TEXT_LINE[prev]$E_OBJ: 
					$FORMAT(_("memory object registered with %1% overlaps with another memory object"), MO.APINAME[prev])$
				$END$
			$ELIF MO.TYPE[moid] == TOPPERS_USTACK$
				$ERROR MO.TEXT_LINE[moid]$E_OBJ: 
					$FORMAT(_("user stack area registered with %1% overlaps with another memory object"), MO.APINAME[moid])$
				$END$
			$ELIF MO.TYPE[prev] == TOPPERS_USTACK$
				$ERROR MO.TEXT_LINE[prev]$E_OBJ: 
					$FORMAT(_("user stack area registered with %1% overlaps with another memory object"), MO.APINAME[prev])$
				$END$
			$ELSE$
				$ERROR MO.TEXT_LINE[moid]$E_SYS: 
					$FORMAT(_("memory objects overlap"))$
				$END$
			$END$
		$END$
	$END$
$END$
$IF MO.LIMITADDR[prev] == 0$
	$MO.SUPPRESSLIMIT[prev] = 1$
$END$

$ =====================================================================
$ メモリオブジェクト初期化ブロックの生成
$ =====================================================================

$IF ISFUNCTION("GENERATE_MEMINIB_TABLE")$
$	// ターゲット依存部のメモリオブジェクト初期化ブロック生成処理を呼ぶ
	$GENERATE_MEMINIB_TABLE()$
$ELIF !OMIT_STANDARD_MEMINIB$
$	// メモリオブジェクト初期化ブロックに出力するエントリの決定
$
$	// tnum_meminib：メモリオブジェクト初期化ブロックに出力するエントリ数
$	// genzero：アドレス0に対応するエントリを出力する
	$tnum_meminib = 0$
	$prev = 0$
	$FOREACH moid MO_MEMTOP_ORDER$
		$IF !prev && MO.BASEADDR[moid] != 0$
			$genzero = 1$
			$tnum_meminib = tnum_meminib + 1$
		$END$
		$IF LENGTH(MO.SUPPRESSLIMIT[moid])$
			$tnum_meminib = tnum_meminib + 1$
		$ELSE$
			$tnum_meminib = tnum_meminib + 2$
		$END$
		$prev = moid$
	$END$

$	// tsize_meminibの妥当性チェック
	$IF tsize_meminib < tnum_meminib$
		$ERROR$E_SYS: 
			$FORMAT(_("illegal memory object initialization block size"))$
		$END$
	$END$

$	// memtop_tableで参照するラベルの宣言の生成
	$FOREACH moid MO_MEMTOP_ORDER$
		$IF MO.LINKER[moid]$
			extern char __start_$MO.MLABEL[moid]$;$NL$
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				extern char __limit_$MO.MLABEL[moid]$;$NL$
			$END$
		$END$
	$END$$NL$

$	// tnum_meminibの生成
	const uint_t _kernel_tnum_meminib = $tnum_meminib$U;$NL$
	$NL$

$	// memtop_tableの生成
	void *const _kernel_memtop_table[$tsize_meminib$] = {$NL$
	$IF LENGTH(genzero)$
		$TAB$0,$NL$
	$END$
	$JOINEACH moid MO_MEMTOP_ORDER ",\n"$
		$IF MO.LINKER[moid]$
			$TAB$&__start_$MO.MLABEL[moid]$
			$SPC$/* $FORMAT("%x", MO.BASEADDR[moid])$ */
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				,$NL$$TAB$&__limit_$MO.MLABEL[moid]$
				$SPC$/* $FORMAT("%x", MO.LIMITADDR[moid])$ */
			$END$
		$ELSE$
			$TAB$(void *)($MO.BASE[moid]$)
			$SPC$/* $FORMAT("%x", MO.BASEADDR[moid])$ */
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				,$NL$$TAB$(void *)(((char *)($MO.BASE[moid]$)) + ($MO.SIZE[moid]$))
				$SPC$/* $FORMAT("%x", MO.LIMITADDR[moid])$ */
			$END$
		$END$
	$END$$NL$
	};$NL$
	$NL$

$	// meminib_tableの生成
	const MEMINIB _kernel_meminib_table[$tsize_meminib$] = {$NL$
	$IF LENGTH(genzero)$
		$TAB${ TA_NULL, 0U, 0U, 0U },$NL$
	$END$
	$JOINEACH moid MO_MEMTOP_ORDER ",\n"$
		$TAB${
		$IF MO.TYPE[moid] == TOPPERS_USTACK$
			$SPC$TOPPERS_USTACK|($MO.MEMATR[moid]$),
			$SPC$$MO.ACPTN4[moid]$,
			$IF MO.TSKID[moid] != 0$
				$SPC$(ACPTN)(&_kernel_tcb_table[$MO.TSKID[moid]$ - 1]),
			$ELSE$
				$SPC$(ACPTN)(NULL),
			$END$
			$SPC$0U
		$ELSE$
			$IF MO.TYPE[moid] == TOPPERS_ATTMEM$
				$SPC$TOPPERS_ATTMEM|($MO.MEMATR[moid]$),
			$ELSE$
				$SPC$TOPPERS_ATTSEC|($MO.MEMATR[moid]$),
			$END$
			$SPC$$MO.ACPTN4[moid]$,
			$SPC$$MO.ACPTN1[moid]$,
			$SPC$$MO.ACPTN2[moid]$
		$END$
		$SPC$}
		$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
			,$NL$$TAB${ TA_NULL, 0U, 0U, 0U }
		$END$
	$END$$NL$
	};$NL$
	$NL$
$END$

$
$  dataセクション初期化ブロックの生成
$
/*$NL$
$SPC$*  Data Section Management Functions$NL$
$SPC$*/$NL$
$NL$

$IF !OMIT_IDATA && LENGTH(DATASEC_LIST)$
$	// サイズが0でないdataセクションのリスト（MO_DATASEC_LIST）を作成
	$IF !LENGTH(DATASEC_LIST_OPTIMIZED)$
		$MO_DATASEC_LIST = {}$
		$FOREACH moid DATASEC_LIST$
			$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
				$MO_DATASEC_LIST = APPEND(MO_DATASEC_LIST, moid)$
			$END$
		$END$
	$END$

$	// dataセクション初期化ブロックで使うシンボルのextern宣言
	$FOREACH moid MO_DATASEC_LIST$
		extern char __start_$MO.SLABEL[moid]$;$NL$
		extern char __end_$MO.SLABEL[moid]$;$NL$
		extern char __start_$MO.ILABEL[moid]$;$NL$
	$END$$NL$

$	// dataセクションの数
	const uint_t _kernel_tnum_datasec = $LENGTH(MO_DATASEC_LIST)$U;$NL$
	$NL$

$	// dataセクション初期化ブロック
	const DATASECINIB _kernel_datasecinib_table[$LENGTH(DATASEC_LIST)$] = {
	$IF LENGTH(MO_DATASEC_LIST)$
		$NL$
		$JOINEACH moid MO_DATASEC_LIST ",\n"$
			$TAB${ &__start_$MO.SLABEL[moid]$, &__end_$MO.SLABEL[moid]$, 
			&__start_$MO.ILABEL[moid]$ }
		$END$$NL$
	$ELSE$
		{ 0, 0, 0 }
	$END$
	};$NL$
$ELSE$
	const uint_t _kernel_tnum_datasec = 0U;$NL$
	TOPPERS_EMPTY_LABEL(const DATASECINIB, _kernel_datasecinib_table);$NL$
$END$$NL$

$
$  bssセクション初期化ブロックの生成
$
/*$NL$
$SPC$*  BSS Section Management Functions$NL$
$SPC$*/$NL$
$NL$

$IF LENGTH(BSSSEC_LIST)$
$	// サイズが0でないbssセクションのリスト（MO_BSSSEC_LIST）を作成
	$IF !LENGTH(BSSSEC_LIST_OPTIMIZED)$
		$MO_BSSSEC_LIST = {}$
		$FOREACH moid BSSSEC_LIST$
			$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
				$MO_BSSSEC_LIST = APPEND(MO_BSSSEC_LIST, moid)$
			$END$
		$END$
	$END$

$	// bssセクション初期化ブロックで使うシンボルのextern宣言
	$FOREACH moid MO_BSSSEC_LIST$
		extern char __start_$MO.SLABEL[moid]$;$NL$
		extern char __end_$MO.SLABEL[moid]$;$NL$
	$END$$NL$

$	// bssセクションの数
	const uint_t _kernel_tnum_bsssec = $LENGTH(MO_BSSSEC_LIST)$U;$NL$
	$NL$

$	// bssセクション初期化ブロック
	const BSSSECINIB _kernel_bsssecinib_table[$LENGTH(BSSSEC_LIST)$] = {
	$IF LENGTH(MO_BSSSEC_LIST)$
		$NL$
		$JOINEACH moid MO_BSSSEC_LIST ",\n"$
			$TAB${ &__start_$MO.SLABEL[moid]$, &__end_$MO.SLABEL[moid]$ }
		$END$$NL$
	$ELSE$
		{ 0, 0 }
	$END$
	};$NL$
$ELSE$
	const uint_t _kernel_tnum_bsssec = 0U;$NL$
	TOPPERS_EMPTY_LABEL(const BSSSECINIB, _kernel_bsssecinib_table);$NL$
$END$$NL$

$ =====================================================================
$ メモリ保護単位に関する情報の生成
$ =====================================================================
$
$ nummp：メモリ保護領域の数
$ MP.BASEADDR[mpid]：先頭番地
$ MP.LIMITADDR[mpid]：上限番地
$ MP.POFFSET[mpid]：物理アドレスとのオフセット
$ MP.MEMATR[mpid]：メモリオブジェクト属性
$ MP.ACPTN1[mpid]：通常操作1（書込み）のアクセス許可パターン
$ MP.ACPTN2[mpid]：通常操作2（読出し，実行）のアクセス許可パターン
$ MP.DOMAIN[mpid]：属するドメイン（無所属の場合はTDOM_NONE）
$ MP.GLOBAL[mpid]：メモリ保護属性がすべてのユーザドメインで同一（ユーザ
$				   ドメインがない場合も含む）
$ MP.PRIVATE[mpid]：メモリ保護属性が属するユーザドメイン以外で同一（ユー
$					ザドメインに属する場合のみ有効）
$ MP.MOID[mpid]：先頭のメモリオブジェクトのID
$
$ 主にMMUを持つターゲット向けの情報であり，MPUを持つターゲットでは必要
$ ないため，GENERATE_MP_INFOが設定されている時にのみ生成する

$IF LENGTH(GENERATE_MP_INFO)$
	$nummp = 0$
	$prevaddr = 0$
	$FOREACH moid SORT(MO_MPROTECT_LIST, "MO.BASEADDR")$
$		// メモリ保護単位の先頭番地と上限番地を取り出す
		$IF MO.LINKER[moid]$
			$IF EQ(MO.PLABEL[moid], "")$
				$plabel = MO.SLABEL[moid]$
			$ELSE$
				$plabel = MO.PLABEL[moid]$
			$END$
			$baseaddr = SYMBOL(CONCAT("__start_", plabel))$
			$limitaddr = SYMBOL(CONCAT("__limit_", plabel))$
		$ELSE$
			$baseaddr = MO.BASEADDR[moid]$
			$limitaddr = MO.LIMITADDR[moid]$
		$END$

$		// メモリ保護単位に関する情報の生成
		$IF baseaddr != limitaddr && !LENGTH(MO.MERGED[moid])$
			$IF prevaddr < baseaddr$
$				// 未使用領域の登録
				$nummp = nummp + 1$
				$MP.BASEADDR[nummp] = prevaddr$
				$MP.LIMITADDR[nummp] = baseaddr$
			$END$

$			// メモリ保護領域の登録
			$nummp = nummp + 1$
			$MP.BASEADDR[nummp] = baseaddr$
			$MP.LIMITADDR[nummp] = limitaddr$
			$MP.POFFSET[nummp] = MO.POFFSET[moid]$
			$MP.MEMATR[nummp] = MO.MEMATR[moid]$
			$MP.ACPTN1[nummp] = MO.ACPTN1[moid]$
			$MP.ACPTN2[nummp] = MO.ACPTN2[moid]$
			$MP.DOMAIN[nummp] = MO.DOMAIN[moid]$
			$MP.MOID[nummp] = moid$

$			// MP.GLOBAL[nummp]の設定
			$IF (MP.ACPTN1[nummp] == TACP_KERNEL
									|| MP.ACPTN1[nummp] == TACP_SHARED)
					&& (MP.ACPTN2[nummp] == TACP_KERNEL
									|| MP.ACPTN2[nummp] == TACP_SHARED)$
				$MP.GLOBAL[nummp] = 1$
			$END$

$			// MP.PRIVATE[nummp]の設定
			$IF MP.DOMAIN[nummp] > 0$
				$default_acptn = 1 << (MP.DOMAIN[nummp] - 1)$
				$IF (MP.ACPTN1[nummp] == TACP_KERNEL
									|| MP.ACPTN1[nummp] == TACP_SHARED
									|| MP.ACPTN1[nummp] == default_acptn)
						&& (MP.ACPTN2[nummp] == TACP_KERNEL
									|| MP.ACPTN2[nummp] == TACP_SHARED
									|| MP.ACPTN2[nummp] == default_acptn)$
					$MP.PRIVATE[nummp] = 1$
				$END$
			$END$

			$prevaddr = limitaddr$
		$END$
	$END$
	$IF prevaddr != 0$
$		// 未使用領域の登録
		$nummp = nummp + 1$
		$MP.BASEADDR[nummp] = prevaddr$
		$MP.LIMITADDR[nummp] = 0$
	$END$
$END$

$ =====================================================================
$ オブジェクト管理領域等に関するエラーチェック
$ =====================================================================

$
$ メモリ領域を含むメモリオブジェクトのサーチ
$
$FUNCTION SEARCH_MO$
	$_base = ARGV[1]$
	$_limit = (ARGV[1] + ARGV[2]) & ((1 << sizeof_void_ptr * 8) - 1)$
	$IF _limit < _base && _limit != 0$
		$RESULT = 0$
	$ELSE$
		$i = 1$
		$j = LENGTH(MO_MEMTOP_ORDER)$
		$found = 0$

		$WHILE !found && i <= j$
			$k = (i + j) / 2$
			$_moid = AT(MO_MEMTOP_ORDER,k-1)$
			$IF _base < MO.BASEADDR[_moid]$
				$j = k - 1$
			$ELIF _base >= MO.LIMITADDR[_moid] && MO.LIMITADDR[_moid] != 0$
				$i = k + 1$
			$ELSE$
				$found = _moid$
			$END$
		$END$
		$IF found && _limit > MO.LIMITADDR[found] && MO.LIMITADDR[found] != 0$
			$found = 0$
		$END$
		$RESULT = found$
	$END$
$END$

$
$ メモリ領域がメモリオブジェクトに含まれているかのチェック
$
$FUNCTION CHECK_MEMOBJ$
	$IF SEARCH_MO(ARGV[1], ARGV[2])$
		$RESULT = 0$
	$ELSE$		
		$RESULT = 1$
	$END$
$END$

$
$ メモリ領域がカーネル専用のメモリオブジェクトに含まれているかのチェック
$
$FUNCTION CHECK_MEMOBJ_KERNEL$
	$moid = SEARCH_MO(ARGV[1], ARGV[2])$
	$IF moid && MO.ACPTN1[moid] == TACP_KERNEL
				&& MO.ACPTN2[moid] == TACP_KERNEL
				&& MO.ACPTN4[moid] == TACP_KERNEL$
		$RESULT = 0$
	$ELSE$		
		$RESULT = 1$
	$END$
$END$

$
$ システムタスクのスタック領域とユーザタスクのシステムスタック領域が，
$ カーネル専用のメモリオブジェクトに含まれているかのチェック
$
$IF !USE_TSKINICTXB$
	$tinib = SYMBOL("_kernel_tinib_table")$
	$FOREACH tskid TSK.ID_LIST$
		$sstk = PEEK(tinib + offsetof_TINIB_sstk, sizeof_void_ptr)$
		$sstksz = PEEK(tinib + offsetof_TINIB_sstksz, sizeof_SIZE)$
		$IF CHECK_MEMOBJ_KERNEL(sstk, sstksz)$
			$IF TSK.DOMAIN[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$E_OBJ: 
					$FORMAT(_("stack area of `%1%\' is not included in any kernel memory object"), tskid)$
				$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$E_OBJ: 
					$FORMAT(_("system stack area of `%1%\' is not included in any kernel memory object"), tskid)$
				$END$
			$END$
		$END$
		$tinib = tinib + sizeof_TINIB$
	$END$
$END$

$
$ 非タスクコンテキスト用のスタック領域が，カーネル専用のメモリオブジェ
$ クトに含まれているかのチェック
$
$istk = PEEK(SYMBOL("_kernel_istk"), sizeof_void_ptr)$
$istksz = PEEK(SYMBOL("_kernel_istksz"), sizeof_SIZE)$
$IF CHECK_MEMOBJ_KERNEL(istk, istksz)$
	$ERROR ICS.TEXT_LINE[1]$E_OBJ: 
		$FORMAT(_("interrupt context stack area is not included in any kernel memory object"))$
	$END$
$END$

$
$ CRE_MPFで指定した固定長メモリプール領域が，カーネルに登録されたメモリ
$ オブジェクトに含まれているかのチェック
$
$mpfinib = SYMBOL("_kernel_mpfinib_table")$
$FOREACH mpfid MPF.ID_LIST$
	$mpf = PEEK(mpfinib + offsetof_MPFINIB_mpf, sizeof_void_ptr)$
	$blksz = PEEK(mpfinib + offsetof_MPFINIB_blksz, sizeof_uint_t)$
	$blkcnt = PEEK(mpfinib + offsetof_MPFINIB_blkcnt, sizeof_uint_t)$
	$mpfsz = blksz * blkcnt$
	$IF CHECK_MEMOBJ(mpf, mpfsz)$
		$ERROR MPF.TEXT_LINE[mpfid]$E_OBJ: 
			$FORMAT(_("%1% of `%2%\' is not included in any memory object"), "memory pool area", mpfid)$
		$END$
	$END$
	$mpfinib = mpfinib + sizeof_MPFINIB$
$END$

$ =====================================================================
$ アラインメントとNULLでないかに関するエラーチェック
$ =====================================================================

$
$  関数の先頭番地のチェック
$
$IF CHECK_FUNC_ALIGN || CHECK_FUNC_NONNULL$
$	// タスクとタスク例外処理ルーチンの先頭番地のチェック
	$tinib = SYMBOL("_kernel_tinib_table")$
	$FOREACH tskid TSK.ID_LIST$
		$task = PEEK(tinib + offsetof_TINIB_task, sizeof_FP)$
		$IF CHECK_FUNC_ALIGN && (task & (CHECK_FUNC_ALIGN - 1)) != 0$
			$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
				"task", TSK.TASK[tskid], tskid, "CRE_TSK")$$END$
		$END$
		$IF CHECK_FUNC_NONNULL && task == 0$
			$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
				"task", TSK.TASK[tskid], tskid, "CRE_TSK")$$END$
		$END$
		$texrtn = PEEK(tinib + offsetof_TINIB_texrtn, sizeof_FP)$
		$IF CHECK_FUNC_ALIGN && (texrtn & (CHECK_FUNC_ALIGN - 1)) != 0$
			$ERROR DEF_TEX.TEXT_LINE[tskid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
				"texrtn", TSK.TEXRTN[tskid], tskid, "DEF_TEX")$$END$
		$END$
		$tinib = tinib + sizeof_TINIB$
	$END$

$	// 周期ハンドラの先頭番地のチェック
	$cycinib = SYMBOL("_kernel_cycinib_table")$
	$FOREACH cycid CYC.ID_LIST$
		$cychdr = PEEK(cycinib + offsetof_CYCINIB_cychdr, 4)$
		$IF CHECK_FUNC_ALIGN && (cychdr & (CHECK_FUNC_ALIGN - 1)) != 0$
			$ERROR CYC.TEXT_LINE[cycid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
				"cychdr", CYC.CYCHDR[cycid], cycid, "CRE_CYC")$$END$
		$END$
		$IF CHECK_FUNC_NONNULL && cychdr == 0$
			$ERROR CYC.TEXT_LINE[cycid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
				"cychdr", CYC.CYCHDR[cycid], cycid, "CRE_CYC")$$END$
		$END$
		$cycinib = cycinib + sizeof_CYCINIB$
	$END$

$	// アラームハンドラの先頭番地のチェック
	$alminib = SYMBOL("_kernel_alminib_table")$
	$FOREACH almid ALM.ID_LIST$
		$almhdr = PEEK(alminib + offsetof_ALMINIB_almhdr, 4)$
		$IF CHECK_FUNC_ALIGN && (almhdr & (CHECK_FUNC_ALIGN - 1)) != 0$
			$ERROR ALM.TEXT_LINE[almid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
				"almhdr", ALM.ALMHDR[almid], almid, "CRE_ALM")$$END$
		$END$
		$IF CHECK_FUNC_NONNULL && almhdr == 0$
			$ERROR ALM.TEXT_LINE[almid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
				"almhdr", ALM.ALMHDR[almid], almid, "CRE_ALM")$$END$
		$END$
		$alminib = alminib + sizeof_ALMINIB$
	$END$
$END$

$
$  スタック領域の先頭番地のチェック
$
$IF CHECK_STACK_ALIGN || CHECK_STACK_NONNULL$
	$tinib = SYMBOL("_kernel_tinib_table")$
	$FOREACH tskid TSK.ID_LIST$
$		// タスクのシステムスタック領域の先頭番地のチェック
		$IF USE_TSKINICTXB$
			$sstk = GET_SSTK_TSKINICTXB(tinib)$
		$ELSE$
			$sstk = PEEK(tinib + offsetof_TINIB_sstk, sizeof_void_ptr)$
		$END$
		$IF CHECK_STACK_ALIGN && (sstk & (CHECK_STACK_ALIGN - 1)) != 0$
			$IF TSK.DOMAIN[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"stk", TSK.STK[tskid], tskid, "CRE_TSK")$$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"sstk", TSK.SSTK[tskid], tskid, "CRE_TSK")$$END$
			$END$
		$END$
		$IF CHECK_STACK_NONNULL && sstk == 0$
			$IF TSK.DOMAIN[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"stk", TSK.STK[tskid], tskid, "CRE_TSK")$$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"sstk", TSK.SSTK[tskid], tskid, "CRE_TSK")$$END$
			$END$
		$END$

$		// タスクのユーザスタック領域の先頭番地のチェック
		$IF TSK.DOMAIN[tskid] != TDOM_KERNEL$
			$IF USE_TSKINICTXB$
				$ustk = GET_USTK_TSKINICTXB(tinib)$
			$ELSE$
				$ustk = PEEK(tinib + offsetof_TINIB_ustk, sizeof_void_ptr)$
			$END$
			$IF CHECK_USTACK_ALIGN && (ustk & (CHECK_USTACK_ALIGN - 1)) != 0$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"stk", TSK.STK[tskid], tskid, "CRE_TSK")$$END$
			$END$
			$IF CHECK_USTACK_NONNULL && ustk == 0$
				$ERROR TSK.TEXT_LINE[tskid]$E_PAR: 
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"stk", TSK.STK[tskid], tskid, "CRE_TSK")$$END$
			$END$
		$END$

		$tinib = tinib + sizeof_TINIB$
	$END$

$	// 非タスクコンテキスト用のスタック領域の先頭番地のチェック
	$istk = PEEK(SYMBOL("_kernel_istk"), sizeof_void_ptr)$
	$IF CHECK_STACK_ALIGN && (istk & (CHECK_STACK_ALIGN - 1)) != 0$
		$ERROR ICS.TEXT_LINE[1]$E_PAR: 
			$FORMAT(_("%1% `%2%\' in %3% is not aligned"),
			"istk", ICS.ISTK[1], "DEF_ICS")$$END$
	$END$
	$IF CHECK_STACK_NONNULL && istk == 0$
		$ERROR ICS.TEXT_LINE[1]$E_PAR: 
			$FORMAT(_("%1% `%2%\' in %3% is null"),
			"istk", ICS.ISTK[1], "DEF_ICS")$$END$
	$END$
$END$

$
$  固定長メモリプール領域の先頭番地のチェック
$
$IF CHECK_MPF_ALIGN || CHECK_MPF_NONNULL$
$	// 固定長メモリプール領域の先頭番地のチェック
	$mpfinib = SYMBOL("_kernel_mpfinib_table")$
	$FOREACH mpfid MPF.ID_LIST$
		$mpf = PEEK(mpfinib + offsetof_MPFINIB_mpf, sizeof_void_ptr)$
		$IF CHECK_MPF_ALIGN && (mpf & (CHECK_MPF_ALIGN - 1)) != 0$
			$ERROR MPF.TEXT_LINE[mpfid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
				"mpf", MPF.MPF[mpfid], mpfid, "CRE_MPF")$$END$
		$END$
		$IF CHECK_MPF_NONNULL && mpf == 0$
			$ERROR MPF.TEXT_LINE[mpfid]$E_PAR: 
				$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
				"mpf", MPF.MPF[mpfid], mpfid, "CRE_MPF")$$END$
		$END$
		$mpfinib = mpfinib + sizeof_MPFINIB$
	$END$
$END$

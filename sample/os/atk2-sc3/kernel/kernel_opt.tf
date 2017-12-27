$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2011-2015 by Center for Embedded Computing Systems
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
$  $Id: kernel_opt.tf 425 2015-12-07 08:06:19Z witz-itoyo $
$

$
$  パス2からの情報の読込み
$
$INCLUDE "cfg2_out.tf"$
$INCLUDE "kernel/kernel_common.tf"$

$
$  データセクションのLMAからVMAへのコピー
$
$COPY_LMA()$

$ =====================================================================
$ kernel_mem3.cの共通部分の生成
$ =====================================================================

$FILE "kernel_mem3.c"$
/* kernel_mem3.c */$NL$
#include "kernel/kernel_int.h"$NL$
#include "Os_Lcfg.h"$NL$
#ifndef TOPPERS_EMPTY_LABEL$NL$
#define TOPPERS_EMPTY_LABEL(x, y) x y[0]$NL$
#endif$NL$
$NL$

/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$

$ =====================================================================
$ シンボルと仮メモリオブジェクト初期化ブロックの読込み
$ =====================================================================

$
$ シンボルと仮メモリオブジェクト初期化ブロックの読込み
$
$ MO.BASEADDR[moid]：メモリオブジェクトの先頭番地
$ MO.LIMITADDR[moid]：メモリオブジェクトの上限番地

$IF !OMIT_STANDARD_MEMINIB$
	$MO_MEMTOP_LIST = {}$
	$memtop_table = SYMBOL("memtop_table")$
	$offset = 0$
	$FOREACH moid MO_START_LIST$
		$IF MO.LINKER[moid]$
			$MO.BASEADDR[moid] = START_SYMBOL(MO.MLABEL[moid])$
			$MO.LIMITADDR[moid] = LIMIT_SYMBOL(MO.MLABEL[moid])$
		$ELSE$
			$MO.BASEADDR[moid] = PEEK(memtop_table + offset, sizeof_void_ptr)$
			$offset = offset + sizeof_void_ptr$
			$MO.LIMITADDR[moid] = (MO.BASEADDR[moid] + MO.SIZE[moid]) & ((1 << sizeof_void_ptr * 8) - 1)$
		$END$
	$END$
$END$

$ =====================================================================
$ 仮のメモリ構成・初期化ファイルの生成
$ =====================================================================

$
$  仮メモリオブジェクト初期化ブロックの生成
$
$IF ISFUNCTION("GENERATE_MEMINIB_TABLE")$
$	// ターゲット依存部のメモリオブジェクト初期化ブロック生成処理を呼ぶ
	$GENERATE_MEMINIB_TABLE()$
$ELIF !OMIT_STANDARD_MEMINIB$
	$IF LENGTH(OPTIMIZE_MEMINIB)$
$		// アドレス0を置く領域
		$tsize_meminib = 1$
		$FOREACH moid MO_START_LIST$
$			// サイズが0でないメモリオブジェクトの先頭番地を置く領域
			$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
				$tsize_meminib = tsize_meminib + 1$
			$END$
			$IF !MO.LINKER[moid]$
$				// リンカが配置しないメモリオブジェクトは最終番地も必要
				$tsize_meminib = tsize_meminib + 1$
			$ELIF (MO.SEFLAG[MO.MOEND[moid]] & 0x80) != 0$
$				// メモリリージョンの最後のメモリオブジェクトは最終番地も必要
				$tsize_meminib = tsize_meminib + 1$
			$END$
		$END$

$		// ターゲット依存でtsize_meminibを補正する場合
		$IF ISFUNCTION("CALC_TSIZE_MEMINIB")$
			$CALC_TSIZE_MEMINIB()$
		$END$
	$END$

	const uint32 tnum_meminib = $tsize_meminib$U;$NL$
	$NL$

	void *const memtop_table[$tsize_meminib$] = {
	$IF LENGTH(MO_START_LIST_NOLINKER)$
		$NL$
		$JOINEACH moid MO_START_LIST_NOLINKER ",\n"$
			$TAB$(void *)($MO.BASE[moid]$)
		$END$$NL$
	$ELSE$
		$SPC$0$SPC$
	$END$
	};$NL$
	$NL$

	const MEMINIB meminib_table[$tsize_meminib$] =
	$SPC${{ TA_NULL, 0U, 0U, 0U }};$NL$
	$NL$
$END$

$
$  仮dataセクション初期化ブロックの生成
$
$IF LENGTH(OPTIMIZE_DATASEC_LIST) && !OMIT_IDATA && LENGTH(DATASEC_LIST)$
$	// 仮dataセクション初期化ブロックのサイズの適正化処理
$	// サイズが0でないdataセクションのリスト（MO_DATASEC_LIST）を作成
	$MO_DATASEC_LIST = {}$
	$FOREACH moid DATASEC_LIST$
		$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
			$MO_DATASEC_LIST = APPEND(MO_DATASEC_LIST, moid)$
		$END$
	$END$

$	// パス4に引き渡す情報
	$DATASEC_LIST = MO_DATASEC_LIST$
	$DATASEC_LIST_OPTIMIZED = 1$
$END$

$
$	// dataセクションの数とdataセクション初期化ブロック
$
$IF !OMIT_STANDARD_DATASECINIB$
	/*$NL$
	$SPC$*	Data Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

$	// dataセクションの数
	#define TNUM_DATASEC	$IF !OMIT_IDATA$$LENGTH(DATASEC_LIST)$$ELSE$0$END$$NL$
	$NL$

	$IF ISFUNCTION("DEFINE_CONST_VAR")$
		$DEFINE_CONST_VAR("const uint32", "tnum_datasec")$ = TNUM_DATASEC;$NL$
	$ELSE$
		const uint32 tnum_datasec = TNUM_DATASEC;$NL$
	$END$
	$NL$

$	// dataセクション初期化ブロック
	$IF !OMIT_IDATA  && LENGTH(DATASEC_LIST)$
		$IF ISFUNCTION("DEFINE_CONST_VAR")$
			$DEFINE_CONST_VAR("const DATASECINIB", "datasecinib_table[TNUM_DATASEC]")$ =
		$ELSE$
			const DATASECINIB datasecinib_table[TNUM_DATASEC] =
		$END$
		$SPC${{ 0U, 0U, 0U }};$NL$
	$ELSE$
		TOPPERS_EMPTY_LABEL(const DATASECINIB, datasecinib_table);$NL$
		$NL$
	$END$$NL$
$END$

$
$  仮bssセクション初期化ブロックの生成
$
$IF LENGTH(OPTIMIZE_BSSSEC_LIST) && LENGTH(BSSSEC_LIST)$
$	// 仮bssセクション初期化ブロックのサイズの適正化処理
$	// サイズが0でないbssセクションのリスト（MO_BSSSEC_LIST）を作成
	$MO_BSSSEC_LIST = {}$
	$FOREACH moid BSSSEC_LIST$
		$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
			$MO_BSSSEC_LIST = APPEND(MO_BSSSEC_LIST, moid)$
		$END$
	$END$

$	// パス4に引き渡す情報
	$BSSSEC_LIST = MO_BSSSEC_LIST$
	$BSSSEC_LIST_OPTIMIZED = 1$
$END$

$
$	// bssセクションの数とbssセクション初期化ブロック
$
$IF !OMIT_STANDARD_BSSSECINIB$
	/*$NL$
	$SPC$*	BSS Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

$	// bssセクション初期化ブロック
	$IF LENGTH(BSSSEC_LIST)$
		const uint32 tnum_bsssec = $LENGTH(BSSSEC_LIST)$U;$NL$
		const BSSSECINIB bsssecinib_table[$LENGTH(BSSSEC_LIST)$] =
		$SPC${{ 0U, 0U }};$NL$
	$ELSE$
		const uint32 _kernel_tnum_bsssec = 0U;$NL$
		TOPPERS_EMPTY_LABEL(const BSSSECINIB, bsssecinib_table);$NL$
	$END$$NL$
$END$

$ ターゲット依存部で必要なMPUINFOBを出力する
$GENERATE_TARGET_MPUINFOB()$

$ タスクスタック領域をextern宣言する(kernel_common.tf)
$GENERATE_EXPORT_TSK_STK()$

$ タスクをextern宣言する(kernel_common.tf)
$EXTERN_TSK()$

$ OSAP管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_OSAPCB()$

$ タスク管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_TCB()$

$ タスク初期化ブロックを出力する(kernel_common.tf)
$GENERATE_TINIB_TABLE()$

$ OSアプリケーション初期化ブロックを出力する(kernel_common.tf)
$GENERATE_OSAPINIB_TABLE()$

$ =====================================================================
$  パス4に渡す情報の生成
$ =====================================================================

$FILE "cfg3_out.tf"$
$$ cfg3_out.tf$NL$
$NL$

$ tsize_meminibの出力
$IF !OMIT_STANDARD_MEMINIB$
	$$tsize_meminib = $tsize_meminib$$$$NL$
	$NL$
$END$

$ DATASEC_LIST，MO_DATASEC_LIST，DATASEC_LIST_OPTIMIZEDの出力
$$DATASEC_LIST = { $DATASEC_LIST$ }$$$NL$
$IF LENGTH(DATASEC_LIST_OPTIMIZED)$
	$$MO_DATASEC_LIST = { $MO_DATASEC_LIST$ }$$$NL$
	$$DATASEC_LIST_OPTIMIZED = $DATASEC_LIST_OPTIMIZED$$$$NL$
$END$
$NL$

$ BSSSEC_LIST，MO_BSSSEC_LIST，BSSSEC_LIST_OPTIMIZEDの出力
$$BSSSEC_LIST = { $BSSSEC_LIST$ }$$$NL$
$IF LENGTH(BSSSEC_LIST_OPTIMIZED)$
	$$MO_BSSSEC_LIST = { $MO_BSSSEC_LIST$ }$$$NL$
	$$BSSSEC_LIST_OPTIMIZED = $BSSSEC_LIST_OPTIMIZED$$$$NL$
$END$
$NL$

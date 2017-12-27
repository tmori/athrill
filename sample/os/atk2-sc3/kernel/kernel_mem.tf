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
$  $Id: kernel_mem.tf 425 2015-12-07 08:06:19Z witz-itoyo $
$

$  パス2，パス3からの情報の読込み
$
$INCLUDE "cfg2_out.tf"$
$INCLUDE "cfg3_out.tf"$
$INCLUDE "kernel/kernel_common.tf"$

$
$  データセクションのLMAからVMAへのコピー
$
$COPY_LMA()$

$ =====================================================================
$ kernel_mem.cの共通部分の生成
$ =====================================================================

$FILE "kernel_mem.c"$
/* kernel_mem.c */$NL$
#include "kernel/kernel_int.h"$NL$
#include "Os_Lcfg.h"$NL$
#include "kernel_mem.h"$NL$
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
	$memtop_table = SYMBOL("memtop_table")$
	$offset = 0$
	$FOREACH moid MO_START_LIST$
		$IF MO.LINKER[moid]$
			$MO.BASEADDR[moid] = START_SYMBOL(MO.MLABEL[moid])$
			$MO.LIMITADDR[moid] = LIMIT_SYMBOL(MO.MLABEL[moid])$
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
				$ERROR MO.TEXT_LINE[moid]$ 
					$FORMAT(_("%1% `%2%\' is too large"),
								"size", MO.SIZE[moid])$
				$END$
			$ELSE$
				$ERROR MO.TEXT_LINE[moid]$
					$FORMAT(_("illegal memory object size"))$
				$END$
			$END$
$			// ターゲット依存のエラーチェック NOS0813
			$IF ISFUNCTION("HOOK_ERRORCHECK_MEM_PASS4") && 
   			    (MO.TYPE[moid] == TOPPERS_ATTMEM)$
				$HOOK_ERRORCHECK_MEM_PASS4(moid)$
			$END$
		$END$
	$END$
$END$

$
$ メモリオブジェクトの重なりのチェック
$
$ MO.SUPPRESSLIMIT[moid]：次のメモリオブジェクトと隣接している
$ MO.MERGED[moid]：併合されたユーザスタック領域の併合先
$ MO.MERGELAST[moid]：併合された最後のユーザスタック領域（併合先のメモ
$                     リオブジェクトに設定）
$ MO_MEMTOP_ORDER：メモリオブジェクトのベースアドレス順のリスト

$MO_MEMTOP_ORDER = {}$
$prev = 0$
$FOREACH moid SORT(MO_MEMTOP_LIST, "MO.BASEADDR")$
$	// ユーザスタック領域は併合される前の情報を用いてタスクスタックのMPU情報生成
	$IF MO.TYPE[moid] == TOPPERS_USTACK$
		$MO.NOMERGE_BASEADDR[moid] = MO.BASEADDR[moid]$
		$MO.NOMERGE_LIMITADDR[moid] = MO.LIMITADDR[moid]$
	$END$
	$IF !prev || (MO.LIMITADDR[prev] <= MO.BASEADDR[moid]
											&& MO.LIMITADDR[prev] != 0)$
$		// メモリオブジェクトの領域に重なりがない場合
		$IF prev && MO.LIMITADDR[prev] == MO.BASEADDR[moid]$
$			// メモリオブジェクトの領域が連続している場合
			$IF MO.TYPE[moid] == TOPPERS_USTACK
					&& MO.TYPE[prev] == TOPPERS_USTACK
					&& MO.OSAPID[prev] == MO.OSAPID[moid]$
$				// ユーザスタック領域の併合処理（連続している場合）
				$MO.BASE[moid] = MO.BASE[prev]$
				$MO.BASEADDR[moid] = MO.BASEADDR[prev]$
				$MO.SIZE[prev] = MO.LIMITADDR[moid] - MO.BASEADDR[prev]$
				$MO.LIMITADDR[prev] = MO.LIMITADDR[moid]$
				$MO.SIZE[moid] = MO.SIZE[prev]$
				$MO.MERGED[moid] = prev$
				$MO.MERGELAST[prev] = moid$
			$ELSE$
				$MO.SUPPRESSLIMIT[prev] = 1$
				$MO_MEMTOP_ORDER = APPEND(MO_MEMTOP_ORDER, moid)$
				$prev = moid$
			$END$
		$ELSE$
$			// メモリオブジェクトの領域が連続していない場合 
			$MO_MEMTOP_ORDER = APPEND(MO_MEMTOP_ORDER, moid)$
			$prev = moid$
		$END$
	$ELSE$
$		// メモリオブジェクトの領域に重なりがある場合
		$IF MO.TYPE[moid] == TOPPERS_USTACK && !MO.LINKER[moid]
			&& MO.TYPE[prev] == TOPPERS_USTACK && !MO.LINKER[prev]
			&& MO.OSAPID[prev] == MO.OSAPID[moid]$
$			// ユーザスタック領域の併合処理（重なりがある場合） 
			$MO.TSKID[prev] = 0$
			$MO.TSKID[moid] = 0$
			$MO.BASE[moid] = MO.BASE[prev]$
			$MO.BASEADDR[moid] = MO.BASEADDR[prev]$
			$IF MO.LIMITADDR[prev] < MO.LIMITADDR[moid]
											&& MO.LIMITADDR[prev] != 0$
				$MO.SIZE[prev] = MO.LIMITADDR[moid] - MO.BASEADDR[prev]$
				$MO.LIMITADDR[prev] = MO.LIMITADDR[moid]$
			$ELSE$
				$MO.LIMITADDR[moid] = MO.LIMITADDR[prev]$
			$END$
			$MO.SIZE[moid] = MO.SIZE[prev]$
			$MO.MERGED[moid] = prev$
			$MO.MERGELAST[prev] = moid$
		$ELSE$
$			// メモリオブジェクトの領域に重なりがある場合
			$IF !(MO.TYPE[moid] == TOPPERS_USTACK && MO.TYPE[prev] == TOPPERS_USTACK)$
$				// エラーメッセージの出力
				$IF MO.TYPE[moid] == TOPPERS_ATTMEM$
					$ERROR MO.TEXT_LINE[moid]$
						$FORMAT(_("memory object overlaps with another memory object"))$
					$END$
				$ELIF MO.TYPE[prev] == TOPPERS_ATTMEM$
					$ERROR MO.TEXT_LINE[prev]$
						$FORMAT(_("memory object overlaps with another memory object"))$
					$END$
				$ELIF MO.TYPE[moid] == TOPPERS_USTACK$
					$ERROR MO.TEXT_LINE[moid]$
						$FORMAT(_("user stack area overlaps with another memory object"))$
					$END$
				$ELIF MO.TYPE[prev] == TOPPERS_USTACK$
					$ERROR MO.TEXT_LINE[prev]$
						$FORMAT(_("user stack area overlaps with another memory object"))$
					$END$
				$ELSE$
					$ERROR MO.TEXT_LINE[moid]$
						$FORMAT(_("memory objects overlap"))$
					$END$
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
		$ERROR$$FORMAT(_("illegal memory object initialization block size"))$$END$
	$END$

$	// tnum_meminibの生成
	const uint32 tnum_meminib = $tnum_meminib$U;$NL$
	$NL$

$	// memtop_tableの生成
	void *const memtop_table[$tsize_meminib$] = {$NL$
	$IF LENGTH(genzero)$
		$TAB$0,$NL$
	$END$
	$JOINEACH moid MO_MEMTOP_ORDER ",\n"$
		$IF MO.LINKER[moid]$
			$TAB$&__start_$MO.MLABEL[moid]$
			$SPC$/* $FORMAT("%x", MO.BASEADDR[moid])$ */
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				 ,$NL$$TAB$&__limit_$MO.MLABEL[ALT(MO.MERGELAST[moid],moid)]$ 
				$SPC$/* $FORMAT("%x", MO.LIMITADDR[moid])$ */
			$END$
		$ELSE$
			$TAB$(void *)($MO.BASE[moid]$)
			$SPC$/* $FORMAT("%x", MO.BASEADDR[moid])$ */
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				,$NL$$TAB$(void *)(((uint8 *)($MO.BASE[moid]$)) + ($MO.SIZE[moid]$))
				$SPC$/* $FORMAT("%x", MO.LIMITADDR[moid])$ */
			$END$
		$END$
	$END$
	$IF tnum_meminib < tsize_meminib$
		$FOREACH i RANGE(tnum_meminib + 1, tsize_meminib)$
			,$NL$$TAB$NULL
		$END$
	$END$$NL$
	};$NL$
	$NL$

$	// meminib_tableの生成
	const MEMINIB meminib_table[$tsize_meminib$] = {$NL$
	$IF LENGTH(genzero)$
		$TAB${ TA_NULL, TACP_KERNEL, TACP_KERNEL, TACP_KERNEL },$NL$
	$END$
	$JOINEACH moid MO_MEMTOP_ORDER ",\n"$
		$TAB${
		$IF MO.TYPE[moid] == TOPPERS_USTACK$
			$SPC$TOPPERS_USTACK|($MO.MEMATR[moid]$),
		$ELSE$
			$IF MO.TYPE[moid] == TOPPERS_ATTMEM$
				$SPC$TOPPERS_ATTMEM|($FORMAT("0x%xU", +MO.MEMATR[moid])$),
			$ELSE$
				$SPC$TOPPERS_ATTSEC|($FORMAT("0x%xU", +MO.MEMATR[moid])$),
			$END$
		$END$
		$SPC$$MO.ACPTN_R[moid]$,
		$SPC$$MO.ACPTN_W[moid]$,
		$SPC$$MO.ACPTN_X[moid]$
		$SPC$}
		$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
			,$NL$$TAB${ TA_NULL, TACP_KERNEL, TACP_KERNEL, TACP_KERNEL }
		$END$
	$END$
	$IF tnum_meminib < tsize_meminib$
		$FOREACH i RANGE(tnum_meminib + 1, tsize_meminib)$
			,$NL$$TAB${ 0U, 0U, 0U, 0U }
		$END$
	$END$$NL$
	};$NL$
	$NL$
$END$

$
$  dataセクション初期化ブロックの生成
$
$IF !OMIT_IDATA && LENGTH(DATASEC_LIST) && !LENGTH(DATASEC_LIST_OPTIMIZED)$
$	// サイズが0でないdataセクションのリスト（MO_DATASEC_LIST）を作成
$	// パス3で作成していない場合には，ここで作成する．
	$MO_DATASEC_LIST = {}$
	$FOREACH moid DATASEC_LIST$
		$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
			$MO_DATASEC_LIST = APPEND(MO_DATASEC_LIST, moid)$
		$END$
	$END$
$END$

$IF !OMIT_STANDARD_DATASECINIB$
	/*$NL$
	$SPC$*	Data Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

	$IF !OMIT_IDATA && LENGTH(DATASEC_LIST)$
$	    // dataセクション初期化ブロックで使うシンボルのextern宣言
        $FOREACH moid MO_DATASEC_LIST$
            extern uint8 __start_$MO.SLABEL[moid]$;$NL$
            extern uint8 __end_$MO.SLABEL[moid]$;$NL$
            extern uint8 __start_$MO.ILABEL[moid]$;$NL$
        $END$$NL$

$	    // dataセクションの数
        #define TNUM_DATASEC		$IF !OMIT_IDATA$$LENGTH(MO_DATASEC_LIST)$U$ELSE$0$END$$NL$
        #define TNUM_DATASEC_TBL	$IF !OMIT_IDATA$$LENGTH(DATASEC_LIST)$U$ELSE$0$END$$NL$
        $NL$

        $IF ISFUNCTION("DEFINE_CONST_VAR")$
            $DEFINE_CONST_VAR("const uint32", "tnum_datasec")$ = TNUM_DATASEC;$NL$
        $ELSE$
            const uint32 tnum_datasec = TNUM_DATASEC;$NL$
        $END$
        $NL$

		$IF ISFUNCTION("DEFINE_CONST_VAR")$
			$DEFINE_CONST_VAR("const DATASECINIB", "datasecinib_table[TNUM_DATASEC_TBL]")$ = {
		$ELSE$
			const DATASECINIB datasecinib_table[TNUM_DATASEC_TBL] = {
		$END$
		$IF LENGTH(MO_DATASEC_LIST)$
			$NL$
			$JOINEACH moid MO_DATASEC_LIST ",\n"$
				$TAB${ &__start_$MO.SLABEL[moid]$, &__end_$MO.SLABEL[moid]$, 
				&__start_$MO.ILABEL[moid]$ }
			$END$$NL$
		$ELSE$
			{ 0U, 0U, 0U }
		$END$
		};$NL$
	$ELSE$
        const uint32 tnum_datasec = 0;$NL$
		TOPPERS_EMPTY_LABEL(const DATASECINIB, datasecinib_table);$NL$
	$END$$NL$
$END$

$
$  bssセクション初期化ブロックの生成
$
$IF LENGTH(BSSSEC_LIST) && !LENGTH(BSSSEC_LIST_OPTIMIZED)$
$	// サイズが0でないbssセクションのリスト（MO_BSSSEC_LIST）を作成
	$MO_BSSSEC_LIST = {}$
	$FOREACH moid BSSSEC_LIST$
		$IF MO.BASEADDR[moid] != MO.LIMITADDR[moid]$
			$MO_BSSSEC_LIST = APPEND(MO_BSSSEC_LIST, moid)$
		$END$
	$END$
$END$

$IF !OMIT_STANDARD_BSSSECINIB$
	/*$NL$
	$SPC$*	BSS Section Management Functions$NL$
	$SPC$*/$NL$
	$NL$

	$IF LENGTH(BSSSEC_LIST)$
$	    // bssセクション初期化ブロックで使うシンボルのextern宣言
        $FOREACH moid MO_BSSSEC_LIST$
            extern uint8 __start_$MO.SLABEL[moid]$;$NL$
            extern uint8 __end_$MO.SLABEL[moid]$;$NL$
        $END$$NL$

$		// bssセクションの数
		const uint32 tnum_bsssec = $LENGTH(MO_BSSSEC_LIST)$U;$NL$
		$NL$

$		// bssセクション初期化ブロック
		const BSSSECINIB bsssecinib_table[$LENGTH(BSSSEC_LIST)$] = {
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
		const uint32 tnum_bsssec = 0U;$NL$
		TOPPERS_EMPTY_LABEL(const BSSSECINIB, bsssecinib_table);$NL$
	$END$$NL$
$END$

$ ターゲット依存部で必要なMPUINFOBを出力する
$GENERATE_TARGET_MPUINFOB()$

$FOREACH moid MO_MEMTOP_LIST$
$	// ユーザスタック領域は併合される前の情報を用いてタスクスタックのMPU情報生成
	$IF MO.TYPE[moid] == TOPPERS_USTACK$
$		// 併合された情報を退避して，タスク初期化ブロック生成後に復帰
		$tmp = MO.BASEADDR[moid]$
		$MO.BASEADDR[moid] = MO.NOMERGE_BASEADDR[moid]$
		$MO.NOMERGE_BASEADDR[moid] = tmp$
		$tmp = MO.LIMITADDR[moid]$
		$MO.LIMITADDR[moid] = MO.NOMERGE_LIMITADDR[moid]$
		$MO.NOMERGE_LIMITADDR[moid] = tmp$
	$END$
$END$

$ タスク初期化ブロックを出力する(kernel_common.tf)
$GENERATE_TINIB_TABLE()$

$FOREACH moid MO_MEMTOP_LIST$
$	// 併合された情報を復帰
	$IF MO.TYPE[moid] == TOPPERS_USTACK$
		$MO.BASEADDR[moid] = MO.NOMERGE_BASEADDR[moid]$
		$MO.LIMITADDR[moid] = MO.NOMERGE_LIMITADDR[moid]$
	$END$
$END$

$ OSアプリケーション初期化ブロックを出力する(kernel_common.tf)
$GENERATE_OSAPINIB_TABLE()$

$FILE "kernel_mem.h"$
/* kernel_mem.h */$NL$
#ifndef TOPPERS_KERNEL_MEM_H$NL$
#define TOPPERS_KERNEL_MEM_H$NL$
/*$NL$
$SPC$*  Include Directives (#include)$NL$
$SPC$*/$NL$
$NL$
$INCLUDES$
$NL$
$NL$

$IF !ISFUNCTION("GENERATE_MEMINIB_TABLE") && !OMIT_STANDARD_MEMINIB$
$	// memtop_tableで参照するラベルの宣言の生成
	$FOREACH moid MO_MEMTOP_ORDER$
		$IF MO.LINKER[moid]$
			extern uint8 __start_$MO.MLABEL[moid]$;$NL$
			$IF !LENGTH(MO.SUPPRESSLIMIT[moid])$
				extern uint8 __limit_$MO.MLABEL[ALT(MO.MERGELAST[moid],moid)]$;
				$NL$
			$END$
		$END$
	$END$$NL$
$END$

$ タスクスタック領域をextern宣言する(kernel_common.tf)
$GENERATE_EXPORT_TSK_STK()$

$ タスクをextern宣言する(kernel_common.tf)
$EXTERN_TSK()$

$ OSAP管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_OSAPCB()$

$ タスク管理ブロックをextern宣言する(kernel_common.tf)
$EXTERN_TCB()$

#endif /* TOPPERS_KERNEL_MEM_H */$NL$

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
$ メモリ領域がカーネル専用のメモリオブジェクトに含まれているかのチェック
$
$FUNCTION CHECK_MEMOBJ_KERNEL$
	$moid = SEARCH_MO(ARGV[1], ARGV[2])$
	$IF moid && MO.ACPTN1[moid] == TACP_KERNEL
				&& MO.ACPTN2[moid] == TACP_KERNEL$
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
	$tinib = SYMBOL("tinib_table")$
	$FOREACH tskid TSK.ID_LIST$
		$sstk = PEEK(tinib + offsetof_TINIB_sstk, sizeof_void_ptr)$
		$sstksz = PEEK(tinib + offsetof_TINIB_sstksz, sizeof_MemorySizeType)$
		$IF CHECK_MEMOBJ_KERNEL(sstk, sstksz)$
			$IF TSK.OSAPID[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("stack area of `%1%\' is not included in any kernel memory object"), tskid)$
				$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$
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
$istk = PEEK(SYMBOL("_ostk"), sizeof_void_ptr)$
$istksz = PEEK(SYMBOL("_ostksz"), sizeof_StackType)$
$IF CHECK_MEMOBJ_KERNEL(istk, istksz)$
	$ERROR OSTK.TEXT_LINE[1]$ 
		$FORMAT(_("interrupt context stack area is not included in any kernel memory object"))$
	$END$
$END$

$ =====================================================================
$ アラインメントとNULLでないかに関するエラーチェック
$ =====================================================================

$
$  スタック領域の先頭番地のチェック
$
$IF CHECK_STACK_ALIGN || CHECK_STACK_NONNULL$
	$tinib = SYMBOL("tinib_table")$
	$FOREACH tskid TSK.ID_LIST$
$		// タスクのシステムスタック領域の先頭番地のチェック NOS1309
		$IF USE_TSKINICTXB$
			$sstk = GET_SSTK_TSKINICTXB(tinib)$
		$ELSE$
			$sstk = PEEK(tinib + offsetof_TINIB_sstk, sizeof_void_ptr)$
		$END$
		$IF CHECK_STACK_ALIGN && (sstk & (CHECK_STACK_ALIGN - 1)) != 0$
			$IF TSK.OSAPID[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"stk", TSK.STK[tskid], tskid, "OsTask")$$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"sstk", TSK.SSTK[tskid], tskid, "OsTask")$$END$
			$END$
		$END$
		$IF CHECK_STACK_NONNULL && sstk == 0$
			$IF TSK.OSAPID[tskid] == TDOM_KERNEL$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"stk", TSK.STK[tskid], tskid, "OsTask")$$END$
			$ELSE$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"sstk", TSK.SSTK[tskid], tskid, "OsTask")$$END$
			$END$
		$END$

$		// タスクのユーザスタック領域の先頭番地のチェック NOS1304
		$IF !OSAP.TRUSTED[TSK.OSAPID[tskid]]$
			$IF USE_TSKINICTXB$
				$ustk = GET_USTK_TSKINICTXB(tinib)$
			$ELSE$
				$ustk = PEEK(tinib + offsetof_TINIB_ustk, sizeof_void_ptr)$
			$END$
			$IF CHECK_USTACK_ALIGN && (ustk & (CHECK_USTACK_ALIGN - 1)) != 0$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is not aligned"),
					"stk", TSK.STK[tskid], tskid, "OsTask")$$END$
			$END$
			
$		//OSが生成したリスタートタスクの場合はユーザスタックの先頭番地がNULLでも良い
			$IF (CHECK_USTACK_NONNULL && ustk == 0) && tskid < tmin_os_restarttask$
				$ERROR TSK.TEXT_LINE[tskid]$
					$FORMAT(_("%1% `%2%\' of `%3%\' in %4% is null"),
					"stk", TSK.STK[tskid], tskid, "OsTask")$$END$
			$END$
		$END$

		$tinib = tinib + sizeof_TINIB$
	$END$

$	// 非タスクコンテキスト用のスタック領域の先頭番地のチェック NOS1301
	$istk = PEEK(SYMBOL("_ostk"), sizeof_void_ptr)$
	$IF CHECK_STACK_ALIGN && (istk & (CHECK_STACK_ALIGN - 1)) != 0$
		$ERROR OSTK.TEXT_LINE[1]$
			$FORMAT(_("%1% `%2%\' in %3% is not aligned"),
			"istk", OSTK.STK[1], "OsOsStack")$$END$
	$END$
	$IF CHECK_STACK_NONNULL && istk == 0$
		$ERROR OSTK.TEXT_LINE[1]$
			$FORMAT(_("%1% `%2%\' in %3% is null"),
			"istk", OSTK.STK[1], "OsOsStack")$$END$
	$END$
$END$

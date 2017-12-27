$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2013-2014 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2013-2014 by FUJI SOFT INCORPORATED, JAPAN
$  Copyright (C) 2013-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
$  Copyright (C) 2013-2014 by Renesas Electronics Corporation, JAPAN
$  Copyright (C) 2013-2014 by Sunny Giken Inc., JAPAN
$  Copyright (C) 2013-2014 by TOSHIBA CORPORATION, JAPAN
$  Copyright (C) 2013-2014 by Witz Corporation, JAPAN
$
$  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
$  $Id: prc_common.tf 834 2017-09-23 02:19:10Z ertl-honda $
$

$
$  有効な割込み番号
$
$ 
$ V850ではリセット，NMI，WDTは割込みに分類されるが，リセットは
$ カーネルが用いるため除外する

$INTNO_VALID = {}$
$FOREACH intno RANGE(0, TNUM_INT - 1)$
	$INTNO_VALID = APPEND(INTNO_VALID, intno)$
$END$

$EXCNO_VALID = {1,2,3,4,5,7}$

$TNUM_EXC = {1,2,3,4,5,6,7}$

$
$  CRE_ISR2で使用できる割込み番号
$
$INTNO_CREISR2_VALID = INTNO_VALID$

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
$ C2ISRの優先度下限
$
$n = +0$
$pmr_isr2_mask = +0xffff$
$WHILE (n < (MIN_PRI_ISR2 + TNUM_INTPRI))$
$pmr_isr2_mask = pmr_isr2_mask & ~(0x01 << n)$
$n = n + 1$
$END$
$pmr_isr1_mask = ~pmr_isr2_mask & 0xffff $

const uint16 kernel_pmr_isr2_mask = $FORMAT("0x%x",pmr_isr2_mask)$;$NL$
const uint16 kernel_pmr_isr1_mask = $FORMAT("0x%x",pmr_isr1_mask)$;$NL$

$
$  割込みハンドラテーブル(EIレベル　マスカブル割込み用)
$
const FunctionRefType kernel_isr_tbl[TNUM_INT] = {$NL$
$JOINEACH intno INTNO_VALID "\n"$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid)$
		$TAB$$ISR.INT_ENTRY[isrid]$
	$ELSE$
		$TAB$kernel_default_int_handler
	$END$
$	//カンマの出力（最後の要素の後ろに出力しない）
	$IF intno != AT(INTNO_VALID,LENGTH(INTNO_VALID) - 1)$
		,
	$END$
	$TAB$$FORMAT("/* %d */", intno)$
$END$
$NL$};$NL$
$NL$

$
$  ISRCBの取得テーブル(EIレベル　マスカブル割込み用)
$
ISRCB *const kernel_isr_p_isrcb_tbl[TNUM_INT] = {$NL$
$JOINEACH intno INTNO_VALID "\n"$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
		$TAB$&(kernel_isrcb_table[$ISR.ID[isrid]$])
	$ELSE$
		$TAB$NULL
	$END$
$	//カンマの出力（最後の要素の後ろに出力しない）
	$IF intno != AT(INTNO_VALID,LENGTH(INTNO_VALID) - 1)$
		,
	$END$
	$TAB$$FORMAT("/* %d */", intno)$
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
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No2
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 2)$$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No3
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x", 3)$$NL$
$TAB$jr _fe_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No4
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 4)$$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No5
$TAB$$ASM_COMMENT()$$FORMAT(" Exception 0x%x ", 5)$$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
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
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
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
			$TAB$addi	$intno$,r0, r2$NL$
			$TAB$jr	_kernel_interrupt$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
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
		$TAB$addi	$intno$,r0, r2$NL$
		$TAB$jr _default_int_handler$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$END$
$END$

$END$


$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2013-2015 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
$  Copyright (C) 2013-2014 by FUJISOFT INCORPORATED, JAPAN
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
$  $Id: prc_common.tf 845 2017-09-25 04:33:56Z ertl-honda $
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
			$CONCAT("kernel_inthdr_", FORMAT("0x%x",+intno))$(void)$NL$
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
$ ICIの優先度を考慮する必要があるため,MAX_PRI_ISR1を使用する
$min_pri_isr2_system = MAX_PRI_ISR1 + 1$

$n = +0$
$pmr_isr2_mask = +0xffff$
$WHILE (n < (min_pri_isr2_system + TNUM_INTPRI))$
$pmr_isr2_mask = pmr_isr2_mask & ~(0x01 << n)$
$n = n + 1$
$END$
$pmr_isr1_mask = ~pmr_isr2_mask & 0xffff $

const uint16 kernel_pmr_isr2_mask = $FORMAT("0x%x",pmr_isr2_mask)$;$NL$
const uint16 kernel_pmr_isr1_mask = $FORMAT("0x%x",pmr_isr1_mask)$;$NL$$NL$

$
$  割込みハンドラテーブル
$
$FOREACH coreid RANGE(0, TMAX_COREID)$
const FunctionRefType kernel_core$coreid$_isr_tbl[TNUM_INT] = {$NL$
$FOREACH intno INTNO_VALID$
	$IF (((intno & 0xffff0000) == ((coreid+1) << 16)) || ((intno & 0xffff0000) == 0xffff0000))$
		$isrid = INT.ISRID[intno]$
		$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_2")  && (OSAP.CORE[ISR.OSAPID[isrid]] == coreid)$
			$TAB$&$ISR.INT_ENTRY[isrid]$
		$ELIF LENGTH(FIND(INTNO_ICI_LIST, intno))$
$			//コア間割割込みハンドラ
			$TAB$&$CONCAT("kernel_inthdr_", FORMAT("0x%x",+intno))$
		$ELSE$
			$TAB$&kernel_default_int_handler
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

const uint32 kernel_isr_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(const uint32) kernel_core$coreid$_isr_tbl
$END$
$NL$};$NL$
$NL$


$
$  ISRCBの取得テーブル
$
$FOREACH coreid RANGE(0, TMAX_COREID)$
ISRCB * const kernel_core$coreid$_isr_p_isrcb_tbl[TNUM_INT] = {$NL$
$FOREACH intno INTNO_VALID$
	$IF ((intno & 0xffff0000) == ((coreid+1) << 16)) || ((intno & 0xffff0000) == 0xffff0000) $
		$isrid = INT.ISRID[intno]$
		$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_2") && (OSAP.CORE[ISR.OSAPID[isrid]] == coreid)$
			$TAB$&kernel_isrcb_$isrid$
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

const uint32 kernel_isr_p_isrcb_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(const uint32) kernel_core$coreid$_isr_p_isrcb_tbl
$END$
$NL$};$NL$
$NL$

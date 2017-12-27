$
$  TOPPERS ATK2
$      Toyohashi Open Platform for Embedded Real-Time Systems
$      Automotive Kernel Version 2
$
$  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
$              Graduate School of Information Science, Nagoya Univ., JAPAN
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
$  $Id: prc.tf 845 2017-09-25 04:33:56Z ertl-honda $
$

$
$     パス2のアーキテクチャ依存テンプレート（V850用）
$

$INCLUDE "arch/v850_gcc/prc_common.tf"$

$
$ テーブル参照方式用ベクタテーブル(v850e3v5)
$

$FOREACH coreid RANGE(0, TMAX_COREID)$

$	// ハードウェア上の割込み番号毎にC1ISRの関数名を取得する

$	// "interrupt"で初期化
$int_init = {}$
$FOREACH intno INTNO_VALID$
	$IF ((intno & 0xffff0000) == ((coreid+1) << 16))$
		$int_init = APPEND(int_init, VALUE("interrupt", intno))$

	$END$
$END$
$FOREACH intno INTNO_VALID$
	$IF ((intno & 0xffff0000) == 0xffff0000)$
		$int_init = APPEND(int_init, VALUE("interrupt", intno))$
	$END$
$END$

$int_handler = {}$
$FOREACH inthdr int_init$
	$c1isr_cnt = 0$
	$intno = inthdr$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid) && EQ(ISR.CATEGORY[isrid], "CATEGORY_1") && (OSAP.CORE[ISR.OSAPID[isrid]] == coreid)$
		$c1isr_cnt = c1isr_cnt + 1$
		$c1isr_info = VALUE(ISR.INT_ENTRY[isrid], +inthdr)$
	$END$

	$IF c1isr_cnt == 0$
		$int_handler = APPEND(int_handler, VALUE("kernel_interrupt", +inthdr))$
	$ELIF c1isr_cnt == 1$
		$int_handler = APPEND(int_handler, c1isr_info)$
	$ELSE$
		$ERROR$$FORMAT(_("intno:%1% is conflicted"), +inthdr)$$END$
	$END$
$END$

extern void kernel_interrupt(void);$NL$
const uint32 __attribute__((aligned(512))) kernel_core$coreid$_intbp_tbl[TNUM_INT] = {$NL$
$JOINEACH inthdr int_handler "\n"$
	$TAB$(uint32)&$inthdr$
$	//カンマの出力（最後の要素の後ろに出力しない）
		$IF (inthdr & 0xffff) < TMAX_INTNO$
			,
		$END$
	$TAB$$FORMAT("/* 0x%x */", +inthdr)$
$END$
$NL$};$NL$

$END$

$NL$
const uint32 kernel_intbp_table[TotalNumberOfCores] = {$NL$
$JOINEACH coreid RANGE(0, TMAX_COREID) ",\n"$
	$TAB$(const uint32) kernel_core$coreid$_intbp_tbl
$END$
$NL$};
$NL$

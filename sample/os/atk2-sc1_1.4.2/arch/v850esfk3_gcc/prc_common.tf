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
$  $Id: prc_common.tf 540 2015-12-29 01:00:01Z ertl-honda $
$

$
$  有効な割込み番号
$
$ <TRACE> (7-2-1-1) INTNO_CREISR2_VALID
$ <TRACE> (7-2-1-2) INTNO_VALID
$ V850ではリセット，NMI，WDTは割込みに分類されるが，リセットは
$ カーネルが用いるため除外する

$INTNO_VALID = {}$
$INTNO_CREISR2_VALID = {}$
$INTNO_INVALID = 54 $
$FOREACH intno RANGE(0, TNUM_INT - 1)$
	$INTNO_VALID = APPEND(INTNO_VALID, intno)$
	$INTNO_CREISR2_VALID = APPEND(INTNO_CREISR2_VALID, intno)$
$END$

$EXCNO_VALID = {1,2,3,4,5,7}$

$TNUM_EXC = {1,2,3,4,5,6,7}$


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
$n = 0$
$ispr_isr2_mask = 0xff$
$WHILE (n < (MIN_PRI_ISR2 + TNUM_INTPRI))$
	$ispr_isr2_mask = ispr_isr2_mask & ~(0x01 << n)$
	$n = n + 1$
$END$
$ispr_isr1_mask = ~ispr_isr2_mask & 0xff $

const uint8 ispr_isr2_mask = $FORMAT("0x%x",ispr_isr2_mask)$;$NL$
const uint8 ispr_isr1_mask = $FORMAT("0x%x",ispr_isr1_mask)$;$NL$


$CPU_INTPRI_MAX = 0$
$CPU_INTPRI_MIN = 8$
$CPU_INTPRI_RANGE = {0,1, ... ,8}$
$IMR_RANGE = {0,1,2,3,4,5,6,7}$
$C2ISR_IINTPRI = 8$

const uint16_t imr_table[][IMR_SIZE] = $NL$
{$NL$
$JOINEACH intlvl CPU_INTPRI_RANGE " , \n"$
	$FOREACH imrno IMR_RANGE$
		$IMRn[imrno] = 0xFFFF$
	$END$
	
	$FOREACH intno INTNO_VALID$
		$isrid = INT.ISRID[intno]$
		$IF (LENGTH(isrid))$
		  $IF ((EQ(ISR.CATEGORY[isrid], "CATEGORY_1")) || (ISR.INTPRI[isrid]) > (CPU_INTPRI_MIN - intlvl))$
			$OFFSET = (intno) / 16$
			$BITPOS = (intno) % 16$
			$IMRn[OFFSET] = IMRn[OFFSET] & ~(1 << BITPOS)$
		    $FORMAT ("/**** BG:intno=%d, prio=%d ****/", intno, ISR.INTPRI[isrid])$
		  $ELSE$
		    $FORMAT ("/**** LE:intno=%d, prio=%d ****/", intno, ISR.INTPRI[isrid])$
		  $END$
		  
		  $IF ((EQ(ISR.CATEGORY[isrid], "CATEGORY_2")))$
		  	$IF ( (ISR.INTPRI[isrid]) > (CPU_INTPRI_MIN - C2ISR_IINTPRI) )$
		  		$C2ISR_IINTPRI = (CPU_INTPRI_MIN - ISR.INTPRI[isrid])$
		 	$END$
		  $END$
		  
		$END$
	$END$
	$TAB${$SPC$
	$JOINEACH imrno IMR_RANGE " , "$
		$FORMAT("0x%1$x" , +IMRn[imrno])$
	$END$
	$SPC$} $TAB$$FORMAT("/* CPU_INTPRI:%d, ATK2_EXT_INTPRI:%d */", intlvl, (CPU_INTPRI_MIN - intlvl))$
$END$
$NL$};$NL$

const uint8 c2isr_iintpri  = $C2ISR_IINTPRI$;$NL$

$
$  ISRCBの取得テーブル(マスカブル割込み用)
$
ISRCB *const isr_p_isrcb_tbl[TNUM_INT] = {$NL$
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


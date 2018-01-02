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
$ 割込みベクタと各割込み入口処理
$

$FILE "Os_Lcfg_asm.S"$


$
$ アセンブラ出力用の関数群
$


$FUNCTION ASM_GLOBAL$
	$TAB$.global $ARGV[1]$
$END$

$FUNCTION ASM_COMMENT$
	//
$END$


#include <v850asm.inc>$NL$$NL$

$TAB$.section .vectors,"ax"$NL$
$TAB$.align	4$NL$

$
$ 割込みベクタと各割込み入口処理
$
$ASM_GLOBAL("__reset")$
$NL$
__reset:
$NL$
$TAB$$ASM_COMMENT()$
$FORMAT(" Exception(RESET) 0x%x ", 0)$$NL$
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
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(NMI) 0x%x ", 1)$$NL$
$TAB$jr _default_int_handler$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No2
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(INTWDT2) 0x%x ", 2)$$NL$
$TAB$jr _default_int_handler$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No3
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(NOP) 0x%x", 3)$$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No4
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(TRAP0) 0x%x ", 4)$$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No5
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(TRAP1) 0x%x ", 5)$$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No6
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(ILGOP/DBG0) 0x%x ", 6)$$NL$
$TAB$jr _ei_exception_entry$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$ 例外 No7
$TAB$$ASM_COMMENT()$$FORMAT(" Exception(NOP) 0x%x ", 7)$$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$

$
$ ベクタテーブル(マスカブル割込み用)
$
$NL$$FOREACH intno INTNO_VALID$
$TAB$$ASM_COMMENT()$$FORMAT("0x%x",intno*16 + 0x80)$ $NL$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid)$
		$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
			$TAB$$FORMAT("jr __kernel_c2isr_interrupt_%d",intno)$$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
		$ELSE$
			$TAB$$FORMAT("jr __kernel_c1isr_interrupt_%d",intno)$$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
		$END$
	$ELSE$
$		// 割込みハンドラの登録がない場合
			$TAB$jr _default_int_handler$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
			$TAB$nop$NL$
	$END$
$END$

$NL$

$TAB$.section .text , "ax"$NL$
$TAB$.align 4$NL$

$FOREACH intno INTNO_VALID$
	$isrid = INT.ISRID[intno]$
	$IF LENGTH(isrid)$
		$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_2")$
			$TAB$.globl	_$ISR.INT_ENTRY[isrid]$$NL$
			$TAB$.globl	$FORMAT("__kernel_c2isr_interrupt_%d",intno)$$NL$
			$FORMAT("__kernel_c2isr_interrupt_%d",intno)$:$NL$
			$TAB$addi	-80 , sp , sp$NL$
			$TAB$st.w	r10 , 52[sp]$NL$
			$TAB$st.w	r11 , 48[sp]$NL$
			$TAB$st.w	r12 , 44[sp]$NL$
			$TAB$st.w	r13 , 40[sp]$NL$
$ r10に割込みハンドラアドレス
$ r11に割込みレベル(CPU割込み優先度)
$ r12に割込みハンドラ番号
			$TAB$Lea 	$+intno$, r10 $NL$
			$TAB$shl 	2, r10 $NL$
			$TAB$add 	r10, r13$NL$
			$TAB$ld.w 	0[r13], r13$NL$
			$TAB$Lea	_$ISR.INT_ENTRY[isrid]$ , r10$NL$
			$TAB$mov	$CPU_INTPRI_MIN - ISR.INTPRI[isrid]$ , r11 $ASM_COMMENT()$$FORMAT("ext_prio=%d cpu_intr_prio=%d",ISR.INTPRI[isrid], CPU_INTPRI_MIN - ISR.INTPRI[isrid])$$NL$
			$TAB$Lea	$+intno$ , r12 $ASM_COMMENT()$$FORMAT("intno=%d",intno)$$NL$
			$TAB$jr		_interrupt_isr2$NL$
		$END$

		$IF EQ(ISR.CATEGORY[isrid], "CATEGORY_1")$
			$TAB$.globl	_$ISR.INT_ENTRY[isrid]$$NL$
			$TAB$.globl	$FORMAT("__kernel_c1isr_interrupt_%d",intno)$$NL$
			$FORMAT("__kernel_c1isr_interrupt_%d",intno)$:$NL$
			$TAB$addi	-80 , sp , sp$NL$
			$TAB$st.w	r10 , 52[sp]$NL$
			$TAB$st.w	r11 , 48[sp]$NL$
			$TAB$st.w	r12 , 44[sp]$NL$
			$TAB$st.w	r13 , 40[sp]$NL$
$ r10に割込みハンドラアドレス
$ r12に割込みハンドラ番号
			$TAB$Lea	_$ISR.INT_ENTRY[isrid]$ , r10$NL$
			$TAB$Lea	$+intno$ , r12 $ASM_COMMENT()$$FORMAT("intno=%d",intno)$$NL$
			$TAB$jr		_interrupt_isr1$NL$
		$END$

		$NL$
	$END$
$END$

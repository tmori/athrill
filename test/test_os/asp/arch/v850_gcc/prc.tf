$ ======================================================================
$ 
$   TOPPERS/ASP Kernel
$       Toyohashi Open Platform for Embedded Real-Time Systems/
$       Advanced Standard Profile Kernel
$ 
$   Copyright (C) 2010-2011 by Meika Sugimoto
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

$ 
$		アーキテクチャ依存テンプレート（V850用）
$ 

$ 
$  ターゲット非依存部に含まれる標準の例外管理機能の初期化処理を用いない
$ 
$OMIT_INITIALIZE_EXCEPTION = 1$

$ 
$  ATT_ISRで使用できる割込み番号とそれに対応する割込みハンドラ番号
$ 
$INTNO_ATTISR_VALID = INTNO_VALID$
$INHNO_ATTISR_VALID = INHNO_VALID$

$ 
$  DEF_INT／DEF_EXCで使用できる割込みハンドラ番号／CPU例外ハンドラ番号
$ 
$INHNO_DEFINH_VALID = INHNO_VALID$
$EXCNO_DEFEXC_VALID = EXCNO_VALID$

$ 
$  CFG_INTで使用できる割込み番号と割込み優先度，割込み属性
$ 
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = { 0,-1,...,-7 }$


$ 
$  定数定義
$ 

$INCLUDE "../../kernel/kernel.tf"$

$ 
$ 割込み処理モデル実現のためのデータ生成
$ 
$ 割込みマスクレベル毎のIMRレジスタ値を生成する．
$ 

$INTLVL_RANGE = {0,1, ... , 7}$
$IMR_RANGE = {0,1,2,3}$

const uint16_t imr_table[][IMR_SIZE] = $NL$
{$NL$
$JOINEACH intlvl INTLVL_RANGE " , \n"$
	$FOREACH imrno IMR_RANGE$
		$IMRn[imrno] = 0xFFFF$
	$END$
	
	$FOREACH intno { 8 , 9 , ... , 63}$
		$IF LENGTH(INT.INTPRI[intno])
		  && (-INT.INTPRI[intno] > intlvl)$
			$OFFSET = (INT.INTNO[intno] - 8) / 16$
			$BITPOS = (INT.INTNO[intno] - 8) % 16$
			$IMRn[OFFSET] = IMRn[OFFSET] & ~(1 << BITPOS)$
		$END$
	$END$
	$TAB${$SPC$
	$JOINEACH imrno IMR_RANGE " , "$
		$FORMAT("0x%1$x" , +IMRn[imrno])$
	$END$
	$SPC$}
$END$
$NL$};$NL$


$ 
$ CPU例外/割込みハンドラの生成
$ 

$FILE "kernel_cfg_asm.S"$

/*$NL$
$SPC$*$TAB$CPU例外/割込みハンドラテーブルの定義$NL$
$SPC$*$NL$
$SPC$*$TAB$各ハンドラに16byteずつ命令を配置可能．$NL$
$SPC$*/$NL$

$NL$
$NL$

#include "v850asm.inc"$NL$
#define TOPPERS_LABEL_ASM$NL$
#define TOPPERS_MACRO_ONLY$NL$
#include "kernel_impl.h"$NL$

$NL$
$NL$

$TAB$.section .vectors , "ax"$NL$
$TAB$.align	4$NL$
$NL$
$NL$

$TAB$.globl	interrupt$NL$
$TAB$.globl	_default_exc_handler$NL$
$TAB$.globl	_default_int_handler$NL$

$NL$

_reset_handler:$NL$
$TAB$.globl _start$NL$
$TAB$jr	_start$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
_int_handler_1:$NL$
$IF LENGTH(INH.INTHDR[1]) != 0$
	$IF (INH.INHATR[1] & TA_NONKERNEL) == 0$
		$TAB$jr	__kernel_$INH.INTHDR[1]$_$+INH.INHNO[1]$$NL$
	$ELSE$
		$TAB$.globl	_$INH.INTHDR[1]$$NL$
		$TAB$jr		_$INH.INTHDR[1]$$NL$
	$END$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
$ELSE$
	$TAB$jr	_goto_default_int_handler$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
	$TAB$nop$NL$
$END$
_int_handler_2:$NL$
$IF LENGTH(INH.INTHDR[2]) != 0$
	$IF (INH.INHATR[2] & TA_NONKERNEL) == 0$
		$TAB$jr	__kernel_$INH.INTHDR[2]$_$+INH.INHNO[intno]$$NL$
	$ELSE$
		$TAB$.globl	_$INH.INTHDR[2]$$NL$
		$TAB$jr		_$INH.INTHDR[2]$$NL$
	$END$
$ELSE$
$TAB$jr	_goto_default_int_handler$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$TAB$nop$NL$
$END$

$NL$
$NL$

$FOREACH excno EXCNO_RANGE$
	_exc_handler_$excno$:$NL$
	$IF LENGTH(EXC.EXCHDR[excno]) != 0$
		$TAB$.globl	$EXC.EXCHDR[excno]$_handler$NL$
		$TAB$jr	$EXC.EXCHDR[excno]$_handler$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$ELSE$
		$TAB$jr	_goto_default_exc_handler$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$END$
$END$

$ J-Writer用にセキュリティIDを設定
security_id:$NL$
$TAB$.long	0xFFFFFFFF$NL$
$TAB$.long	0xFFFFFFFF$NL$
$TAB$.long	0xFFFFFFFF$NL$
$TAB$.long	0xFFFFFFFF$NL$

$NL$

$FOREACH intno INTNO_RANGE$
	_int_handler_$intno$:$NL$
	$IF LENGTH(INH.INTHDR[intno]) != 0$
		$IF (INH.INHATR[intno] & TA_NONKERNEL) == 0$
			$TAB$.globl	$INH.INTHDR[intno]$_handler$NL$
			$TAB$jr	__kernel_$INH.INTHDR[intno]$_$+INH.INHNO[intno]$$NL$
		$ELSE$
			$TAB$.globl	_$INH.INTHDR[intno]$$NL$
			$TAB$jr		_$INH.INTHDR[intno]$$NL$
		$END$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$ELSE$
		$TAB$jr	_goto_default_int_handler$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
		$TAB$nop$NL$
	$END$
$END$

$NL$
$NL$

$TAB$.section .text , "ax"$NL$
$TAB$.align 4$NL$

$NL$

$FOREACH excno EXCNO_RANGE$
	$IF LENGTH(EXC.EXCHDR[excno]) != 0$
		$TAB$.globl	_$EXC.EXCHDR[excno]$$NL$
		$EXC.EXCHDR[excno]$_handler:$NL$
		$TAB$addi	-80 , sp , sp$NL$
		$TAB$st.w	r10 , 52[sp]$NL$
		$TAB$st.w	r11 , 48[sp]$NL$
		$TAB$st.w	r12 , 44[sp]$NL$
		$TAB$Lea	_$EXC.EXCHDR[excno]$ , r10$NL$
		$TAB$addi	80 , sp , r11$NL$
		$TAB$Lea	$+EXC.EXCNO[excno]$ , r12$NL$
		$TAB$jr		exception$NL$
		$NL$
	$END$
$END$


$FOREACH intno INTNO_RANGE$
	$IF LENGTH(INH.INTHDR[intno]) != 0$
		$TAB$.globl	_$INH.INTHDR[intno]$$NL$
		$TAB$.globl	__kernel_$INH.INTHDR[intno]$_$+INH.INHNO[intno]$$NL$
		__kernel_$INH.INTHDR[intno]$_$+INH.INHNO[intno]$:$NL$
		$TAB$addi	-80 , sp , sp$NL$
		$TAB$st.w	r10 , 52[sp]$NL$
		$TAB$st.w	r11 , 48[sp]$NL$
		$TAB$st.w	r12 , 44[sp]$NL$
$ r10に割込みハンドラ番号，r11に割込みレベル，r12に割込み番号を置く
		$TAB$Lea	_$INH.INTHDR[intno]$ , r10$NL$
		$TAB$mov	$-INT.INTPRI[intno]$ , r11$NL$
		$TAB$mov	$+INH.INHNO[intno]$ , r12$NL$
		$TAB$jr		interrupt$NL$
		$NL$
	$END$
$END$

$NL$
$NL$

_goto_default_int_handler:$NL$
$TAB$addi	-80 , sp , sp$NL$
$TAB$st.w	r10 , 0[sp]$NL$
$TAB$Lea	_default_int_handler , r10$NL$
$TAB$jr		interrupt$NL$

$NL$
$NL$

_goto_default_exc_handler:$NL$
$TAB$addi	-80 , sp , sp$NL$
$TAB$st.w	r10 , 0[sp]$NL$
$TAB$Lea	_default_exc_handler , r10$NL$
$TAB$jr		exception$NL$


$TAB$.end$NL$
$NL$

$ 
$ 割込み属性のチェック
$ 
$ 外部割込みのみTA_NEGEDGE，TA_POSEDGE，TA_BOTHEDGEが指定されているか
$ チェックを行う．
$ 

$FOREACH intno INT.ID_LIST$
	$IF LENGTH(INT.INTATR[intno])$
		$IF LENGTH(FIND(EXTINT_RANGE , +intno))$
$ 外部割込みの場合：属性が一つだけ指定されているかチェック
			$IF !(INT.INTATR[intno] == TA_NEGEDGE)
			 && !(INT.INTATR[intno] == TA_POSEDGE)
			 && !(INT.INTATR[intno] == TA_BOTHEDGE)$
				$ERROR$
					$FORMAT("Multiple external interrupt property can't set %1% `%2%\' in CFG_INT" , intno , INT.INTNO[intno])$
				$END$
			$END$
		$ELSE$
$ 外部割込み以外の場合：外部割込み属性が指定されていないかチェック
			$IF (+INT.INTATR[intno] & (+TA_NEGEDGE | +TA_POSEDGE | +TA_BOTHEDGE)) != 0$
 				$ERROR$
 					$FORMAT("TA_NEGEDGE,TA_POSEDGE,TA_BOTHEDGE can't set %1% `%2%\' in CFG_INT" , intno , INT.INTNO[intno])$
				$NL$
 				$END$
			$END$
		$END$
	$END$
$END$


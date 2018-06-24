$ 
$		パス2のアーキテクチャ依存テンプレート（SH12A用）
$ 

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

$IF LENGTH(TEXCNO_EMULATE_TEXRTN)$
    $EXCNO_DEFEXC_VALID = APPEND(EXCNO_DEFEXC_VALID, TEXCNO_EMULATE_TEXRTN)$
$END$
$IF LENGTH(TEXCNO_EMULATE_RET_TEX)$
    $EXCNO_DEFEXC_VALID = APPEND(EXCNO_DEFEXC_VALID, TEXCNO_EMULATE_RET_TEX)$
$END$

$ 
$  CFG_INTで使用できる割込み番号と割込み優先度
$ 
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = { -1,-2,...,-15 }$

$ 
$  カーネル管理外の割込み
$ 
$TARGET_INHATR = TA_NONKERNEL$

$ 
$  標準テンプレートファイルのインクルード
$ 
$INCLUDE "kernel/kernel.tf"$


$ 
$  割込み番号，割込みハンドラ番号，CPU例外ハンドラ番号の範囲
$  　　番号が連続していない場合、途中の番号も含む。

$ 　以下の条件を満たしているかのエラーチェック
$  　　・0〜3はスタートアップルーチン用に予約
$  　　・ベクタテーブルを生成する都合で、EXCNO_RANGEとINTNO_RANGEは
$  　　　連続していなければ、ならない。
$ 
$IF TMIN_EXCNO != 4$
	$ERROR$ TMIN_EXCNO($+TMIN_EXCNO$) != 4 $END$
$END$
$IF (TMAX_EXCNO + 1) != TMIN_INTNO$
	$ERROR$ (TMAX_EXCNO($+TMAX_EXCNO$) + 1) != TMIN_INTNO($+TMIN_INTNO$) $END$
$END$

$EXCNO_RANGE = RANGE(4, TMIN_INTNO - 1)$
$INTNO_RANGE = RANGE(TMIN_INTNO, TMAX_INTNO)$
$INHNO_RANGE = INTNO_RANGE$


/*												$NL$$SPC$
 *  Target-dependent Definitions (SH12A)		$NL$$SPC$
 */												$NL$$NL$

$ 
$  CFG_INTのアーキテクチャ依存のエラーチェック
$ 
$ 　１．割込み優先度のエラーチェック
$ 		・同じ割込み優先度になるべき割込み番号に対して、異なる割込み
$ 		　優先度を指定していないかをチェックする。
$ 		・same_intpri_listはチップ依存部で定義する。
$ 		　（-1をセパレータとする。）
$ 		・same_intpri_listからは、静的APIの引数として与えられた文字列は
$ 		　取り出せないので、この文字列が必要な場合は、INT.INTNO[intno1]
$ 		　のような記述を用いて、取り出す。
$ 

$intno_list = {}$
$FOREACH intno (same_intpri_list)$
	$IF (intno != -1)$
$       セパレータ以外のときの処理
		$IF (LENGTH(INT.INTPRI[intno]) != 0)$
			$intno_list = APPEND(intno_list, intno)$
		$END$

$       セパレータまで到達したときの処理
	$ELSE$
$       　　indexの範囲0〜10に特に意味はない。
$       　　同じ割込み優先度を設定すべき割込み番号数をカバーできればよい。
		$FOREACH index ({ 0,1,...,10 })$
			$IF (LENGTH(intno_list) >= index + 2)$
				$intno1 = AT(intno_list, index)$
				$intno2 = AT(intno_list, index + 1)$
				$IF (INT.INTPRI[intno1] != INT.INTPRI[intno2])$
					$ERROR INT.TEXT_LINE[intno1]$:E_PAR:
						$FORMAT("CFG_INT:These intno's intpri must be same.\n")$
						$FORMAT("\t configuration file %s:(%d)\n",
							INT.TEXT_LINE[intno1], +INT.TEXT_LINE[intno1])$
						$FORMAT("\t\t intno:%s(%d), intpri:%s(%d)\n",
							INT.INTNO[intno1], +intno1,
							INT.INTPRI[intno1], +INT.INTPRI[intno1])$
						$FORMAT("\t configuration file %s:(%d)\n",
							INT.TEXT_LINE[intno2], +INT.TEXT_LINE[intno2])$
						$FORMAT("\t\t intno:%s(%d), intpri:%s(%d)\n",
							INT.INTNO[intno2], +intno2,
							INT.INTPRI[intno2], +INT.INTPRI[intno2])$
					$END$
				$END$
			$END$
		$END$

		$intno_list = {}$
	$END$
$END$


$ 
$  CFG_INTのアーキテクチャ依存のエラーチェック
$ 
$ 　２．割込み属性のエラーチェック
$ 　　　　コア依存部またはチップ依存部で以下の変数、リストを定義する。
$ 　　　　　・TINTNO_NMI：NMIの割込み番号
$ 　　　　　・valid_irq_intatr_list：IRQ割込みで使用できる割込み属性
$ 　　　　　・valid_nmi_intatr_list：NMIで使用できる割込み属性
$ 
$ 	割込み属性に設定できる値は割込み要因毎に異なるので、
$ 	それぞれチェックしている。（非依存部でまとめてチェックできない。）
$ 	そのため、１つのエラーで、複数個のエラーメッセージが表示されるが
$ 	止むを得ない。
$ 

$FOREACH intno (INT.ORDER_LIST)$
	$intno0 = INT.INTNO[intno]$
$ 		intnoと同値のintno0を求めているのは、文字列属性を取り出すため
	$intatr0 = INT.INTATR[intno]$
	
	$IF (TMIN_INTNO_IRQ <= intno0) && (intno0 <= TMAX_INTNO_IRQ)$
$ 											IRQ割込みの場合
$ 			TA_ENAINTの指定は無視する。
		$intatr1 = intatr0 & ~TA_ENAINT$
		$IF LENGTH(FIND(valid_irq_intatr_list, intatr1)) == 0$
			$ERROR INT.TEXT_LINE[intno]$E_RSATR:
				$FORMAT(_("illegal %1% `%2%\' of intno `%3%\' in %4%"),
					"intatr", intatr0, intno0, "CFG_INT")$
			$END$
		$END$
	$ELSE$
		$IF LENGTH(TINTNO_NMI) && (intno0 == TINTNO_NMI)$
$ 											NMIの場合
$ 		備考
$ 			NMIをわざわざ割込み許可しているのは間違いの可能性が
$ 			高いので、TA_ENAINTが指定されている場合もエラー扱いにする。
			$IF LENGTH(FIND(valid_nmi_intatr_list, intatr0)) == 0$
				$ERROR INT.TEXT_LINE[intno]$E_RSATR:
					$FORMAT(_("illegal %1% `%2%\' of intno `%3%\' in %4%"),
						"intatr", intatr0, intno0, "CFG_INT")$
				$END$
			$END$
		$ELSE$
$ 											IRQ, NMI以外の割込みの場合
			$IF (intatr0 & ~TA_ENAINT) != 0$
				$ERROR INT.TEXT_LINE[intno]$E_RSATR:
					$FORMAT(_("illegal %1% `%2%\' of intno `%3%\' in %4%"),
						"intatr", intatr0, intno0, "CFG_INT")$
				$END$
			$END$
		$END$
	$END$
$END$


/***************************************************************$NL$$SPC$
 *																$NL$$SPC$
 *  ターゲット依存かつアプリケーション依存の処理（SH12A）		$NL$$SPC$
 *																$NL$$SPC$
 *	このファイルはコンフィギュレータにより自動生成されている。	$NL$$SPC$
 *	修正する場合はテンプレートファイルを修正すること。			$NL$$SPC$
 **************************************************************/$NL$

$ 
$  割込み優先度テーブル
$ 
const uint8_t _kernel_int_iipm_tbl[TNUM_INT] = {				$NL$
$FOREACH intno INTNO_RANGE$
	$IF LENGTH(INT.INTNO[intno])$
		$intmask = (-INT.INTPRI[intno])$
	$ELSE$
$		NOT_CFG_INT_BITは，割込み優先度が設定されていないことを示す。
		$intmask = NOT_CFG_INT_BIT$
	$END$
	$TAB$$FORMAT("UINT8_C(0x%02x), /* 0x%03x */", +intmask, +intno)$$NL$
$END$
$NL$
};																$NL$$NL$


$ 
$  アセンブラ・ファイル
$ 
$FILE "kernel_cfg_asm.S"$

#define _start		__kernel_start		$NL$
#define _ext_ker __kernel_ext_ker$NL$

/***************************************************************$NL$$SPC$
 *																$NL$$SPC$
 *  ターゲット依存かつアプリケーション依存の処理（SH12A）		$NL$$SPC$
 *																$NL$$SPC$
 *	このファイルはコンフィギュレータにより自動生成されている。	$NL$$SPC$
 *	修正する場合はテンプレートファイルを修正すること。			$NL$$SPC$
 **************************************************************/$NL$

$ 
$  割込みハンドラ／CPU例外ハンドラの出入口処理の生成
$ 
/*															$NL$$SPC$
 *  割込みハンドラの入口処理								$NL$$SPC$
 *															$NL$$SPC$
 *　　レジスタ割当											$NL$$SPC$
 *　　　　r7：割込み受付直後／CPU例外発生時のSRのコピー		$NL$$SPC$
 *　　　　r6：割込みハンドラ／CPU例外ハンドラの先頭アドレス	$NL$$SPC$
 *　　　　r5：割込み／CPU例外ハンドラ番号					$NL$$SPC$
 *　　　　r4：_kernel_interrupt_entry／_kernel_exception_entry	$NL$$SPC$
 *			（prc_support.S内での分岐先）					$NL$$SPC$
 *															$NL$$SPC$
 *	割込みとCPU例外の相違点									$NL$$SPC$
 *	　　・CPU例外ハンドラに引数を与える						$NL$$SPC$
 *	　　・CPU例外では、例外発生時のIPMと同じ値で
 										割込みを許可する	$NL$$SPC$
 *		（CPU例外により、IPMを変化させない）				$NL$$SPC$
 *															$NL$$SPC$
 *	　　・割込みハンドラの場合								$NL$$SPC$
 *	　　  	割込み受付時にハードウェアがIPMを設定する		$NL$$SPC$
 *	　　  	割込み許可にはこのIPMの値を用いる				$NL$$SPC$
 *	　　・CPU例外ハンドラの場合								$NL$$SPC$
 *	　　  	CPU例外受付時にIPMの値は変化しない				$NL$$SPC$
 *	　　  	割込み許可にはCPU例外発生時のIPMの値を用いる	$NL$$SPC$
 */															$NL$$TAB$
	.section .vector_entry,"xa"  							$NL$
$FOREACH inhno INHNO_VALID$
	$IF LENGTH(INH.INHNO[inhno])$
$ 
$ 割込みハンドラの登録がある場合
$ 
$NL$$TAB$
$ 	kernel_cfg.c内のconst INHINIB __kernel_inhinib_table[TNUM_INHNO]で
$ 	このラベルを参照するため、グローバルスコープにする必要がある。

	$label1 = CONCAT("__kernel_", INH.INTHDR[inhno])$
	$label2 = CONCAT("_", INH.INHNO[inhno])$
	$label  = CONCAT(label1, label2)$

	$IF LENGTH(TINTNO_NMI) && (inhno == TINTNO_NMI)$
$ 
$ NMIの場合
$ 　通常の出入り口処理を経由せず、直接、割込みハンドラを呼び出す。
$ 
	.global $label$											$NL$$TAB$
	.align 2                       							$NL$
$label$:													$NL$
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION 					$NL$$TAB$
	/*  NMIの場合は、SH2Aでもレジスタバンクに待避されない  */$NL$$TAB$
	movml.l r7,@-r15 										$NL$
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */ 			$NL$$TAB$
	mov.l r7,@-r15 											$NL$$TAB$
	mov.l r6,@-r15											$NL$$TAB$
	mov.l r5,@-r15											$NL$$TAB$
	mov.l r4,@-r15											$NL$$TAB$
	mov.l r3,@-r15 											$NL$$TAB$
	mov.l r2,@-r15											$NL$$TAB$
	mov.l r1,@-r15											$NL$$TAB$
	mov.l r0,@-r15											$NL$
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */ 				$NL$$TAB$
	sts.l pr,@-r15											$NL$$TAB$
	mov.l _c_routine_$label$,r6     						$NL$$TAB$
	jsr	  @r6												$NL$$TAB$$SPC$
	 nop													$NL$$TAB$
	lds.l  @r15+,pr											$NL$
#ifdef TOPPERS_SUPPORT_SH2A_INSTRUCTION						$NL$$TAB$
	movml.l @r15+,r7										$NL$
#else	/*	TOPPERS_SUPPORT_SH2A_INSTRUCTION  */			$NL$$TAB$
	mov.l  @r15+,r0											$NL$$TAB$
	mov.l  @r15+,r1											$NL$$TAB$
	mov.l  @r15+,r2											$NL$$TAB$
	mov.l  @r15+,r3											$NL$$TAB$
	mov.l  @r15+,r4											$NL$$TAB$
	mov.l  @r15+,r5											$NL$$TAB$
	mov.l  @r15+,r6											$NL$$TAB$
	mov.l  @r15+,r7											$NL$
#endif	/* TOPPERS_SUPPORT_SH2A_INSTRUCTION */				$NL$$TAB$
	rte														$NL$$TAB$$SPC$
	 nop													$NL$$TAB$

	.align 2												$NL$
_c_routine_$label$:
			/* 割込みハンドラの先頭アドレス */				$NL$$TAB$
	.long  _$INH.INTHDR[inhno]$         					$NL$$TAB$
	$ELSE$
$ 
$ NMI以外の割込みの場合
$ 
	.global $label$											$NL$$TAB$
	.align 2                       							$NL$
$label$:													$NL$
#ifndef USE_BANKED_REG    									$NL$$TAB$
	$TAB$ /*  必要最小限のレジスタをスタックに待避  */ 		$NL$$TAB$
	mov.l r7,@-r15 											$NL$$TAB$
	mov.l r6,@-r15											$NL$$TAB$
	mov.l r5,@-r15											$NL$$TAB$
	mov.l r4,@-r15											$NL$
#endif /* USE_BANKED_REG */   								$NL$$TAB$
	stc   sr,r7								/* SRの保存 */ 	$NL$$TAB$
	mov.l _iipm_disall_$label$,r6							$NL$$TAB$
	ldc	  r6,sr								/* 全割込み禁止 */ $NL$
					$TAB$$TAB$ /* 共有割込み入口処理 */    	$NL$$TAB$
	mov.l _common_routine_$label$,r4 						$NL$
					$TAB$$TAB$ /* 割込みハンドラの先頭アドレス */ $NL$$TAB$
	mov.l _c_routine_$label$,r6     						$NL$$TAB$
	mov.l _inhno_$label$,r5					/* 割込み番号 */ $NL$$TAB$
	jmp	  @r4												$NL$$TAB$$SPC$
	 nop													$NL$$TAB$

	.align 2												$NL$
			$TAB$$TAB$ /* 割込み禁止用マスク */ 			$NL$
_iipm_disall_$label$:										$NL$
			$TAB$$TAB$ /* ipm以外のビットはゼロで良い */ 	$NL$$TAB$
	.long  __kernel_iipm_disall								$NL$
_c_routine_$label$:
			/* 割込みハンドラの先頭アドレス */				$NL$$TAB$
	.long  _$INH.INTHDR[inhno]$         					$NL$$TAB$
		$TAB$ /* cpu_support.S内の分岐先アドレス */			$NL$
_common_routine_$label$: 									$NL$$TAB$
	$IF (INH.INHATR[inhno] & TA_NONKERNEL) != 0$
	.long  __kernel_non_kernel_int_entry /*  カーネル管理外の割込み  */ $NL$
	$ELSE$
	.long  __kernel_interrupt_entry							$NL$
	$END$
_inhno_$label$:							/* 割込み番号	*/	$NL$$TAB$
	.long  $inhno$ 											$NL$$NL$
	$END$

	$ELSE$
$ 
$ 	割込みハンドラの登録がない場合
$      r6に割込み番号を格納して、cpu_support.S内の
$      _default_int_handler_entryへ分岐する。

	$label  = CONCAT("__kernel_default_int_handler_entry_", inhno)$
															$NL$$TAB$
	.align 2                       							$NL$
$label$:													$NL$
#ifndef USE_BANKED_REG    									$NL$$TAB$
	$TAB$ /*  必要最小限のレジスタをスタックに待避  */ 		$NL$$TAB$
	mov.l r7,@-r15 											$NL$$TAB$
	mov.l r6,@-r15											$NL$
#endif /* USE_BANKED_REG */   								$NL$$TAB$
	$TAB$			/* cpu_support.S内の分岐先アドレス	*/ 	$NL$$TAB$
	mov.l _default_int_handler_entry_asm_$label$,r7 		$NL$$TAB$
	mov.l _inhno_$label$,r6				/* 割込み番号 */	$NL$$TAB$
	jmp	  @r7                        						$NL$$TAB$$SPC$
	 nop						 	 						$NL$$TAB$
	.align 2						 						$NL$$TAB$$TAB$
					/* cpu_support.S内の分岐先アドレス	*/ 	$NL$
_default_int_handler_entry_asm_$label$:						$NL$$TAB$
	.long  __kernel_default_int_handler_entry				$NL$
_inhno_$label$:                     /* 割込み番号 */ 		$NL$$TAB$
	.long  $inhno$ 											$NL$
	$END$
$END$

/*															$NL$$SPC$
 *  CPU例外ハンドラの出入口処理								$NL$$SPC$
 */															$NL$$NL$
	.section .vector_entry,"xa"  							$NL$
$FOREACH excno EXCNO_VALID$
	$IF LENGTH(EXC.EXCNO[excno])$
$ 
$ CPU例外ハンドラの登録がある場合
$ 
	$label1 = CONCAT("__kernel_", EXC.EXCHDR[excno])$
	$label2 = CONCAT("_", EXC.EXCNO[excno])$
	$label  = CONCAT(label1, label2)$
															$NL$$TAB$
	.align 2                       							$NL$
$label$:													$NL$$TAB$
	$TAB$ /*  必要最小限のレジスタをスタックに待避  */ 		$NL$$TAB$
	mov.l r7,@-r15 											$NL$$TAB$
	mov.l r6,@-r15											$NL$$TAB$
	mov.l r5,@-r15											$NL$$TAB$
	mov.l r4,@-r15											$NL$$TAB$
	stc   sr,r7								/* SRの保存 */	$NL$$TAB$
	mov.l _iipm_disall_$label$,r6							$NL$$TAB$
	ldc	  r6,sr								/* 全割込み禁止 */ $NL$
			$TAB$$TAB$ /* 共有CPU例外入口処理 */ 			$NL$$TAB$
	mov.l _common_routine_$label$,r4 						$NL$
			$TAB$$TAB$ /* ベクタ番号 */    					$NL$$TAB$
	mov.l _vectorno_$label$,r5       						$NL$
			$TAB$$TAB$ /* CPU例外ハンドラの先頭アドレス */ 	$NL$$TAB$
	mov.l _c_routine_$label$,r6      						$NL$$TAB$
	jmp	  @r4												$NL$$TAB$$SPC$
	 nop													$NL$$TAB$
	.align 2												$NL$
_iipm_disall_$label$:			/* 割込み禁止用マスク */ 	$NL$
			$TAB$$TAB$ /* ipm以外のビットはゼロで良い */ 	$NL$$TAB$
	.long  __kernel_iipm_disall								$NL$
			$TAB$$TAB$ /* CPU例外ハンドラの先頭アドレス */ 	$NL$
_c_routine_$label$:											$NL$$TAB$
	.long  _$EXC.EXCHDR[excno]$         					$NL$
_vectorno_$label$:		/* ベクタ番号 */ 					$NL$$TAB$
	.long  $excno$                      					$NL$
			$TAB$$TAB$ /* cpu_support.S内の分岐先アドレス */ $NL$
_common_routine_$label$:									$NL$$TAB$
	.long  __kernel_exception_entry        					$NL$$NL$
	$ELSE$
$ 
$ CPU例外ハンドラの登録がない場合
$      r6にベクタ番号を格納して、cpu_support.S内の
$      _default_exc_handler_entryへ分岐する。
$ 
	$label  = CONCAT("__kernel_exc_entry_", excno)$
															$NL$$TAB$
	.align 2                           						$NL$
$label$:													$NL$$TAB$
	$TAB$ /*  必要最小限のレジスタをスタックに待避  */ 		$NL$$TAB$
	mov.l r7,@-r15 			           						$NL$$TAB$
	mov.l r6,@-r15											$NL$$TAB$
	$TAB$	/* cpu_support.S内の分岐先アドレス */			$NL$$TAB$
	mov.l _default_exc_handler_entry_asm_$label$,r7			$NL$$TAB$
	jmp	  @r7                        						$NL$$TAB$$SPC$
	 mov  #$excno$,r6				/* ベクタ番号 */    	$NL$

	$IF (excno > 0x80)$
		$ERROR$
			$FORMAT("excno(%d) > 0x80 ! (sign extension in mov #imm, rn instruction)", 
					+excno)$$NL$
		$END$
	$END$
															$NL$$TAB$
	.align 2						 						$NL$$TAB$
		 /* cpu_support.S内の分岐先アドレス */ 				$NL$
_default_exc_handler_entry_asm_$label$:    					$NL$$TAB$
	.long  __kernel_default_exc_handler_entry				$NL$$NL$

	$END$
$END$


/*															$NL$$SPC$
 *  ベクタテーブル											$NL$$SPC$
 */															$NL$$NL$$TAB$
	.section .vector										$NL$$TAB$
    .global __kernel_vectors								$NL$$NL$
	
__kernel_vectors:											$NL$$TAB$
	.long	_start		/* 0 */								$NL$$TAB$
	.long	0     		/* 1 */								$NL$$TAB$
	.long	_start		/* 2 */								$NL$$TAB$
	.long	0     		/* 3 */								$NL$$NL$$TAB$
	/* CPU exception */										$NL$
$FOREACH excno EXCNO_RANGE$
	$IF LENGTH(TINTNO_NMI) && (excno == TINTNO_NMI)$
$ 
$ NMIの場合
$ 
	$inhno = excno$
	$IF LENGTH(INH.INHNO[inhno])$
$ 
$ 割込みハンドラの登録がある場合
$ 
		$TAB$.long __kernel_$INH.INTHDR[inhno]$_$INH.INHNO[inhno]$
	$ELSE$
$ 
$ 割込みハンドラの登録がない場合
$ 
		$TAB$.long __kernel_default_int_handler_entry_$inhno$
	$END$
	$FORMAT(" /* %d */", +inhno)$$NL$

$ 
$ CPU例外の場合
$ 
	$ELSE$
	$IF LENGTH(EXC.EXCNO[excno])$
$ 
$ CPU例外ハンドラの登録がある場合
$ 
		$IF LENGTH(FIND(EXCNO_VALID, excno))$
		    $TAB$.long __kernel_$EXC.EXCHDR[excno]$_$EXC.EXCNO[excno]$
		$ELSE$
			$TAB$.long _ext_ker
		$END$
	$ELSE$
$ 
$ CPU例外ハンドラの登録がない場合
$ 
		$IF LENGTH(FIND(EXCNO_VALID, excno))$
			$TAB$.long __kernel_exc_entry_$excno$
		$ELSE$
			$TAB$.long _ext_ker
		$END$
	$END$
	$FORMAT(" /* %d */", +excno)$$NL$
	$END$
$END$
																	$NL$
	$TAB$/* interrupt */											$NL$
$FOREACH inhno INHNO_RANGE$
	$IF LENGTH(INH.INHNO[inhno])$
$ 
$ 割込みハンドラの登録がある場合
$ 
		$TAB$.long __kernel_$INH.INTHDR[inhno]$_$INH.INHNO[inhno]$
	$ELSE$
$ 
$ 割込みハンドラの登録がない場合
$ 
		$IF LENGTH(FIND(INHNO_VALID, inhno))$
			$TAB$.long __kernel_default_int_handler_entry_$inhno$
		$ELSE$
			$TAB$.long _ext_ker
		$END$

	$END$
	$FORMAT(" /* %d */", +inhno)$$NL$
$END$
$NL$

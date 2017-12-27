/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *	
 *	Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 */


#ifndef TOPPERS_PRC_SVC_H
#define TOPPERS_PRC_SVC_H

#ifndef TOPPERS_MACRO_ONLY

typedef sint32 FunctionCodeType;                  /* 機能コードの型 */

/*
 *  カーネルのサービスコールのインタフェース
 *
 *  スクラッチレジスタ
 *    r10 : 返り値
 *    r11：関数コード
 *    r6 - r9 : 引数（4つまで）
 *  関数呼び出し(jarl)でサービスコールを呼び出すため，lpも保存する
 *  syscallでsvc_entryへジャンプ
 */

/* "memory"指定あり */

#define CAL_SVC_0M(TYPE, FNCD) \
	register TYPE  r10 asm("r10"); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"=r"(r10),"+r"(r11) \
		: \
		:"memory","lp","r6","r7","r8","r9","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	); \
	return(r10);

#define CAL_SVC_1M(TYPE, FNCD, TYPE1, PAR1) \
	register TYPE  r10 asm("r10"); \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"=r"(r10),"+r"(r11),"+r"(r6) \
		: \
		:"memory","lp","r7","r8","r9","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	); \
	return(r10);


#define CAL_SVC_2M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	register TYPE  r10 asm("r10"); \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register TYPE2  r7 asm("r7") = (TYPE2)(PAR2); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"=r"(r10),"+r"(r11),"+r"(r6),"+r"(r7) \
		: \
		:"memory","lp","r8","r9","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	); \
	return(r10);


#define CAL_SVC_3M(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	register TYPE  r10 asm("r10"); \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register TYPE2  r7 asm("r7") = (TYPE2)(PAR2); \
	register TYPE3  r8 asm("r8") = (TYPE3)(PAR3); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"=r"(r10),"+r"(r11),"+r"(r6),"+r"(r7),"+r"(r8) \
		: \
		:"memory","lp","r9","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	); \
	return(r10);

/* 
 *  返り値がvoidの場合
 *  r10を破壊レジスタに加える
 */

#define CAL_SVC_0N(TYPE, FNCD) \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"+r"(r11) \
		: \
		:"memory","lp","r6","r7","r8","r9","r10","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	);

#define CAL_SVC_1N(TYPE, FNCD, TYPE1, PAR1) \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"+r"(r11),"+r"(r6) \
		: \
		:"memory","lp","r7","r8","r9","r10","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	);


#define CAL_SVC_2N(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register TYPE2  r7 asm("r7") = (TYPE2)(PAR2); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"+r"(r11),"+r"(r6),"+r"(r7) \
		: \
		:"memory","lp","r8","r9","r10","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	);


#define CAL_SVC_3N(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	register TYPE1  r6 asm("r6") = (TYPE1)(PAR1); \
	register TYPE2  r7 asm("r7") = (TYPE2)(PAR2); \
	register TYPE3  r8 asm("r8") = (TYPE3)(PAR3); \
	register FunctionCodeType    r11 asm("r11") = (FunctionCodeType)(FNCD); \
	Asm ( \
        "syscall 1\n\t" \
        "nop\n\t" \
		:"+r"(r11),"+r"(r6),"+r"(r7),"+r"(r8) \
		: \
		:"memory","lp","r9","r10","r12","r13","r14","r15","r16","r17","r18","r19", "ep" \
	);


#ifndef TOPPERS_SVC_CALL
#include "gcc/tool_svc.h"
#endif /* TOPPERS_SVC_CALL */

#endif /* TOPPERS_MACRO_ONLY */


#endif /* TOPPERS_TARGET_SVC_H */


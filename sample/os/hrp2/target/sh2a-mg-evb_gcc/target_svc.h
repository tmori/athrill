/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
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


#ifndef TOPPERS_TARGET_SVC_H
#define TOPPERS_TARGET_SVC_H

#ifndef TOPPERS_MACRO_ONLY


/*
 *  カーネルのサービスコールのインタフェース
 *
 *  スクラッチレジスタ
 *    r1 : 関数コード
 *    r4 - r7 : 引数（4つまで）
 *    r2 : 第5引数
 *  関数呼び出し(jsr)でサービスコールを呼び出すため，PRも保存する
 */
extern char _kernel_svc_entry;

#define CAL_SVC_0(TYPE, FNCD) \
	register TYPE  r0 asm("r0"); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%2\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1) \
		:"r"(entry),"r"(r1)\
		:"pr","r2","r3","r4","r5","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_1(TYPE, FNCD, TYPE1, PAR1) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%3\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4) \
		:"r"(entry),"r"(r1),"r"(r4)\
		:"pr","r2","r3","r5","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_2(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%4\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5)\
		:"pr","r2","r3","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_3(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%5\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5),"+r"(r6) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5),"r"(r6)\
		:"pr","r2","r3","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_4(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
								TYPE3, PAR3, TYPE4, PAR4) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register TYPE4 r7 asm("r7") = (TYPE4)(PAR4); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%6\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5),"+r"(r6),"+r"(r7) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5),"r"(r6),"r"(r7)\
		:"pr","r2","r3","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_5(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
						TYPE3, PAR3, TYPE4, PAR4, TYPE5, PAR5) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register TYPE4 r7 asm("r7") = (TYPE4)(PAR4); \
	register TYPE5 r2 asm("r2") = (TYPE5)(PAR5); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%7\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r2),"+r"(r4),"+r"(r5),"+r"(r6),"+r"(r7) \
		:"r"(entry),"r"(r1),"r"(r2),"r"(r4),"r"(r5),"r"(r6),"r"(r7) \
		:"pr","r3","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_0M(TYPE, FNCD) \
	register TYPE  r0 asm("r0"); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%2\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1) \
		:"r"(entry),"r"(r1)\
		:"memory","pr","r2","r3","r4","r5","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_1M(TYPE, FNCD, TYPE1, PAR1) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%3\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4) \
		:"r"(entry),"r"(r1),"r"(r4)\
		:"memory","pr","r2","r3","r5","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_2M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%4\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5)\
		:"memory","pr","r2","r3","r6","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_3M(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%5\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5),"+r"(r6) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5),"r"(r6)\
		:"memory","pr","r2","r3","r7","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_4M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
								TYPE3, PAR3, TYPE4, PAR4) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register TYPE4 r7 asm("r7") = (TYPE4)(PAR4); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%6\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r4),"+r"(r5),"+r"(r6),"+r"(r7) \
		:"r"(entry),"r"(r1),"r"(r4),"r"(r5),"r"(r6),"r"(r7)\
		:"memory","pr","r2","r3","mach","macl" \
	); \
	return(r0);

#define CAL_SVC_5M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
						TYPE3, PAR3, TYPE4, PAR4, TYPE5, PAR5) \
	register TYPE  r0 asm("r0"); \
	register TYPE1 r4 asm("r4") = (TYPE1)(PAR1); \
	register TYPE2 r5 asm("r5") = (TYPE2)(PAR2); \
	register TYPE3 r6 asm("r6") = (TYPE3)(PAR3); \
	register TYPE4 r7 asm("r7") = (TYPE4)(PAR4); \
	register TYPE5 r2 asm("r2") = (TYPE5)(PAR5); \
	register FN    r1 asm("r1") = FNCD; \
    char *entry = &_kernel_svc_entry; \
	Asm ( \
        "jsr @%7\n\t" \
        "nop\n\t" \
		:"=r"(r0),"+r"(r1),"+r"(r2),"+r"(r4),"+r"(r5),"+r"(r6),"+r"(r7) \
		:"r"(entry),"r"(r1),"r"(r2),"r"(r4),"r"(r5),"r"(r6),"r"(r7) \
		:"memory","pr","r3","mach","macl" \
	); \
	return(r0);

#ifndef TOPPERS_SVC_CALL
#include "gcc/tool_svc.h"
#endif /* TOPPERS_SVC_CALL */

Inline ER_UINT
cal_svc(FN fncd, intptr_t par1, intptr_t par2,
        intptr_t par3, intptr_t par4, intptr_t par5)
{
    if(fncd > 0){
        CAL_SVC_5M(FN, fncd, intptr_t, par1, intptr_t, par2, 
                intptr_t, par3, intptr_t, par4, intptr_t, par5);
    } else {
        return (E_RSFN);
    }
}

#endif /* TOPPERS_MACRO_ONLY */


#endif /* TOPPERS_TARGET_SVC_H */


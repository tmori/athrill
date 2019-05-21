/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011-2012 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	
 *	上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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
	return(0);

#define CAL_SVC_1(TYPE, FNCD, TYPE1, PAR1) \
	return(0);

#define CAL_SVC_2(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	return(0);

#define CAL_SVC_3(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	return(0);

#define CAL_SVC_4(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
								TYPE3, PAR3, TYPE4, PAR4) \
	return(0);

#define CAL_SVC_5(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
						TYPE3, PAR3, TYPE4, PAR4, TYPE5, PAR5) \
	return(0);

#define CAL_SVC_0M(TYPE, FNCD) \
	return(0);

#define CAL_SVC_1M(TYPE, FNCD, TYPE1, PAR1) \
	return(0);

#define CAL_SVC_2M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
	return(0);

#define CAL_SVC_3M(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
	return(0);

#define CAL_SVC_4M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
								TYPE3, PAR3, TYPE4, PAR4) \
	return(0);

#define CAL_SVC_5M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2, \
						TYPE3, PAR3, TYPE4, PAR4, TYPE5, PAR5) \
	return(0);

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


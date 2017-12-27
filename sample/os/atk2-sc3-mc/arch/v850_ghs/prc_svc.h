/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *	
 *  Copyright (C) 2013-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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

extern int cal_svc_0m(int fncd) __attribute__((section(".text_shared")));
extern int cal_svc_1m(int par1, int fncd) __attribute__((section(".text_shared")));
extern int cal_svc_2m(int par1, int par2, int fncd) __attribute__((section(".text_shared")));
extern int cal_svc_3m(int par1, int par2, int par3, int fncd) __attribute__((section(".text_shared")));

extern void cal_svc_0n(int fncd) __attribute__((section(".text_shared")));
extern void cal_svc_1n(int par1, int fncd) __attribute__((section(".text_shared")));
extern void cal_svc_2n(int par1, int par2, int fncd) __attribute__((section(".text_shared")));
extern void cal_svc_3n(int par1, int par2, int par3, int fncd) __attribute__((section(".text_shared")));

#define CAL_SVC_0M(TYPE, FNCD) cal_svc_0m((int)((FunctionCodeType)FNCD))

#define CAL_SVC_1M(TYPE, FNCD, TYPE1, PAR1) \
cal_svc_1m((int)((TYPE1)PAR1), (int)((FunctionCodeType)FNCD))

#define CAL_SVC_2M(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
cal_svc_2m((int)((TYPE1)PAR1), (int)((TYPE2)PAR2), (int)((FunctionCodeType)FNCD))

#define CAL_SVC_3M(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
cal_svc_3m((int)((TYPE1)PAR1), (int)((TYPE2)PAR2), (int)((TYPE3)PAR3), (int)((FunctionCodeType)FNCD))

#define CAL_SVC_0N(TYPE, FNCD) cal_svc_0n((int)((FunctionCodeType)FNCD))

#define CAL_SVC_1N(TYPE, FNCD, TYPE1, PAR1) \
cal_svc_1n((int)((TYPE1)PAR1), (int)((FunctionCodeType)FNCD))

#define CAL_SVC_2N(TYPE, FNCD, TYPE1, PAR1, TYPE2, PAR2) \
cal_svc_2n((int)((TYPE1)PAR1), (int)((TYPE2)PAR2), (int)((FunctionCodeType)FNCD))

#define CAL_SVC_3N(TYPE, FNCD, TYPE1, PAR1, \
							TYPE2, PAR2, TYPE3, PAR3) \
cal_svc_3n((int)((TYPE1)PAR1), (int)((TYPE2)PAR2), (int)((TYPE3)PAR3), (int)((FunctionCodeType)FNCD))

#ifndef TOPPERS_SVC_CALL
#include "ghs/tool_svc.h"
#endif /* TOPPERS_SVC_CALL */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_TARGET_SVC_H */


/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
 *
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: interrupt.h 778 2017-03-06 07:21:41Z nces-hibino $
 */

/*
 *		割込み管理機能
 */

#ifndef TOPPERS_INTERRUPT_H
#define TOPPERS_INTERRUPT_H

#include "resource.h"
#include "osap.h"
#include "spinlock.h"
#include "mc.h"

/*
 *  優先度値の定義（内部表現）
 */
#define TPRI_MINISR		(-1)                /* 最低割込み優先度 */

/*
 *  ISRIDからISRCBを取り出すためのマクロ
 */
#define get_isrcb(isrid)	(p_isrcb_table[(isrid)])

/*
 *  ISRCBからISRIDを取り出すためのマクロ
 */
#define ISR2ID(p_isrcb)	((ISRType) (((p_isrcb)->p_isrinib) - isrinib_table))

#ifndef OMIT_INITIALIZE_INTERRUPT

/*
 *  割込み要求ライン初期化ブロック
 */
typedef struct interrupt_request_initialization_block {
	InterruptNumberType	intno;          /* 割込み番号 */
	AttributeType		intatr;         /* 割込み属性 */
	PriorityType		intpri;         /* 割込み優先度 */
	CoreIdType			coreid;
#if defined(TOPPERS_CFG1_OUT) || defined(CFG_USE_STACKMONITORING)
	MemorySizeType remain_stksz;        /* スタック残量チェック方式用スタックサイズ */
#endif /* defined(TOPPERS_CFG1_OUT) || defined(CFG_USE_STACKMONITORING) */
} INTINIB;

/*
 * コア間割込みスタック残量チェック方式用スタックサイズ
 */
#if defined(CFG_USE_STACKMONITORING)
extern const MemorySizeType			ici_remain_stksz[];
#endif /* defined(CFG_USE_STACKMONITORING) */

/*
 *  割込み要求ラインの数（Os_Lcfg.c）
 */
extern const InterruptNumberType	tnum_intno;

/*
 *  割込み要求ライン初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const INTINIB				intinib_table[];

/*
 *  コア間割込み初期化ブロック
 */
typedef struct inter_core_interrupt_initialization_block {
	FunctionRefType	ici_routing;
	uint32			ici_bit_ptn;
	ISRType			iciid;
} ICIINIB;

extern const ISRType				tnum_ici; /* ICISRの数 */

/*
 *  コア毎のコア間割込みの数（Os_Lcfg.c）
 */
extern const uint8					tnum_ici_of_core[];

/*
 *  コア間割込み初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const ICIINIB				iciinib_table[];

/*
 *  コア毎のコア間割込み管理ブロックのエリア（Os_Lcfg.c）
 */
extern const ICIINIB				*p_iciinb_table[];

/*
 *  ISRCBから割込み番号を割出すマクロ
 */
#define GET_INTNO(p_isrcb)	((p_isrcb)->p_isrinib->p_intinib->intno)

/*
 *  ISRCBから割込み優先度を割出すマクロ
 */
#define GET_INTPRI(p_isrcb)	((ISR2ID(p_isrcb) < (tnum_isr2 - tnum_ici)) ? \
							 ((p_isrcb)->p_isrinib->p_intinib->intpri) :  \
							 GET_ICI_INTPRI((p_isrcb)->p_isrinib->p_osapinib->coreid))

/*
 *  ISRCBからコアIDを割出すマクロ
 */
#define GET_ISR_COREID(p_isrcb)	((p_isrcb)->p_isrinib->p_osapinib->coreid)

#endif /* OMIT_INITIALIZE_INTERRUPT */

/*
 *  ICISRも含めたC2ISR数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const ISRType tnum_isr2;                                                 /* C2ISRの数（ICISRも含む） */

typedef struct isr_initialization_block {
	const INTINIB	*p_intinib;     /* 割込み要求ライン初期化ブロックへのポインタ */
	const OSAPINIB	*p_osapinib;    /* 所属するOSアプリケーション */
} ISRINIB;

struct isr_control_block {
	const ISRINIB	*p_isrinib;
	RESCB			*p_lastrescb;   /* 最後に獲得したリソース管理ブロックへのポインタ */
	SPNCB			*p_lastspncb;   /* 最後に取得したスピンロック管理ブロックへのポインタ */
};

/*
 *  C2ISRの初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const ISRINIB	isrinib_table[];

/*
 *  C2ISRの管理ブロックのエリア（Os_Lcfg.c）
 */
extern ISRCB * const	p_isrcb_table[];

/*
 *  割込み管理機能の初期化
 */
extern void interrupt_initialize(void);

/*
 *  割込み禁止の強制解除
 */

extern void release_interrupts(OSServiceIdType serviceId);

/*
 *  C2ISR終了時のチェック関数
 */
extern void exit_isr2(void);

#endif /* TOPPERS_INTERRUPT_H */

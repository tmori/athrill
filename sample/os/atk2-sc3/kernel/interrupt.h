/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2015 by Witz Corporation
 *  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
 *  $Id: interrupt.h 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		割込み管理機能
 */

#ifndef TOPPERS_INTERRUPT_H
#define TOPPERS_INTERRUPT_H

#include "resource.h"
#include "osap.h"

/*
 *  優先度値の定義（内部表現）
 */
#define TPRI_MINISR		(-1)            /* 最低割込み優先度 */

/*
 *  ISRIDからISRCBを取り出すためのマクロ
 */
#define get_isrcb(isrid)	(&(isrcb_table[(isrid)]))

/*
 *  ISRCBからISRIDを取り出すためのマクロ
 */
#define ISR2ID(p_isrcb)	((ISRType) ((p_isrcb) - isrcb_table))

#ifndef OMIT_INITIALIZE_INTERRUPT

/*
 *  割込み要求ライン初期化ブロック
 */
typedef struct interrupt_request_initialization_block {
	InterruptNumberType	intno;          /* 割込み番号 */
	AttributeType		intatr;         /* 割込み属性 */
	PriorityType		intpri;         /* 割込み優先度 */
#if defined(TOPPERS_CFG1_OUT) || defined(CFG_USE_STACKMONITORING)
	MemorySizeType remain_stksz;        /* スタック残量チェック方式用スタックサイズ */
#endif /* defined(TOPPERS_CFG1_OUT) || defined(CFG_USE_STACKMONITORING) */
} INTINIB;

/*
 *  割込み要求ラインの数（Os_Lcfg.c）
 */
extern const InterruptNumberType	tnum_intno;

/*
 *  割込み要求ライン初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const INTINIB				intinib_table[];

/*
 *  ISRCBから割込み番号を割出すマクロ
 */
#define GET_INTNO(p_isrcb)	((p_isrcb)->p_isrinib->p_intinib->intno)

/*
 *  ISRCBから割込み優先度を割出すマクロ
 */
#define GET_INTPRI(p_isrcb)	((p_isrcb)->p_isrinib->p_intinib->intpri)

#endif /* OMIT_INITIALIZE_INTERRUPT */

typedef struct isr_initialization_block {
	const INTINIB	*p_intinib;     /* 割込み要求ライン初期化ブロックへのポインタ */
	OSAPCB			*p_osapcb;      /* 所属するOSアプリケーションの管理ブロック */
	uint32			acsbtmp;        /* アクセス許可OSアプリケーション ビットマップ */
} ISRINIB;

typedef struct isr_control_block {
	const ISRINIB	*p_isrinib;
	RESCB			*p_lastrescb;   /* 最後に獲得したリソース管理ブロックへのポインタ */
#ifdef CFG_USE_PROTECTIONHOOK
	boolean	calltfn;                /* 信頼関数呼び出し中フラグ */
#endif /* CFG_USE_PROTECTIONHOOK */
} ISRCB;

/*
 *  C2ISR数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const ISRType	tnum_isr2;                         /* C2ISRの数 */

/*
 *  C2ISRの初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const ISRINIB	isrinib_table[];

/*
 *  C2ISRの管理ブロックのエリア（Os_Lcfg.c）
 */
extern ISRCB			isrcb_table[];

/*
 *  実行中のC2ISR
 *
 *  C2ISRを実行していない時は，NULL にする
 */
extern ISRCB			*p_runisr;

/*
 *  SuspendAllInterrupts のネスト回数
 */
extern uint8			sus_all_cnt;

/*
 *  SuspendOSInterrupts のネスト回数
 */
extern uint8			sus_os_cnt;

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

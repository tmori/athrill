/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2015 by Witz Corporation
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
 *  $Id: osap.h 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		OSアプリケーション管理機能
 */

#ifndef TOPPERS_OSAP_H
#define TOPPERS_OSAP_H

/*
 *  OSアプリケーションの属性
 */
#define TA_NONTRUSTED			(FALSE)
#define TA_TRUSTED				(TRUE)

/*
 *  OSアプリケーションIDからOSAPINIBを取り出すためのマクロ
 */
#define get_osapinib(osapid)	(&(osapinib_table[(osapid)]))

/*
 *  OSAPIDからOSAPCBを取り出すためのマクロ
 */
#define get_osapcb(osapid)		(p_osapcb_table[(osapid)])

/*
 *  OSAPCBからOSアプリケーションIDを取り出すためのマクロ
 */
#define OSAPID(p_osapcb)	((ApplicationType) (((p_osapcb)->p_osapinib) - (osapinib_table)))

#ifndef TOPPERS_MACRO_ONLY

/*
 *  OSアプリケーション制御ブロック
 *
 *  OSアプリケーションに関する情報を，OSアプリケーション制御ブロックと
 *  して定義する
 *  他のオブジェクトは，ROMに置く初期化ブロックとRAMに置く制御ブロック
 *  で構成されているが，OSアプリケーションに関する情報は実行時に変更さ
 *  れることがないため，初期化ブロックを制御ブロックとして使用する
 *
 *  ATK2-SC3では，メモリプロテクション機能のための情報を持たせるが，
 *  メモリプロテクション実装はCPUに依存するため，メモリプロテクション
 *  情報の本体は機種依存部に持たせる
 */
typedef struct os_application_initialization_block {
	TCB			*p_restart_tcb;                         /* OSAPのリスタートタスク管理ブロックへのポインタ */
	boolean		osap_trusted;                           /* OSアプリケーションの属性 */
	CoreIdType	coreid;                                 /* OSアプリケーションの割付けコア */
	uint32		btptn;                                  /* 非信頼OSアプリケーションのビットパターン */

#ifndef OMIT_OSAPMPUINFOB
	OSAPMPUINFOB osap_mpu;                              /* OSアプリケーションのMPU情報 */
#endif

} OSAPINIB;

/*
 *  OSアプリケーション管理ブロック
 */
struct os_application_control_block {
	const OSAPINIB			*p_osapinib;                /* 初期化ブロックへのポインタ */
	ApplicationStateType	osap_stat;                  /* OSAP状態 */
};

/*
 *  OSアプリケーション数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const ApplicationType			tnum_osap;      /* OSアプリケーションの数 */

extern const TrustedFunctionIndexType	tnum_tfn;       /* 信頼関数の数 */

typedef StatusType (*TrustedFunctionRefType)(TrustedFunctionIndexType FunctionIndex,
											 TrustedFunctionParameterRefType FunctionParams);

/*
 *  信頼関数初期化ブロック
 */
typedef struct trusted_function_initialization_block {
	TrustedFunctionRefType	trs_func;   /* 信頼関数の起動番地 */
	MemorySizeType			tf_stksz;   /* スタックサイズ */
	CoreIdType				coreid;     /* 割付けコア */
} TFINIB;

/*
 *  OSAPCBのエリア（Os_Lcfg.c）
 */
extern OSAPCB * const	p_osapcb_table[];


/*
 *  OSAPINIBの外部参照（kernel_mem.c）
 */
extern const OSAPINIB	osapinib_table[];

/*
 *  TFINIBの外部参照（Os_Lcfg.c）
 */
extern const TFINIB		tfinib_table[];

/*
 *  指定OSAPを終了/再起動する内部関数(コア間割込みからの呼び出し)
 */
extern void term_osap_for_other_core(void);

/*
 *  OSアプリケーション管理モジュールの初期化
 */
extern void osap_initialize(void);

/*
 *  リスタートタスクのpcにforce_term_osap_mainのアドレスを格納する
 */
extern void force_term_osap(OSAPCB *p_osapcb, RestartType RestartOption, CCB *p_ccb, CCB *p_my_ccb);

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_OSAP_H_ */

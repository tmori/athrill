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
 *  Copyright (C) 2011-2017 by Witz Corporation
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
 *  $Id: kernel_impl.h 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		ATK2内部向け標準ヘッダファイル
 *
 *  このヘッダファイルは，カーネルを構成するプログラムのソースファイル
 *  で必ずインクルードするべき標準ヘッダファイルである
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく
 *  これにより，マクロ定義以外を除くようになっている
 */

#ifndef TOPPERS_KERNEL_IMPL_H
#define TOPPERS_KERNEL_IMPL_H

#include "kernel_rename.h"

/*
 *  アプリケーションと共通のヘッダファイル
 */
#define OMIT_INCLUDE_OS_LCFG
#include "Os.h"

/* 無効ポインタ */
#ifndef NULL
#define NULL		NULL_PTR
#endif /* NULL */

/*
 *  型キャストを行うマクロの定義
 */
#ifndef CAST
#define CAST(type, val)		((type) (val))
#endif /* CAST */

#ifndef TOPPERS_MACRO_ONLY

/* 最適化するため，依存部再定義できる型 */
#ifndef OMIT_DATA_TYPE
/*
 *  カーネル内部のデータ型
 */
typedef uint32	InterruptNumberType;            /* 割込み番号 */
typedef uint32	AttributeType;                  /* オブジェクトの属性 */
typedef sint32	PriorityType;                   /* 優先度 */
#endif /* OMIT_DATA_TYPE */

typedef void (*FunctionRefType)(void);          /* プログラムの起動番地 */

typedef struct task_control_block TCB;          /* TCB 解決のための宣言 */

#ifdef TOPPERS_ENABLE_TRACE
extern void log_dsp_enter(const TCB *p_tcb);
extern void log_dsp_leave(const TCB *p_tcb);
#endif

/*
 *  エラーフックOFF時，サービスID取得とパラメータ取得もOFFになる
 */
#ifdef CFG_USE_ERRORHOOK

#ifdef CFG_USE_PARAMETERACCESS
extern ErrorHook_Par	temp_errorhook_par1;
extern ErrorHook_Par	temp_errorhook_par2;
extern ErrorHook_Par	temp_errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

#endif /* CFG_USE_ERRORHOOK */

/*
 *  OS内部からのShutdownOSの呼び出し
 */
extern void internal_shutdownos(StatusType ercd);

#ifdef CFG_USE_SHUTDOWNHOOK
extern void internal_call_shtdwnhk(StatusType ercd);
#endif /* CFG_USE_SHUTDOWNHOOK */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ASSERTマクロ
 */
#ifndef NDEBUG
#define ASSERT(exp) do {								\
		if (!(exp)) {									\
			fatal_file_name = __FILE__;					\
			fatal_line_num = __LINE__;					\
			internal_shutdownos(E_OS_SYS_ASSERT_FATAL);	\
		}												\
} while (0)

#define ASSERT_NO_REACHED do {						\
		fatal_file_name = __FILE__;					\
		fatal_line_num = __LINE__;					\
		internal_shutdownos(E_OS_SYS_ASSERT_FATAL);	\
} while (0)

#else /* NDEBUG */
#define ASSERT(exp)
#define ASSERT_NO_REACHED
#endif /* NDEBUG */

/*
 *  ターゲット依存情報の定義
 */
#include "target_config.h"

/*
 *  すべての関数をコンパイルするための定義
 */
#ifdef ALLFUNC
#include "allfunc.h"
#endif /* ALLFUNC */

/*
 *  アプリケーションモード値の定義
 */
#define APPMODE_NONE	((AppModeType) 0)   /* モードなし */

/*
 *  実行中のコンテキスト（callevel_statの下位12ビット）の値の定義
 *  TCL_NULLの時に，本来の呼出下コンテキストが判別できなくなることに注意
 */
#define TCL_NULL		UINT_C(0x0000)      /* システムサービスを呼び出せない */
#define TCL_TASK		UINT_C(0x0001)      /* タスク */
#define TCL_ISR2		UINT_C(0x0002)      /* C2ISR */
#define TCL_PROTECT		UINT_C(0x0004)      /* ProtectionHook */
#define TCL_PREPOST		UINT_C(0x0008)      /* PreTaskHook，PostTaskHook */
#define TCL_STARTUP		UINT_C(0x0010)      /* StartupHook */
#define TCL_SHUTDOWN	UINT_C(0x0020)      /* ShutdownHook */
#define TCL_ERROR		UINT_C(0x0040)      /* ErrorHook */
#define TCL_ALRMCBAK	UINT_C(0x0080)      /* Alarm CallBack routine */
#define TCLMASK			UINT_C(0x0fff)      /* コールレベルを示すビットのマスク */
/*
 *  システム状態 (callevel_statの上位4ビット)の値の定義
 */
#define TSYS_NULL		UINT_C(0x0000)      /* システム状態クリア */
#define TSYS_DISALLINT	UINT_C(0x1000)      /* DisableAllInterrupts発行中 */
#define TSYS_SUSALLINT	UINT_C(0x2000)      /* SuspendAllInterrupts発行中 */
#define TSYS_SUSOSINT	UINT_C(0x4000)      /* SuspendOSInterrupts発行中 */
#define TSYS_ISR1		UINT_C(0x8000)      /* C1ISR起動済み */
#define TSYSMASK		UINT_C(0xf000)      /* システム状態を示すビットのマスク */


#ifdef CFG_USE_STACKMONITORING
#ifndef STACK_MAGIC_NUMBER
/*
 *  スタックモニタリング用マジックナンバーの定義
 *  ターゲット依存部の定義は優先される
 */
#define STACK_MAGIC_NUMBER	0x4E434553      /* NCESのASCIIコード(0x4E434553) */
#endif /* STACK_MAGIC_NUMBER */

#ifndef TOPPERS_ISTK_MAGIC_REGION
/* 割込みスタック用マジックナンバー領域取得マクロ */
#define TOPPERS_ISTK_MAGIC_REGION(stk, stksz)	(stk)
#endif /* TOPPERS_ISTK_MAGIC_REGION */

#ifndef TOPPERS_TSTK_MAGIC_REGION
/* タスクスタック用マジックナンバー領域取得マクロ */
#ifndef USE_TSKINICTXB
#define TOPPERS_TSTK_MAGIC_REGION(p_tinib)	((StackType *) ((p_tinib)->stk))
#endif /* USE_TSKINICTXB */
#endif /* TOPPERS_TSTK_MAGIC_REGION */

#endif /* CFG_USE_STACKMONITORING */

/*
 *  callevel_statのビット操作
 */
#define ENTER_CALLEVEL(bit)		(callevel_stat |= (bit))
#define LEAVE_CALLEVEL(bit)		(callevel_stat &= (uint16) ~(uint32) (bit))


/*
 *  各システムサービスを呼び出せる処理単位
 */
#define CALLEVEL_ACTIVATETASK				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_TERMINATETASK				(TCL_TASK)
#define CALLEVEL_CHAINTASK					(TCL_TASK)
#define CALLEVEL_SCHEDULE					(TCL_TASK)
#define CALLEVEL_GETTASKID					(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_PROTECT)
#define CALLEVEL_GETTASKSTATE				(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST)
#define CALLEVEL_GETRESOURCE				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_RELEASERESOURCE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_SETEVENT					(TCL_TASK | TCL_ISR2)
#define CALLEVEL_CLEAREVENT					(TCL_TASK)
#define CALLEVEL_GETEVENT					(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST)
#define CALLEVEL_WAITEVENT					(TCL_TASK)
#define CALLEVEL_GETALARMBASE				(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST)
#define CALLEVEL_GETALARM					(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST)
#define CALLEVEL_SETRELALARM				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_SETABSALARM				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_CANCELALARM				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETACTIVEAPPMODE			(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN)
#define CALLEVEL_SHUTDOWNOS					(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_STARTUP)
#define CALLEVEL_GETISRID					(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_INCREMENTCOUNTER			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETCOUNTERVALUE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETELAPSEDVALUE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STARTSCHEDULETABLEREL		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STARTSCHEDULETABLEABS		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STOPSCHEDULETABLE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_NEXTSCHEDULETABLE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETSCHEDULETABLESTATUS		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_DISABLEINTERRUPTSOURCE		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT | TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_ENABLEINTERRUPTSOURCE		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT | TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_GETFAULTYCONTEXT			(TCL_PROTECT)

/*
 *  その他の定数値（標準割込みモデル）
 */
#define TIPM_ENAALL		UINT_C(0)   /* 割込み優先度マスク全解除 */

/*
 *  オブジェクト属性の定義（標準割込みモデル）
 */
#define ENABLE		UINT_C(0x01)
#define DISABLE		UINT_C(0x00)

/*
 *  OS内部用無効なシステムサービスID
 */
#define OSServiceId_Invalid		((OSServiceIdType) 0xff)

/*
 *  ヘッダファイルを持たないモジュールの関数・変数の宣言
 */
#ifndef TOPPERS_MACRO_ONLY

#ifdef TOPPERS_StartOS
/*
 *  アプリケーションモードの数
 */
extern const AppModeType	tnum_appmode;

#endif /* TOPPERS_StartOS */

/*
 *  OS実行制御のための変数（osctl_manage.c）
 */
extern uint16				callevel_stat;  /* 実行中のコンテキスト */
extern AppModeType			appmodeid;      /* アプリケーションモードID */

/*
 *  カーネル動作状態フラグ
 */
extern boolean				kerflg;

/*
 *  エラーフック呼び出しのための宣言（osctl.c）
 */
#ifdef CFG_USE_ERRORHOOK
extern void internal_call_errorhook(StatusType ercd, OSServiceIdType svcid);
#endif /* CFG_USE_ERRORHOOK */

/*
 *  ポストタスクフック/プレタスクフック
 *  スタックモニタリング機能の初期化/プロテクションフック呼び出しのための宣言（osctl.c）
 */
#ifdef CFG_USE_POSTTASKHOOK
extern void call_posttaskhook(void);
#endif /* CFG_USE_POSTTASKHOOK */

#ifdef CFG_USE_PRETASKHOOK
extern void call_pretaskhook(void);
#endif /* CFG_USE_PRETASKHOOK */

#ifdef CFG_USE_STACKMONITORING
extern void init_stack_magic_region(void);
#endif /* CFG_USE_STACKMONITORING */

extern void call_protectionhk_main(StatusType ercd);

/*
 *  各モジュールの初期化（Os_Lcfg.c）
 */
extern void object_initialize(void);

/*
 *  各モジュールの終了処理（Os_Lcfg.c）
 */
extern void object_terminate(void);

/*
 *  非タスクコンテキスト用のスタック領域（Os_Lcfg.c）
 */
extern const MemorySizeType	ostksz;             /* スタック領域のサイズ（丸めた値） */
extern StackType * const	ostk;               /* スタック領域の先頭番地 */
#ifdef TOPPERS_OSTKPT
extern StackType * const	ostkpt;                /* スタックポインタの初期値 */
#endif /* TOPPERS_OSTKPT */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_KERNEL_IMPL_H */

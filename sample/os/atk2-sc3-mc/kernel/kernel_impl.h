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
 *  $Id: kernel_impl.h 485 2015-12-17 08:21:50Z witz-itoyo $
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

/*
 *  カーネル内部用リネームを使用するための定義
 */
#ifndef TOPPERS_SVC_FUNCCALL
#define TOPPERS_SVC_FUNCCALL
#endif

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

/*
 *  優先度の範囲（外部表現）
 */
#define TMIN_TPRI		UINT_C(0)       /* タスク優先度の最小値（最低値）*/
#define TMAX_TPRI		UINT_C(15)      /* タスク優先度の最大値（最高値）*/

/*
 *  優先度の段階数の定義
 */
#define TNUM_TPRI		((TMAX_TPRI - TMIN_TPRI) + 1U)

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

/*
 *  OS内部からのShutdownAllCoresの呼び出し
 */
extern void internal_shutdownallcores(StatusType ercd);

#ifdef CFG_USE_SHUTDOWNHOOK
extern void internal_call_shtdwnhk(StatusType ercd);
#endif /* CFG_USE_SHUTDOWNHOOK */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ASSERTマクロ
 */
#ifndef NDEBUG
#define ASSERT(exp) do {									  \
		if (!(exp)) {										  \
			fatal_file_name = __FILE__;						  \
			fatal_line_num = __LINE__;						  \
			internal_shutdownallcores(E_OS_SYS_ASSERT_FATAL); \
		}													  \
} while (0)

#define ASSERT_NO_REACHED do {							  \
		fatal_file_name = __FILE__;						  \
		fatal_line_num = __LINE__;						  \
		internal_shutdownallcores(E_OS_SYS_ASSERT_FATAL); \
} while (0)

#else /* NDEBUG */
#define ASSERT(exp)
#define ASSERT_NO_REACHED
#endif /* NDEBUG */

/*
 *  カーネルロックの方式（TTYPE_KLOCK）
 */
#define G_KLOCK		UINT_C(0x01)
#define C_KLOCK		UINT_C(0x02)

/*
 *  CCB関連の定義
 */
#include "ccb.h"

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
#define TCL_NULL			UINT_C(0x0000)                          /* システムサービスを呼び出せない */
#define TCL_TASK			UINT_C(0x0001)                          /* タスク */
#define TCL_ISR2			UINT_C(0x0002)                          /* C2ISR */
#define TCL_PROTECT			UINT_C(0x0004)                          /* ProtectionHook */
#define TCL_PREPOST			UINT_C(0x0008)                          /* PreTaskHook，PostTaskHook */
#define TCL_STARTUP			UINT_C(0x0010)                          /* StartupHook */
#define TCL_SHUTDOWN		UINT_C(0x0020)                          /* ShutdownHook */
#define TCL_ERROR			UINT_C(0x0040)                          /* ErrorHook */
/* OSアプリケーションに所属のフックマスク */
#define TCLMASK				UINT_C(0x0fff)                          /* コールレベルを示すビットのマスク */

/*
 *  システム状態 (callevel_statの上位4ビット)の値の定義
 */
#define TSYS_NULL			UINT_C(0x0000)      /* システム状態クリア */
#define TSYS_DISALLINT		UINT_C(0x1000)      /* DisableAllInterrupts発行中 */
#define TSYS_SUSALLINT		UINT_C(0x2000)      /* SuspendAllInterrupts発行中 */
#define TSYS_SUSOSINT		UINT_C(0x4000)      /* SuspendOSInterrupts発行中 */
#define TSYS_ISR1			UINT_C(0x8000)      /* C1ISR起動済み */
#define TSYSMASK			UINT_C(0xf000)      /* システム状態を示すビットのマスク */


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

#ifndef TOPPERS_SSTK_MAGIC_REGION
/* 信頼タスクスタック用マジックナンバー領域取得マクロ */
#ifndef USE_TSKINICTXB
#define TOPPERS_SSTK_MAGIC_REGION(p_tinib)	((StackType *) ((p_tinib)->sstk))
#endif /* USE_TSKINICTXB */
#endif /* TOPPERS_SSTK_MAGIC_REGION */

#endif /* CFG_USE_STACKMONITORING */

#define TFN_EXIT_TASK	(TMAX_SVCID - TARGET_SVC_NUM)
#define exit_task	(_kernel_exit_task)

/*
 *  フック種別の定義
 */
#define STARTUP_HOOK	UINT_C(0)
#define SHUTDOWN_HOOK	UINT_C(1)
#define ERROR_HOOK	UINT_C(2)

/*
 *  callevel_statのビット操作
 */
#define ENTER_CALLEVEL(bit)		((get_my_p_ccb()->callevel_stat) |= (bit))
#define LEAVE_CALLEVEL(bit)		((get_my_p_ccb()->callevel_stat) &= (uint16) ~(uint32) (bit))


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
#define CALLEVEL_GETISRID					(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_INCREMENTCOUNTER			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETCOUNTERVALUE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETELAPSEDVALUE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STARTSCHEDULETABLEREL		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STARTSCHEDULETABLEABS		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_STOPSCHEDULETABLE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_NEXTSCHEDULETABLE			(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETSCHEDULETABLESTATUS		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_GETAPPLICATIONID			(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT)
#define CALLEVEL_CALLTRUSTEDFUNCTION		(TCL_TASK | TCL_ISR2)
#define CALLEVEL_CHECKISRMEMORYACCESS		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_CHECKTASKMEMORYACCESS		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_CHECKOBJECTACCESS			(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_CHECKOBJECTOWNERSHIP		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PROTECT)
#define CALLEVEL_GETAPPLICATIONSTATE		(TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT | TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT)
#define CALLEVEL_DISABLEINTERRUPTSOURCE		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT | TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_ENABLEINTERRUPTSOURCE		(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_PROTECT | TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_GETSPINLOCK				(TCL_TASK | TCL_ISR2 | TCL_PROTECT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_RELEASESPINLOCK			(TCL_TASK | TCL_ISR2 | TCL_PROTECT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_TRYTOGETSPINLOCK			(TCL_TASK | TCL_ISR2 | TCL_PROTECT | TSYS_SUSALLINT | TSYS_SUSOSINT)
#define CALLEVEL_SHUTDOWNALLCORES			(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_STARTUP)
#define CALLEVEL_GETFAULTYCONTEXT			(TCL_PROTECT)
#define CALLEVEL_IOCSEND					(TCL_TASK | TCL_ISR2)
#define CALLEVEL_IOCWRITE					(TCL_TASK | TCL_ISR2)
#define CALLEVEL_IOCRECEIVE					(TCL_TASK | TCL_ISR2)
#define CALLEVEL_IOCREAD					(TCL_TASK | TCL_ISR2)
#define CALLEVEL_IOCEMPTYQUEUE				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_RAISEINTERCOREINTERRUPT	(TCL_TASK | TCL_ISR2 | TCL_ERROR | TCL_PREPOST | TCL_PROTECT | TSYS_SUSALLINT | TSYS_SUSOSINT | TSYS_DISALLINT)
#define CALLEVEL_ALLOWACCESS				(TCL_TASK | TCL_ISR2)
#define CALLEVEL_TERMINATEAPPLICATION		(TCL_TASK | TCL_ISR2)


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
extern const AppModeType tnum_appmode;

#endif /* TOPPERS_StartOS */


extern void p_inib_initialize(void);

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

extern void call_protectionhk_main(StatusType protection_error);

/*
 *  各モジュールの初期化（Os_Lcfg.c）
 */
extern void object_initialize(void);

/*
 *  各モジュールの終了処理（Os_Lcfg.c）
 */
extern void object_terminate(void);

/*
 *  OS実行制御のための変数（osctl_manage.c）
 */
extern AppModeType			appmodeid; /* アプリケーションモードID */

/*
 *  非タスクコンテキスト用のスタック領域（Os_Lcfg.c）
 */
extern const MemorySizeType	_ostksz_table[];        /* スタック領域のサイズ（丸めた値） */
extern StackType * const	_ostk_table[];          /* スタック領域の先頭番地 */
#ifdef TOPPERS_OSTKPT
extern StackType * const	_ostkpt_table[];        /* スタックポインタの初期値 */
#endif /* TOPPERS_OSTKPT */

/*
 *  ビットマップサーチ関数
 *
 *  bitmap内の1のビットの内，最も下位（右）のものをサーチし，そのビッ
 *  ト番号を返す
 *  ビット番号は，最下位ビットを0とする．bitmapに0を指定
 *  してはならない．この関数では，bitmapが16ビットであることを仮定し，
 *  uint16型としている
 *
 *  ビットサーチ命令を持つプロセッサでは，ビットサーチ命令を使うように
 *  書き直した方が効率が良い場合がある
 *  このような場合には，ターゲット依存部でビットサーチ命令を使った
 *  bitmap_searchを定義し，OMIT_BITMAP_SEARCHをマクロ定義すればよい
 *  また，ビットサーチ命令のサーチ方向が逆などの理由で優先度とビット
 *  との対応を変更したい場合には，PRIMAP_BITをマクロ定義すればよい
 *
 *  また，標準ライブラリにffsがあるなら，次のように定義して標準ライブ
 *  ラリを使った方が効率が良い可能性もある
 *		#define	bitmap_search(bitmap) (ffs(bitmap) - 1)
 */
#ifndef PRIMAP_BIT
#define PRIMAP_BIT(pri)		((uint16) ((uint16) 1U << (pri)))
#endif /* PRIMAP_BIT */

#ifndef OMIT_BITMAP_SEARCH
#define BITMAP_NUM		15       /* bitmap_search_tableのサイズ */

LOCAL_INLINE uint16
bitmap_search(uint16 bitmap)
{
	/*
	 *  ビットマップサーチ関数用テーブル
	 */
	const uint8	bitmap_search_table[BITMAP_NUM] = {
		0U, 1U, 0U, 2U, 0U, 1U, 0U,
		3U, 0U, 1U, 0U, 2U, 0U, 1U, 0U
	};

	uint16		n = 0U;

	ASSERT(bitmap != 0U);
	if ((bitmap & 0x00ffU) == 0U) {
		bitmap >>= 8U;
		n += 8U;
	}
	if ((bitmap & 0x0fU) == 0U) {
		bitmap >>= 4U;
		n += 4U;
	}
	return(n + bitmap_search_table[(bitmap & 0x000fU) - 1U]);
}

#endif /* OMIT_BITMAP_SEARCH */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_KERNEL_IMPL_H */

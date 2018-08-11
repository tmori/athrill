/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
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
 *  $Id: scheduletable.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		スケジュールテーブル機能
 */

#ifndef TOPPERS_SCHEDULETABLE_H
#define TOPPERS_SCHEDULETABLE_H

#include "counter.h"

/*
 *  満了点テーブル制御用特殊な満了点インデックス
 */
#define EXPPTINDEX_TOP			((uint8) 0x00)
#define EXPPTINDEX_INITIAL		((uint8) 0xff)

/*
 *  スケジュールテーブルIDからスケジュールテーブル管理ブロックを取り出すためのマクロ
 */
#define get_schtblcb(schtblid)	(&(schtblcb_table[(schtblid)]))

/*
 *  暗黙同期スケジュールテーブルに関する定義
 */
#define is_implschtbl(schtblid)	((schtblid) < tnum_implscheduletable)

/*
 *  個々の満了点テーブル型
 */
typedef struct scheduletable_expire_point_block {
	TickType		offset;                                 /* オフセット値 */
	FunctionRefType	expptfnt;                               /* 満了点処理関数のポインタ */
} SCHTBLEXPPTCB;


/*
 *  スケジュールテーブル初期化ブロック
 */
typedef struct scheduletable_initialization_block {
	CNTCB				*p_cntcb;                           /* 駆動カウンタ管理ブロックのポインタ */
	TickType			length;                             /* 周期の長さ */
	AppModeType			autosta;                            /* 起動するアプリケーションモード */
	AttributeType		actatr;                             /* 自動起動の属性 */
	TickType			staval;                             /* 自動起動ティック値 */
	const SCHTBLEXPPTCB	*p_exppt;                           /* 満了点テーブルの先頭ポインタ */
	boolean				repeat;                             /* 周期制御の有無 */
	uint8				tnum_exppt;                         /* 満了点数 */
} SCHTBLINIB;

/*
 *  スケジュールテーブル管理ブロック
 */
typedef struct scheduletable_control_block {
	CNTEXPINFO							cntexpinfo;         /* カウンタ満了情報(構造体の先頭に入る必要) */
	const SCHTBLINIB					*p_schtblinib;      /* 初期化ブロックへのポインタ */
	struct scheduletable_control_block	*p_prevschtblcb;    /* 自分をNextにしたスケジュールテーブル管理ブロックへのポインタ */
	struct scheduletable_control_block	*p_nextschtblcb;    /* Nextスケジュールテーブル管理ブロックへのポインタ */
	ScheduleTableStatusType				status;             /* スケジュールテーブル状態 */
	uint8								expptindex;         /* 満了点インデックス */
} SCHTBLCB;

/*
 *  満了処理実行用管理情報
 */
typedef struct scheduletable_expire_info {
	SCHTBLCB *p_schtblcb;                                   /* スケジュールテーブル管理ブロックのアドレス */
} SCHTBLEXPINFO;

/*
 *  スケジュールテーブル数を保持する変数の宣言
 */
extern const ScheduleTableType	tnum_scheduletable;         /* 全スケジュールテーブルの数 */
extern const ScheduleTableType	tnum_implscheduletable;     /* 暗黙同期スケジュールテーブル数 */

/*
 *  スケジュールテーブル初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const SCHTBLINIB			schtblinib_table[];
/*
 *  スケジュールテーブル管理ブロックのエリア（Os_Lcfg.c）
 */
extern SCHTBLCB					schtblcb_table[];

/*
 *  スケジュールテーブルオブジェクトの初期化
 */
extern void schtbl_initialize(void);

/*
 *  スケジュールテーブル満了処理関数
 */
extern void schtbl_expire(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb);

/*
 *  満了処理関数から各タイミング処理の実行
 */
extern void schtbl_expiry_process(SCHTBLEXPINFO *p_schtblexpinfo, const CNTCB *p_cntcb);

/*
 *  スケジュールテーブルの開始処理
 */
extern boolean schtbl_head(SCHTBLCB *p_schtblcb, const CNTCB *p_cntcb);

/*
 *  スケジュールテーブルの各満了点処理
 */
extern boolean schtbl_exppoint_process(SCHTBLCB *p_schtblcb, const CNTCB *p_cntcb);

/*
 *  スケジュールテーブルの終端処理
 */
extern boolean schtbl_tail(SCHTBLCB *p_schtblcb, SCHTBLEXPINFO *p_schtblexpinfo, const CNTCB *p_cntcb);

#endif /* TOPPERS_SCHEDULETABLE_H */

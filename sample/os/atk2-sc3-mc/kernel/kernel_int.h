/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2015 by Center for Embedded Computing Systems
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
 *  $Id: kernel_int.h 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		Os_Lcfg.c（およびcfg1_out.c）用ヘッダファイル
 */

#ifndef TOPPERS_KERNEL_INT_H
#define TOPPERS_KERNEL_INT_H

/*
 *  カーネル標準ヘッダファイル
 */
#include "kernel_impl.h"

/*
 *  カーネルの各ヘッダファイル
 */
#include "task.h"
#include "counter.h"
#include "alarm.h"
#include "interrupt.h"
#include "resource.h"
#include "scheduletable.h"
#include "memory.h"
#include "spinlock.h"
#include "ioc_impl.h"

/*
 *  メインルーチン名定義用のマクロ
 */
#define TASKNAME(TaskName)	TaskMain ## TaskName
#define ISRNAME(ISRName)	ISRMain  ## ISRName
#define ICISRNAME(ICIName)	ICIMain  ## ICIName

/*
 *  オブジェクトの属性の定義（コンフィギュレーションファイルでのみ使用）
 */

/*
 *  イベントマスク値
 */
#define AUTO		UINT_C(0x00)    /* イベントマスク値=AUTO */

#define TA_NOWRITE		UINT_C(0x01)    /* 書込みアクセス禁止 */
#define TA_NOREAD		UINT_C(0x02)    /* 読出しアクセス禁止 */
#define TA_EXEC			UINT_C(0x04)    /* 実行アクセス許可 */
#define TA_MEMINI		UINT_C(0x08)    /* メモリの初期化を行う */
#define TA_MEMPRSV		UINT_C(0x10)    /* メモリの初期化を行わない */
#define TA_SDATA		UINT_C(0x20)    /* ショートデータ領域に配置 */
#define TA_UNCACHE		UINT_C(0x40)    /* キャッシュ不可 */
#define TA_IODEV		UINT_C(0x80)    /* 周辺デバイスの領域 */

#define NO_STANDARD		UINT_C(0x0400)      /* 標準RAMリージョン */
#ifdef TOPPERS_TARGET_SUPPORT_ATT_MOD
#define TOPPERS_SUPPORT_ATT_MOD         /* ATT_MODがサポートされている */
#endif /* TOPPERS_TARGET_SUPPORT_ATT_MOD */

/*
 * OSアプリケーションID（HRP2の保護ドメインID）
 */
#define TDOM_KERNEL		(-1)            /* カーネルドメイン */
#define TDOM_NONE		(-2)            /* 無所属（保護ドメインに属さない）*/

#define TA_NULL			UINT_C(0)       /* オブジェクト属性を指定しない */

/*
 * OSアプリケーション関係のマクロ
 */
#define TMAX_NTOSAPP		32          /* 非信頼OSアプリケーションの最大数 */

#endif /* TOPPERS_KERNEL_INT_H */

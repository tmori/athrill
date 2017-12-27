/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2011-2017 by Center for Embedded Computing Systems
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
 *  $Id: Std_Types.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		AUTOSAR仕様共通規定のデータ型・定数・マクロ
 *
 *  このヘッダファイルは，AUTOSAR仕様共通規定のデータ型・定数・マクロ
 *  の定義を含むヘッダファイル
 *  AUTOSAR仕様との互換性を必要とするアプリケーションがインクルード
 *  することを想定している
 *  ATK2のコンパイルに必要な共通の定義も本ヘッダにて定義する
 *
 *  アセンブリ言語のソースファイルからこのファイルをインクルードする時
 *  は，TOPPERS_MACRO_ONLYを定義しておく
 *  これにより，マクロ定義以外を除くようになっている
 */

#ifndef TOPPERS_STD_TYPES_H
#define TOPPERS_STD_TYPES_H

#include "Platform_Types.h"
#include "Compiler.h"

/*
 *  Type definitions
 */
#ifndef TOPPERS_MACRO_ONLY

typedef uint8 Std_ReturnType;

typedef struct {
	uint16	vendorID;
	uint16	moduleID;
	uint8	sw_major_version;
	uint8	sw_minor_version;
	uint8	sw_patch_version;
} Std_VersionInfoType;

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  Symbol definitions
 */
#ifndef STATUSTYPEDEFINED
#define STATUSTYPEDEFINED

#ifndef TOPPERS_MACRO_ONLY
typedef unsigned char StatusType;   /* OSEK compliance */
#endif /* TOPPERS_MACRO_ONLY */

#define E_OK		UINT_C(0x00)
#endif /* STATUSTYPEDEFINED */

#define E_NOT_OK	UINT_C(0x01)

#define STD_HIGH	UINT_C(1)   /* Physical state 5V or 3.3V */
#define STD_LOW		UINT_C(0)   /* Physical state 0V         */

#define STD_ACTIVE	UINT_C(1)   /* Logical state active */
#define STD_IDLE	UINT_C(0)   /* Logical state idle   */

#define STD_ON		UINT_C(1)
#define STD_OFF		UINT_C(0)

#endif /* TOPPERS_STD_TYPES_H */

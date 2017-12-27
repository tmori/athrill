/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: Platform_Types.h 549 2015-12-30 10:06:17Z ertl-honda $
 */

/*
 *		AUTOSAR platform types header file
 */

#ifndef TOPPERS_PLATFORM_TYPES_H
#define TOPPERS_PLATFORM_TYPES_H

/*
 *  Symbol definitions - general
 */
#define CPU_TYPE_8		UINT_C(8)
#define CPU_TYPE_16		UINT_C(16)
#define CPU_TYPE_32		UINT_C(32)

#define MSB_FIRST		UINT_C(0)
#define LSB_FIRST		UINT_C(1)

#define HIGH_BYTE_FIRST	UINT_C(0)
#define LOW_BYTE_FIRST	UINT_C(1)

#ifndef TRUE
#define TRUE			UINT_C(1)
#endif /* TRUE */

#ifndef FALSE
#define FALSE			UINT_C(0)
#endif /* FALSE */

/*
 *  Symbol definitions - platform
 */
#define CPU_TYPE		(CPU_TYPE_32)
#define CPU_BIT_ORDER	(MSB_FIRST)
#define CPU_BYTE_ORDER	(LOW_BYTE_FIRST)

/*
 *  Stack definitions - platform
 */
#define DEFINE_VAR_STACK(type, var)	type __attribute__((aligned(4))) var

/*
 *  Type definitions
 */
#ifndef TOPPERS_MACRO_ONLY

#include <stdint.h>

typedef unsigned char boolean;

typedef char				char8;
typedef unsigned char		uint8;
typedef signed char			sint8;
typedef unsigned short		uint16;
typedef signed short		sint16;
typedef unsigned int		uint32;
typedef signed int			sint32;
typedef unsigned long long	uint64;
typedef signed long long	sint64;

typedef unsigned long	uint8_least;
typedef unsigned long	uint16_least;
typedef unsigned long	uint32_least;
typedef signed long		sint8_least;
typedef signed long		sint16_least;
typedef signed long		sint32_least;

typedef float	float32;
typedef double	float64;

typedef uint32	uintptr;                /* ポインタを格納できる符号無し整数 */
typedef sint32	sintptr;                /* ポインタを格納できる符号付き整数 */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_PLATFORM_TYPES_H */

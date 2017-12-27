/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
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
 *  @(#) $Id: Compiler.h 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		コンパイラ依存定義
 */

#ifndef TOPPERS_COMPILER_H
#define TOPPERS_COMPILER_H

#include "Compiler_Cfg.h"

#define AUTOMATIC
#define TYPEDEF
#define NULL_PTR	((void *) 0)

#define FUNC(rettype, memclass)						rettype
#define FUNC_P2CONST(rettype, ptrclass, memclass)	const rettype *
#define FUNC_P2VAR(rettype, ptrclass, memclass)		rettype *
#define P2VAR(ptrtype, memclass, ptrclass)			ptrtype *
#define P2CONST(ptrtype, memclass, ptrclass)		const ptrtype *
#define CONSTP2VAR(ptrtype, memclass, ptrclass)		ptrtype * const
#define CONSTP2CONST(ptrtype, memclass, ptrclass)	const ptrtype * const
#define P2FUNC(rettype, ptrclass, fctname)			rettype (*fctname)
#define CONST(consttype, memclass)					const consttype
#define VAR(vartype, memclass)						vartype

#define INLINE			__inline__
#define LOCAL_INLINE	static __inline__

#define Asm		asm     /* インラインアセンブラ（最適化抑止）*/

#define NoReturn	__attribute__((__noreturn__))

/* コンパイラ依存のデータ型の定義 */
#ifndef TOPPERS_MACRO_ONLY

#include <limits.h>

/*
 *  空ラベルの定義
 *    ghsでは大きさゼロの配列はコンパイルエラーになるため、
 *    ここで別途定義する。
 */
#define TOPPERS_EMPTY_LABEL(x, y)	x y[1]

#endif /* TOPPERS_MACRO_ONLY */





#endif /* TOPPERS_COMPILER_H */

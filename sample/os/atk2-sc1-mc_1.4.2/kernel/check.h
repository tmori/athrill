/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2017 by Center for Embedded Computing Systems
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
 *  $Id: check.h 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		エラーチェック用マクロ
 */

#ifndef TOPPERS_CHECK_H
#define TOPPERS_CHECK_H

#include "Os_Cfg.h"

/*
 * 汎用チェックマクロ
 */
#define CHECK_ERROR_ERCD(exp, error) \
	do {							 \
		if (!(exp)) {				 \
			ercd = (error);			 \
			goto error_exit;		 \
		}							 \
	} while (0)

#define CHECK_ERROR_NO_ERCD(exp) \
	do {						 \
		if (!(exp)) {			 \
			goto error_exit;	 \
		}						 \
	} while (0)

#define D_CHECK_ERROR_ERCD(exp, error) \
	do {							   \
		if (!(exp)) {				   \
			ercd = (error);			   \
			goto d_error_exit;		   \
		}							   \
	} while (0)

#define D_CHECK_ERROR_NO_ERCD(exp) \
	do {						   \
		if (!(exp)) {			   \
			goto d_error_exit;	   \
		}						   \
	} while (0)

/*
 *  標準エラーのチェックマクロ
 */
#define S_CHECK_ERROR(exp, error)		CHECK_ERROR_ERCD(exp, error)


#ifdef CFG_USE_ERRORHOOK
/*
 *  エラーフック使用有無によるラベルの再定義
 */
#define error_exit		exit_errorhook
#define d_error_exit	d_exit_errorhook

/*
 *  エラーフックを使用する時はエラーコードを代入する
 */
#define S_N_CHECK_ERROR(exp, error)		CHECK_ERROR_ERCD(exp, error)

#else /* CFG_USE_ERRORHOOK */
/*
 *  エラーフック使用有無によるラベルの再定義
 */
#define error_exit		exit_no_errorhook
#define d_error_exit	d_exit_no_errorhook

/*
 *  エラーフックを使用しない時はエラーコードを代入しない
 */
#define S_N_CHECK_ERROR(exp, error)		CHECK_ERROR_NO_ERCD(exp)

#endif /* CFG_USE_ERRORHOOK */


/* 以下のチェックマクロをOS内部で用いる */

/*
 *  エラーコードに対応したマクロ
 *   標準エラー
 */
#ifdef OMIT_STANDARD_DISALLINT

#define S_CHECK_DISALLINT()
#define S_N_CHECK_DISALLINT()

#else /* OMIT_STANDARD_DISALLINT */

#define S_CHECK_DISALLINT()												   \
	S_CHECK_ERROR(														   \
		(((get_my_p_ccb()->callevel_stat) & TSYS_DISALLINT) == TSYS_NULL), \
		E_OS_DISABLEDINT												   \
		)
#define S_N_CHECK_DISALLINT()											   \
	S_N_CHECK_ERROR(													   \
		(((get_my_p_ccb()->callevel_stat) & TSYS_DISALLINT) == TSYS_NULL), \
		E_OS_DISABLEDINT												   \
		)

#endif /* OMIT_STANDARD_DISALLINT */

#define S_CHECK_STATE(exp)				S_CHECK_ERROR(exp, E_OS_STATE)
#define S_N_CHECK_STATE(exp)			S_N_CHECK_ERROR(exp, E_OS_STATE)
#define S_CHECK_LIMIT(exp)				S_CHECK_ERROR(exp, E_OS_LIMIT)
#define S_N_CHECK_LIMIT(exp)			S_N_CHECK_ERROR(exp, E_OS_LIMIT)
#define S_D_CHECK_NOFUNC(exp)			D_CHECK_ERROR_ERCD(exp, E_OS_NOFUNC)
#define S_D_CHECK_STATE(exp)			D_CHECK_ERROR_ERCD(exp, E_OS_STATE)
#define S_D_CHECK_LIMIT(exp)			D_CHECK_ERROR_ERCD(exp, E_OS_LIMIT)

#define S_D_CHECK_IOC_LIMIT(exp) do { \
		if (!(exp)) {				  \
			ercd = IOC_E_LIMIT;		  \
			p_ioccb->lostflg = TRUE;  \
			goto d_error_exit;		  \
		}							  \
} while (0)

#define S_D_CHECK_IOC_LOST(exp) do {  \
		if (!(exp)) {				  \
			ercd = IOC_E_LOST_DATA;	  \
			p_ioccb->lostflg = FALSE; \
			goto d_error_exit;		  \
		}							  \
} while (0)

#define S_D_CHECK_IOC_NO_DATA(exp)		D_CHECK_ERROR_ERCD(exp, IOC_E_NO_DATA)


/*
 *  エラーコードに対応したマクロ
 *   拡張エラー
 */
#ifdef CFG_USE_EXTENDEDSTATUS

#define CHECK_ACCESS(exp)				CHECK_ERROR_ERCD(exp, E_OS_ACCESS)
#define D_CHECK_ACCESS(exp)				D_CHECK_ERROR_ERCD(exp, E_OS_ACCESS)
#define CHECK_CALLEVEL(clmask)										\
	CHECK_ERROR_ERCD(												\
		(((get_my_p_ccb()->callevel_stat) | (clmask)) == (clmask)),	\
		E_OS_CALLEVEL												\
		)
#define CHECK_NOFUNC(exp)				CHECK_ERROR_ERCD(exp, E_OS_NOFUNC)
#define CHECK_RESOURCE(exp)				CHECK_ERROR_ERCD(exp, E_OS_RESOURCE)
#define CHECK_STATE(exp)				CHECK_ERROR_ERCD(exp, E_OS_STATE)
#define D_CHECK_STATE(exp)				D_CHECK_ERROR_ERCD(exp, E_OS_STATE)
#define CHECK_VALUE(exp)				CHECK_ERROR_ERCD(exp, E_OS_VALUE)
#define CHECK_DISABLEDINT()										\
	CHECK_ERROR_ERCD(											\
		(((get_my_p_ccb()->callevel_stat) &						\
		  (TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)) ==	\
		 TSYS_NULL												\
		),														\
		E_OS_DISABLEDINT										\
		)
#define CHECK_PARAM_POINTER(p_exp)		CHECK_ERROR_ERCD(((p_exp) != NULL), OS_E_PARAM_POINTER)
#define CHECK_ID(exp)					CHECK_ERROR_ERCD(exp, E_OS_ID)
#define CHECK_IOC_ID(exp)				CHECK_ERROR_ERCD(exp, IOC_E_ID)
#define CHECK_CORE(exp)					CHECK_ERROR_ERCD(exp, E_OS_CORE)
#define D_CHECK_CORE(exp)				D_CHECK_ERROR_ERCD(exp, E_OS_CORE)
#define CHECK_INTER_DEADLOCK(exp)		CHECK_ERROR_ERCD(exp, E_OS_INTERFERENCE_DEADLOCK)
#define CHECK_NEST_DEADLOCK(exp)		CHECK_ERROR_ERCD(exp, E_OS_NESTING_DEADLOCK)
#define CHECK_SPINLOCK(exp)				CHECK_ERROR_ERCD(exp, E_OS_SPINLOCK)
#define CHECK_IOC_ACCESS(exp)			CHECK_ERROR_ERCD(exp, IOC_E_NOK)
#define CHECK_IOC_CALLEVEL_DISABLEDINT(clmask)						\
	CHECK_ERROR_ERCD(												\
		(((get_my_p_ccb()->callevel_stat) | (clmask)) == (clmask)),	\
		IOC_E_NOK													\
		)

#else /* CFG_USE_EXTENDEDSTATUS */

#define CHECK_ACCESS(exp)
#define D_CHECK_ACCESS(exp)
#define CHECK_CALLEVEL(clmask)
#define CHECK_NOFUNC(exp)
#define CHECK_STATE(exp)
#define CHECK_RESOURCE(exp)
#define D_CHECK_STATE(exp)
#define CHECK_VALUE(exp)
#define CHECK_DISABLEDINT()
#define CHECK_PARAM_POINTER(p_exp)
#define CHECK_ID(exp)
#define CHECK_CORE(exp)
#define D_CHECK_CORE(exp)
#define CHECK_INTER_DEADLOCK(exp)
#define CHECK_NEST_DEADLOCK(exp)
#define CHECK_SPINLOCK(exp)
#define CHECK_IOC_ACCESS(exp)
#define CHECK_IOC_CALLEVEL_DISABLEDINT(clmask)

#endif /* CFG_USE_EXTENDEDSTATUS */

#endif /* TOPPERS_CHECK_H */

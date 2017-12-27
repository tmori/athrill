/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: check.h 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		エラーチェック用マクロ
 */

#ifndef TOPPERS_CHECK_H
#define TOPPERS_CHECK_H

#include "Os_Cfg.h"
#include "memory.h"

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

#define S_CHECK_DISALLINT()								 \
	S_CHECK_ERROR(										 \
		((callevel_stat & TSYS_DISALLINT) == TSYS_NULL), \
		E_OS_DISABLEDINT								 \
		)
#define S_N_CHECK_DISALLINT()							 \
	S_N_CHECK_ERROR(									 \
		((callevel_stat & TSYS_DISALLINT) == TSYS_NULL), \
		E_OS_DISABLEDINT								 \
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
#define CHECK_ACCESS(exp)				CHECK_ERROR_ERCD(exp, E_OS_ACCESS)
#define D_CHECK_ACCESS(exp)				D_CHECK_ERROR_ERCD(exp, E_OS_ACCESS)
#define CHECK_CALLEVEL(clmask)					  \
	CHECK_ERROR_ERCD(							  \
		((callevel_stat | (clmask)) == (clmask)), \
		E_OS_CALLEVEL							  \
		)
#define CHECK_NOFUNC(exp)				CHECK_ERROR_ERCD(exp, E_OS_NOFUNC)
#define CHECK_RESOURCE(exp)				CHECK_ERROR_ERCD(exp, E_OS_RESOURCE)
#define CHECK_STATE(exp)				CHECK_ERROR_ERCD(exp, E_OS_STATE)
#define D_CHECK_STATE(exp)				D_CHECK_ERROR_ERCD(exp, E_OS_STATE)
#define CHECK_VALUE(exp)				CHECK_ERROR_ERCD(exp, E_OS_VALUE)
#define CHECK_DISABLEDINT()										\
	CHECK_ERROR_ERCD(											\
		((callevel_stat &										\
		  (TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)) ==	\
		 TSYS_NULL												\
		),														\
		E_OS_DISABLEDINT										\
		)
#define CHECK_PARAM_POINTER(p_exp)		CHECK_ERROR_ERCD(((p_exp) != NULL), OS_E_PARAM_POINTER)
#define CHECK_ID(exp)					CHECK_ERROR_ERCD(exp, E_OS_ID)
#define D_CHECK_ID(exp)					D_CHECK_ERROR_ERCD(exp, E_OS_ID)
#define CHECK_IOC_ACCESS(exp)			CHECK_ERROR_ERCD(exp, IOC_E_NOK)
#define CHECK_IOC_CALLEVEL_DISABLEDINT(clmask)	  \
	CHECK_ERROR_ERCD(							  \
		((callevel_stat | (clmask)) == (clmask)), \
		IOC_E_NOK								  \
		)
#define CHECK_SERVICEID(exp)			CHECK_ERROR_ERCD(exp, E_OS_SERVICEID)
#define CHECK_RIGHT(btmp)														   \
	CHECK_ERROR_ERCD(															   \
		(run_trusted != FALSE) || ((p_runosap->p_osapinib->btptn & (btmp)) != 0U), \
		E_OS_ACCESS																   \
		)
#define CHECK_ILLEGAL_ADDRESS(exp)		CHECK_ERROR_ERCD(exp, E_OS_ILLEGAL_ADDRESS)
#define CHECK_MEM_WRITE(p_var, type)	CHECK_ERROR_ERCD((PROBE_MEM_WRITE(p_var, type) != FALSE), E_OS_ILLEGAL_ADDRESS)
#define CHECK_MEM_RW(p_var, type)		CHECK_ERROR_ERCD((PROBE_MEM_RW(p_var, type) != FALSE), E_OS_ILLEGAL_ADDRESS)
#define CHECK_NO_ERCD(exp)				CHECK_ERROR_NO_ERCD(exp)
#define CHECK_IOC_MEM_READ(p_var, size, alignsz) {			\
		if ((probe_memory_read(p_var, size) == FALSE) ||	\
			(((uintptr) (p_var) & (alignsz - 1U)) != 0U)) {	\
			ercd = E_OS_SYS_ASSERT_FATAL;					\
			goto exit_no_errorhook;							\
		}													\
}
#define CHECK_IOC_MEM_WRITE(p_var, size, alignsz) {			\
		if ((probe_memory_write(p_var, size) == FALSE) ||	\
			(((uintptr) (p_var) & (alignsz - 1U)) != 0U)) {	\
			ercd = E_OS_SYS_ASSERT_FATAL;					\
			goto exit_no_errorhook;							\
		}													\
}

#endif /* TOPPERS_CHECK_H */
